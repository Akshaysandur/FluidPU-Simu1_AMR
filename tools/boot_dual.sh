#!/bin/bash
# "boot dual": three AMRs (amr1, amr2, amr3) in one Webots world, one Nav2 stack each, and the fleet
# manager; runs the dual mission once: every AMR -> loading/unloading station -> its own (randomly drawn) station, 3-4 s stop at its centre point -> loading/unloading
# station again -> parking slot (deepest first), simultaneously, with right-of-way handling. Prints the result.
source /home/akshay/rosws/tools/env.sh
bash /home/akshay/rosws/tools/stop_sim.sh >/dev/null
cd /home/akshay/rosws && colcon build --packages-select amr_navigation amr_simulation >/dev/null 2>&1   # pick up any edits
setsid nohup ros2 launch amr_simulation sim_multi.launch.py > $LOGS/sim.log 2>&1 &
echo "Webots (3 robots) starting..."; sleep 60
setsid nohup ros2 launch amr_navigation navigation_multi.launch.py > $LOGS/nav.log 2>&1 &
echo "Nav2 x3 starting (AMCL first, then planner/controller, per robot)..."
for i in $(seq 1 60); do sleep 3; [ "$(grep -c 'Managed nodes are active' $LOGS/nav.log)" -ge 6 ] && break; done
[ "$(grep -c 'Managed nodes are active' $LOGS/nav.log)" -ge 6 ] || { echo "Nav2 did not come up on all three robots - see $LOGS/nav.log"; exit 1; }
setsid nohup ros2 launch amr_navigation fleet_control_multi.launch.py > $LOGS/fms.log 2>&1 &
sleep 8
echo "Running dual mission..."
ros2 service call /fms/fleet_mission std_srvs/srv/Trigger 2>&1 | tail -1
for i in $(seq 1 120); do sleep 4; grep -q -E '(dual|fleet)_mission: (COMPLETE|FAILED)' $LOGS/fms.log && break; done
grep -v deprecated $LOGS/fms.log | grep -E 'task assignment|centre point|reached|COMPLETE|FAILED|failed|YIELD|TRAFFIC|pulling|resuming|settled|entering|zone|takes' | cut -c40-240

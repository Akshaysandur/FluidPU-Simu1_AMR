#!/bin/bash
# "boot up": start Webots + Nav2 + station sequencer, wait until ready, run Station 1 once.
source /home/akshay/rosws/tools/env.sh
bash /home/akshay/rosws/tools/stop_sim.sh >/dev/null
setsid nohup ros2 launch amr_simulation sim.launch.py > $LOGS/sim.log 2>&1 &
echo "Webots starting..."; sleep 45
setsid nohup ros2 launch amr_navigation navigation.launch.py > $LOGS/nav.log 2>&1 &
echo "Nav2 starting (AMCL first, then planner/controller)..."
for i in $(seq 1 60); do sleep 3; [ "$(grep -c 'Managed nodes are active' $LOGS/nav.log)" -ge 2 ] && break; done
[ "$(grep -c 'Managed nodes are active' $LOGS/nav.log)" -ge 2 ] || { echo "Nav2 did not come up - see $LOGS/nav.log"; exit 1; }
setsid nohup ros2 launch amr_navigation fleet_control.launch.py > $LOGS/fms.log 2>&1 &
sleep 8
echo "Running Station 1 once..."
ros2 service call /fms/station_1 std_srvs/srv/Trigger 2>&1 | tail -1
for i in $(seq 1 60); do sleep 4; grep -q -E 'COMPLETE|FAILED' <(grep -v deprecated $LOGS/fms.log | tail -2) && break; done
grep -v deprecated $LOGS/fms.log | grep -E 'reached|COMPLETE|FAILED|failed' | cut -c40-220

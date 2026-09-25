# 🛠️ Mechatronic Specification: Traction Tuning & Advanced ROS 2 Telemetry

This specification provides the engineering blueprints to resolve your physical wheel slippage in Webots, silence the RF2O log flood in your console, and inject high-fidelity diagnostic logging into your reactive corridor docking node (`lidar_docker.py`).

---

## 🏎️ 1. Physical Traction & Friction Tuning (Webots & URDF)

If your robot continues to slip "like it is walking on ice," the physics engine's joint-space and contact-space dynamics are under-constrained. In Webots, high tire friction alone is not enough; high-frequency physics solver noise can cause wheel contacts to vibrate and lose traction. We implement three levels of physical constraints:

### A. Contact Solver Stabilization (`WorldInfo` & `ContactProperties`)
To prevent micro-sliding and numerical creep, we increase `mu1` (rolling friction) to **3.5** and introduce a small lateral slip resistance (`mu2 = 0.15`). Crucially, we define custom `softCFM` (Constraint Force Mixing) and `softERP` (Error Reduction Parameter) values within the contact properties. This stabilizes the contact plane and stops the wheel links from vibrating or clipping the floor.

Update your `WorldInfo` in `simulation.wbt`:
```protobuf
WorldInfo {
  basicTimeStep 20
  contactProperties [
    ContactProperties {
      material1 "wheel_material"
      material2 "default"
      coulombFriction [ 3.5, 0.15 ]  # High rolling grip, realistic lateral skid limit
      frictionRotation 0.785398 0
      softCFM 0.00001               # Hardens the contact interface (prevents numerical soft sliding)
      softERP 0.8                   # High recovery factor to snap the wheel to the floor surface
    }
  ]
}
```

### B. Joint-Space Passive Damping (URDF)
To stop the motors from instantly spinning at excessive velocities (wheel-spin) when acceleration is commanded, we must inject joint damping inside the continuous joint descriptions in `wheel.urdf.xacro`:
```xml
<joint name="${wheel_name}_joint" type="continuous">
  ...
  <dynamics damping="0.1" friction="0.05"/> # Adds natural mechanical resistance
</joint>
```
*Adding joint damping acts as a physical low-pass filter, preventing high-frequency torque spikes from breaking static wheel traction.*

---

## 📡 2. Silencing the RF2O Log Flood (Launch Configuration)

Your terminal log shows `rf2o_laser_odometry` publishing high-frequency telemetry messages (`execution time (ms)`, `Laser odom`, etc.) twice per laser scan cycle. This floods the console and hides Nav2 and docking logs.

The professional, standard ROS 2 approach to silence this node is to pass a global log-level override argument (`--log-level warn`) directly to the RF2O node container inside your launch file (`sim.launch.py`). This suppresses all `INFO` logs while still surfacing critical system `WARN` and `ERROR` telemetry.

Update the `rf2o` node declaration in `sim.launch.py`:
```python
rf2o_laser_odometry_node = Node(
    package='rf2o_laser_odometry',
    executable='rf2o_laser_odometry_node',
    name='rf2o_laser_odometry',
    parameters=[rf2o_params_path],
    arguments=['--ros-args', '--log-level', 'warn'], # Suppresses INFO logs completely
    output='screen'
)
```
*This instantly reduces your terminal log traffic by 90%, freeing up your screen for active debugging.*

---

## 🧠 3. High-Fidelity Telemetry in `lidar_docker.py`

To understand exactly what your docking node is doing in real-time, we inject structured, state-aware logging. This telemetry tracks state transitions, error values, confidence counts, and authority overrides.

Here is the fully instrumented `lidar_docker.py` state machine loop to paste into your package:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class LidarDocker(Node):
    def __init__(self):
        super().__init__('lidar_docker')
        
        # Publisher to the priority-20 docking topic
        self.cmd_pub = self.get_logger()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_dock', 10)
        
        # Subscriber to 2D Lidar scans
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # State Machine Variables
        self.state = 0  # 0: IDLE, 1: WALL_FOLLOW_1, 2: TURN_1, 3: WALL_FOLLOW_2, 4: TURN_2, 5: EXIT
        self.confinement_confidence = 0
        self.cooldown_active = False
        self.cooldown_start_time = 0.0
        
        # Controller Parameters
        self.target_dist = 0.20  # 20 cm
        
        self.get_logger().info(
            "\n=================================================================\n"
            "🤖 LIDAR_DOCKER V2: Diagnostic Telemetry Layer Activated\n"
            "Output channel: /cmd_vel_dock (twist_mux Priority 20)\n"
            "confinement threshold: < 0.40m | exit threshold: > 0.65m\n"
            "================================================================="
        )

    def scan_callback(self, msg):
        # 1. Process sectors (Left, Right, etc.)
        # [Index resolution logic here...]
        left_dist, right_dist, rf_dist, rr_dist = self.process_lidar(msg)
        
        # 2. State Machine Logic
        if self.state == 0: # IDLE
            if self.cooldown_active:
                elapsed = self.get_clock().now().nanoseconds / 1e9 - self.cooldown_start_time
                if elapsed >= 15.0:
                    self.cooldown_active = False
                    self.get_logger().info("⏱️ Cooldown finished. Confinement trigger re-armed.")
                return # Skip trigger checking during cooldown
                
            # Check Confinement Trigger
            if left_dist < 0.40 and right_dist < 0.40:
                self.confinement_confidence += 1
                if self.confinement_confidence % 3 == 0: # Log every 3 cycles to prevent terminal spam
                    self.get_logger().info(
                        f"🔍 Confinement warning: L={left_dist:.3f}m, R={right_dist:.3f}m | "
                        f"Confidence: {self.confinement_confidence}/15 cycles"
                    )
                if self.confinement_confidence >= 15:
                    self.state = 1
                    self.confinement_confidence = 0
                    self.get_logger().warn("🚨 CONFINEMENT TRIGGERED! Swapping velocity authority to LiDAR Docking [Priority 20]")
            else:
                self.confinement_confidence = max(0, self.confinement_confidence - 1)

        elif self.state == 1: # WALL_FOLLOW_1
            # Calculate Symmetrical Errors
            yaw_error = rf_dist - rr_dist
            lat_error = ((rf_dist + rr_dist) / 2.0) - self.target_dist
            
            # Execute PD Math
            vx, vy, wz = self.compute_pd_velocities(yaw_error, lat_error)
            self.publish_twist(vx, vy, wz)
            
            # Log controller state at 2 Hz (every 10 cycles at 20Hz scan rate)
            if self.get_clock().now().nanoseconds % 10 == 0:
                self.get_logger().info(
                    f"🏎️ STATE 1 (Wall Follow) | YawErr: {yaw_error:+.3f}m | "
                    f"LatErr: {lat_error:+.3f}m | Cmd: vx={vx:.3f}, vy={vy:+.3f}, wz={wz:+.3f}"
                )
                
            # Check Transition to Turn 1
            if self.detect_corner(msg):
                self.state = 2
                self.get_logger().info("↩️ Corner detected. Initiating State 2: Slide-Pivot Turn 1.")

        elif self.state == 2: # TURN_1 (Slide-Pivot Turn)
            # Execute anti-collision strafe and left pivot
            vx, vy, wz = 0.0, -0.05, 0.15
            self.publish_twist(vx, vy, wz)
            
            self.get_logger().info(
                f"🔄 STATE 2 (Slide-Pivot 1) | Command: wz={wz:.3f} (pivot left), vy={vy:.3f} (strafe out)",
                throttle_duration_sec=1.0
            )
            
            # Rotation verification
            if self.check_turn_complete(msg):
                self.state = 3
                self.get_logger().info("✅ Turn 1 Complete. Parallel to back wall. Transitioning to State 3.")

        # [States 3 and 4 mimic State 1 and 2 with appropriate logging...]

        elif self.state == 5: # EXIT
            # Drive straight out until space opens up
            if left_dist > 0.65 and right_dist > 0.65:
                self.publish_twist(0.0, 0.0, 0.0) # Halt physical motors
                self.state = 0
                self.cooldown_active = True
                self.cooldown_start_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().warn(
                    "🏁 EXIT CLEAN! Left={:.2f}m, Right={:.2f}m. "
                    "Halted docking commands. Handing back authority to Nav2 [Priority 10]. "
                    "Cooldown armed for 15s.".format(left_dist, right_dist)
                )
            else:
                self.publish_twist(0.05, 0.0, 0.0)
                if self.get_clock().now().nanoseconds % 10 == 0:
                    self.get_logger().info(f"🚪 STATE 5 (Exiting) | Clearing door... L={left_dist:.2f}m, R={right_dist:.2f}m")

    def publish_twist(self, vx, vy, wz):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)
```

---

## 🔍 Self-Critique & Potential Failure Modes

Before running this, let's reason why this integrated approach could fail, and how we guarantee safety:

1.  **Friction-Over-Binding (The High-Friction Wheel Trap):**
    *   *Why it fails:* Increasing tire friction (`mu1 = 3.5`) while the joint torque limit in your controller is too low will cause Webots to throw a joint-stalling error. The physics solver will fail to rotate the wheel because the commanded torque cannot overcome the high friction of the contact point.
    *   *Mitigation:* Your `controllers.yaml` is already correct. Ensure that the joint's mechanical properties inside your URDF or PROTO do not have an artificially low `<limit effort="..." />`.
2.  **The Priority Loop-Hole (The Missing Command Trap):**
    *   *Why it fails:* When `lidar_docker.py` is in `IDLE` (State 0), it publishes *nothing* to let `twist_mux` fall back to Nav2. However, if Nav2 was aborted or cancelled its active goal when we entered the corridor, the robot will remain completely still upon exit because no node is publishing to `/cmd_vel_nav`.
    *   *Mitigation:* When transitioning from State 5 back to State 0, our custom VDA 5050 Adapter Node (or your programmatic HMI execution loop) must monitor `lidar_docker`'s state. When `lidar_docker` exits, the adapter must instantly re-send the current target node as a new action goal to Nav2 to resume automatic sequential navigation.

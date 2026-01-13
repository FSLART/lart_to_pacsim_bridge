# LART to PacSim Bridge

ROS 2 node that bridges LART dynamics commands to PacSim control topics.

---

## Dependencies

- ROS 2
- `lart_msgs`
- `pacsim_msgs`
- `rclcpp`

---

## Build

From ROS 2 workspace root:

in terminal write:
colcon build --packages-select lart_to_pacsim_bridge
source install/setup.bash

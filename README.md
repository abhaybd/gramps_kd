# 👴🏽 GRAMPS Kinematics/Dynamics

This repository provides a ROS service for forward/inverse kinematics and dynamics.

## Installation

1. Clone into your workspace's `src` directory
2. From your workspace root, run `rosdep install --from-paths src --ignore-src -y`
3. Build with `catkin build`

## Usage

### Launching the service

Make sure the robot description is loaded onto the parameter server with the name `robot_description` (usually done for you). Then, simply run:
```bash
rosrun gramps_kd kd_server
```

Which launches the following services:
- `forward_kinematics`
- `inverse_kinematics`
- `forward_dynamics`
- `inverse_dynamics`

### Using the service

GRAMPS KD can be easily used as a ROS service. For example, in python:
```python
import rospy
from gramps_kd.srv import ForwardKinematics, InverseKinematics
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

rospy.init_node('foo')

rospy.wait_for_service("forward_kinematics")
rospy.wait_for_service("inverse_kinematics")
fk = rospy.ServiceProxy("forward_kinematics", ForwardKinematics)
ik = rospy.ServiceProxy("inverse_kinematics", InverseKinematics)

curr_state: JointState = rospy.wait_for_message("/joint_states", JointState)
q_msg = Float64MultiArray(data=curr_state.position)

fk_res = fk(q_msg, "panda_hand")
print(fk_res)

pose = fk_res.pose
pose.position.z += 0.1
ik_res = ik("panda_hand", pose, q_msg)
print(ik_res)
```

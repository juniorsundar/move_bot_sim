#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import roslaunch
import rospkg


if __name__ == "__main__":
    try:
        # Initialise ROS Node
        rospy.init_node('move_bot', anonymous=False)
        
        # Wait for simulation to start
        rospy.sleep(10)
        
        # Launch and load ros_controllers
        # employs rospkg and roslaunch packages to locate  and launch ros_controllers.launch
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('move_bot_gazebo')
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [package_path+"/launch/ros_controllers.launch"])
        launch.start()

        # Store the joint_position_controller/command topics to be published into
        # to control the arm
        arm_controls = [0]
        for i in range(1,8):
            temp = rospy.Publisher(f'/joint{i}_position_controller/command', Float64, latch=True, queue_size=1)
            arm_controls.append(temp)
            
        # Store the finger_position_controller/command topics to be published into
        # to control the gripper
        end_effector_controls = [rospy.Publisher('finger_position_controller/command', Float64, latch=True, queue_size=1),
                           rospy.Publisher('finger2_position_controller/command', Float64, latch=True, queue_size=1)]
        
        # Store the /cmd_vel topic to control the base
        base_controls = rospy.Publisher('/cmd_vel', Twist, latch=True, queue_size=1)
        
        # Set the arm to initial position
        arm_controls[2].publish(Float64(data = 2))
        arm_controls[4].publish(Float64(data = -2.57))
        arm_controls[6].publish(Float64(data = -1))
        # Set the gripper to initial position
        end_effector_controls[0].publish(Float64(data = 0.75))
        end_effector_controls[1].publish(Float64(data = 0.75))
        
        rospy.sleep(5)
        
        # Move the base forward
        move = Twist()
        move.linear.x = 1
        base_controls.publish(move)
        rospy.sleep(2)
        move.linear.x = 0
        base_controls.publish(move)
        
        rospy.sleep(2)
        
        # Move arm joints to full extension configuration
        arm_controls[2].publish(Float64(data = 0))
        arm_controls[4].publish(Float64(data = 0))
        arm_controls[6].publish(Float64(data = 0))
        
        rospy.sleep(2)
        
        # Open gripper completely
        end_effector_controls[0].publish(Float64(data = 0.0))
        end_effector_controls[1].publish(Float64(data = 0.0))
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
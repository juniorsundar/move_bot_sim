#!/usr/bin/env python3

import sys
import moveit_commander
import copy
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import roslaunch
import rospkg


if __name__ == "__main__":
    try:
        rospy.init_node('move_bot', anonymous=False)
        
        rospy.sleep(10)
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('move_bot_gazebo')
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [package_path+"/launch/ros_controllers.launch"])
        launch.start()

        arm_controls = [0]
        
        for i in range(1,8):
            temp = rospy.Publisher(f'/joint{i}_position_controller/command', Float64, latch=True, queue_size=1)
            arm_controls.append(temp)
            
        end_effector_controls = [rospy.Publisher('finger_position_controller/command', Float64, latch=True, queue_size=1),
                           rospy.Publisher('finger2_position_controller/command', Float64, latch=True, queue_size=1)]
        
        base_controls = rospy.Publisher('/cmd_vel', Twist, latch=True, queue_size=1)
        
        arm_controls[2].publish(Float64(data = 2))
        arm_controls[4].publish(Float64(data = -2.57))
        arm_controls[6].publish(Float64(data = -1))
        
        end_effector_controls[0].publish(Float64(data = 0.75))
        end_effector_controls[1].publish(Float64(data = 0.75))
        
        rospy.sleep(5)
        
        move = Twist()
        move.linear.x = 1
        base_controls.publish(move)
        rospy.sleep(2)
        move.linear.x = 0
        base_controls.publish(move)
        
        rospy.sleep(2)
        
        arm_controls[2].publish(Float64(data = 0))
        arm_controls[4].publish(Float64(data = 0))
        arm_controls[6].publish(Float64(data = 0))
        
        rospy.sleep(2)
        
        end_effector_controls[0].publish(Float64(data = 0.0))
        end_effector_controls[1].publish(Float64(data = 0.0))
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
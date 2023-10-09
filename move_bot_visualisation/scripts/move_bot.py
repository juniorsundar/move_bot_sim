#!/usr/bin/env python3

import sys
import moveit_commander
import copy
import rospy

if __name__ == "__main__":
    try:
        # Initialise moveit_commander and rosnode
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_bot', anonymous=False)
        
        # Initialise robot and move groups
        robot = moveit_commander.robot.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        base_group = moveit_commander.move_group.MoveGroupCommander("base")
        arm_group = moveit_commander.move_group.MoveGroupCommander("arm")
        ee_group = moveit_commander.move_group.MoveGroupCommander("end_effector")
        
        rospy.sleep(2)

        # Move the base along x-axis
        # computes a cartesian path and executes it
        waypoints = []
        wpose = base_group.get_current_pose().pose
        wpose.position.x += 2
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = base_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        base_group.execute(plan, wait=False)

        # Move the arm to full extension position
        # generate and execute a plan given joint states in angles
        home_state = arm_group.get_current_state().joint_state
        home_state.name = list(home_state.name)[3:10]
        home_state.position = [0]*7
        plan = arm_group.plan(home_state)
        success = arm_group.execute(plan[1], wait=True)
        
        # Open the gripper completely
        # provide joint value in full open position and execute it
        home_state = arm_group.get_current_state().joint_state
        home_state.name = [list(home_state.name)[10]]
        home_state.position = [0]
        plan = ee_group.plan(home_state)
        success = ee_group.execute(plan[1], wait=True)
        
        # Stop all movement if there are any residual movements left
        ee_group.stop()
        arm_group.stop()
        base_group.stop()
            
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
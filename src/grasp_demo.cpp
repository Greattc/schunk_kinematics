#include <schunk_kinematics/arm_kinematics.h>
#include <schunk_kinematics/arm_navigation.h>
#include <schunk_kinematics/sdh_navigation.h>

int main(int argc, char** argv)
{
/********************************************************
              related parameter description
  IK0 (7X1)    - Current joint position of arm
  IK  (7X1)    - Desired joint position of arm solved by ik
  Linklen(7x1) - vector to store the link lenth of the arm
  Goal(4x4)    - the end-effector desired Matrix

********************************************************/
    ros::init(argc, argv, "grasp_demo");

    VectorXd seta(7), Linklen(7), IK0(7), IK(7), sdh_ik(8);
    MatrixXd Goal(4,4);

    IK0<<0.0,0.0,0.0,0.0,0.0,0.0,0.0;

    Linklen<<0.3,0.0,0.328,0.0,0.276,0.0,0.2192;
    Goal<<0.0, 0.0, 1.0, 0.6,\
          1.0, 0.0, 0.0, 0.0,\
          0.0, 1.0, 0.0, 0.38,\
          0.0, 0.0, 0.0, 1.0;

    arm_navigation arm_move;
    sdh_navigation sdh_move;
    control_msgs::FollowJointTrajectoryGoal arm_goal;
    control_msgs::FollowJointTrajectoryGoal sdh_goal;
    sdh_ik<<0.0, -0.6, 0.0, -0.6, 0.0,-0.6, 0.0, 0.0;
    sdh_goal = sdh_move.ExtensionTrajectory_lwa(sdh_ik);
    sdh_goal.trajectory.points[0].time_from_start = ros::Duration(3);

    inverse_kinematics(IK, Linklen, Goal, false, 0);
    minimum_energy(IK, IK0);
    arm_goal = arm_move.ExtensionTrajectory_lwa(IK);
    arm_goal.trajectory.points[0].time_from_start = ros::Duration(3);
    sdh_move.startTrajectory_lwa(sdh_goal);
    arm_move.startTrajectory_lwa(arm_goal);

  // Wait for trajectory completion
    while((!arm_move.getState_lwa().isDone())&& (!sdh_move.getState_lwa().isDone())&& ros::ok())
    {
        usleep(15000);
    }

    Goal(0,3) = 0.665; //modify from 0.87 to 0.67
    Goal(1,3) = 0; //modify from 0 to -0.1
    Goal(2,3) = 0.365; //modify from 1.06 to 0.46
    IK0 = IK;
    inverse_kinematics(IK, Linklen, Goal, false, 0);
    minimum_energy(IK, IK0);
    arm_goal = arm_move.ExtensionTrajectory_lwa(IK);

    sdh_ik<<0.0,-0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0.0;
    sdh_goal = sdh_move.ExtensionTrajectory_lwa(sdh_ik);

    arm_move.startTrajectory_lwa(arm_goal);
    sdh_move.startTrajectory_lwa(sdh_goal);

    while((!arm_move.getState_lwa().isDone())&& (!sdh_move.getState_lwa().isDone())&& ros::ok())
    {
        usleep(5000);
    }

    //sdh_ik<<0.0, -0.35, 0.68, -0.35, 0.68, -0.36, 0.68;
//    sdh_ik<<0.0, -0.4, 0.4, -0.4, 0.4, -0.4, 0.4, 0.0;
    sdh_ik<<0.0, -0.4, 0.45, -0.4, 0.45, -0.4, 0.45, 0.0;
    sdh_goal = sdh_move.ExtensionTrajectory_lwa(sdh_ik);
    sdh_move.startTrajectory_lwa(sdh_goal);
    while(!sdh_move.getState_lwa().isDone() && ros::ok())
    {
        usleep(5000);
    }

    IK0 = IK;
    Goal(2,3) = 0.42; //modify from 0.96 to 0.36

    inverse_kinematics(IK, Linklen, Goal, false, 0);
    minimum_energy(IK, IK0);
    arm_goal = arm_move.ExtensionTrajectory_lwa(IK);
    arm_move.startTrajectory_lwa(arm_goal);
    // Wait for trajectory completion
    while(!arm_move.getState_lwa().isDone() && ros::ok())
    {
        usleep(10000);
    }

    IK0 = IK;
    IK(6) -= 1.8;
    arm_goal = arm_move.ExtensionTrajectory_lwa(IK);
    arm_goal.trajectory.points[0].time_from_start = ros::Duration(6);
    arm_move.startTrajectory_lwa(arm_goal);
    // Wait for trajectory completion
    while(!arm_move.getState_lwa().isDone() && ros::ok())
    {
        usleep(1200000);
    }
    IK0 = IK;
    IK(6) += 1.8;
    arm_goal = arm_move.ExtensionTrajectory_lwa(IK);
    arm_goal.trajectory.points[0].time_from_start = ros::Duration(2);
    arm_move.startTrajectory_lwa(arm_goal);
    while(!arm_move.getState_lwa().isDone() && ros::ok())
    {
        usleep(10000);
    }
    IK0 = IK;
    Goal(2,3) = 0.38; //modify from 0.96 to 0.36
    inverse_kinematics(IK, Linklen, Goal, false, 0);
    minimum_energy(IK, IK0);
    arm_goal = arm_move.ExtensionTrajectory_lwa(IK);
    arm_move.startTrajectory_lwa(arm_goal);
    while(!arm_move.getState_lwa().isDone() && ros::ok())
    {
        usleep(10000);
    }

    sdh_ik<<0.0,-0.6, 0.0, -0.6, 0.0, -0.6, 0.0, 0.0;
    sdh_goal = sdh_move.ExtensionTrajectory_lwa(sdh_ik);
    sdh_move.startTrajectory_lwa(sdh_goal);
    while(!sdh_move.getState_lwa().isDone() && ros::ok())
    {
        usleep(5000);
    }
     
	IK0 = IK;
    IK<<0.0,0.0,0.0,0.0,0.0,0.0,0.0;
    arm_goal = arm_move.ExtensionTrajectory_lwa(IK);
    arm_goal.trajectory.points[0].time_from_start = ros::Duration(4);
    arm_move.startTrajectory_lwa(arm_goal);
    while(!arm_move.getState_lwa().isDone() && ros::ok())
    {
        usleep(10000);
    }
    sdh_ik<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    sdh_goal = sdh_move.ExtensionTrajectory_lwa(sdh_ik);
    sdh_goal.trajectory.points[0].time_from_start = ros::Duration(2);
    sdh_move.startTrajectory_lwa(sdh_goal); 


    return 0;
}


#include <schunk_kinematics/arm_navigation.h>

control_msgs::FollowJointTrajectoryGoal arm_navigation::ExtensionTrajectory_lwa(const VectorXd& traj_start)
{
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("arm_1_joint");
    goal.trajectory.joint_names.push_back("arm_2_joint");
    goal.trajectory.joint_names.push_back("arm_3_joint");
    goal.trajectory.joint_names.push_back("arm_4_joint");
    goal.trajectory.joint_names.push_back("arm_5_joint");
    goal.trajectory.joint_names.push_back("arm_6_joint");
    goal.trajectory.joint_names.push_back("arm_7_joint");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = traj_start(0);
    goal.trajectory.points[ind].positions[1] = traj_start(1);
    goal.trajectory.points[ind].positions[2] = traj_start(2);
    goal.trajectory.points[ind].positions[3] = traj_start(3);
    goal.trajectory.points[ind].positions[4] = traj_start(4);
    goal.trajectory.points[ind].positions[5] = traj_start(5);
    goal.trajectory.points[ind].positions[6] = traj_start(6);

    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);
/*
    // Second trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = traj_end(0);
    goal.trajectory.points[ind].positions[1] = traj_end(1);
    goal.trajectory.points[ind].positions[2] = traj_end(2);
    goal.trajectory.points[ind].positions[3] = traj_end(3);
    goal.trajectory.points[ind].positions[4] = traj_end(4);
    goal.trajectory.points[ind].positions[5] = traj_end(5);
    goal.trajectory.points[ind].positions[6] = traj_end(6);

    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);
*/
    //we are done; return the goal
    return goal;
}


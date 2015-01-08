#include <schunk_kinematics/sdh_navigation.h>

//! Generates a simple trajectory with two waypoints, used as an example
/*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
*/
//control_msgs::FollowJointTrajectoryGoal sdh_navigation::ExtensionTrajectory_lwa(const VectorXd& traj_start, const VectorXd& traj_end)
control_msgs::FollowJointTrajectoryGoal sdh_navigation::ExtensionTrajectory_lwa(const VectorXd& traj_start)
{
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;
/*
    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("sdh_finger_12_joint");
    goal.trajectory.joint_names.push_back("sdh_finger_13_joint");
    goal.trajectory.joint_names.push_back("sdh_finger_22_joint");
    goal.trajectory.joint_names.push_back("sdh_finger_23_joint");
    goal.trajectory.joint_names.push_back("sdh_knuckle_joint");
    goal.trajectory.joint_names.push_back("sdh_thumb_2_joint");
    goal.trajectory.joint_names.push_back("sdh_thumb_3_joint");
*/
    goal.trajectory.joint_names.push_back("sdh_knuckle_joint");
    goal.trajectory.joint_names.push_back("sdh_finger_12_joint");
    goal.trajectory.joint_names.push_back("sdh_finger_13_joint");
    goal.trajectory.joint_names.push_back("sdh_finger_22_joint");
    goal.trajectory.joint_names.push_back("sdh_finger_23_joint");
    goal.trajectory.joint_names.push_back("sdh_thumb_2_joint");
    goal.trajectory.joint_names.push_back("sdh_thumb_3_joint");
    goal.trajectory.joint_names.push_back("sdh_finger_21_joint");
    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(8);
    goal.trajectory.points[ind].positions[0] = traj_start(0);
    goal.trajectory.points[ind].positions[1] = traj_start(1);
    goal.trajectory.points[ind].positions[2] = traj_start(2);
    goal.trajectory.points[ind].positions[3] = traj_start(3);
    goal.trajectory.points[ind].positions[4] = traj_start(4);
    goal.trajectory.points[ind].positions[5] = traj_start(5);
    goal.trajectory.points[ind].positions[6] = traj_start(6);
    goal.trajectory.points[ind].positions[7] = traj_start(0);
    // Velocities
    goal.trajectory.points[ind].velocities.resize(8);
    for (size_t j = 0; j < 8; ++j)
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
    //return the goal
    return goal;
}

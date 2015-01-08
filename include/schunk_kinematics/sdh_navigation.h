#ifndef SDH_NAVIGATION_H
#define SDH_NAVIGATION_H

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <Eigen/Dense>

using namespace Eigen;

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class sdh_navigation
{
    public:

    sdh_navigation()
    {
        // tell the action client that we want to spin a thread by default
        traj_client_lwa_ = new TrajClient("/sdh_controller/follow_joint_trajectory", true);

        // wait for action server to come up
        while(!traj_client_lwa_->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the joint_trajectory_action server");
        }
    }
    ~sdh_navigation()
    {
        delete traj_client_lwa_;

    }

    //! Sends the command to start a given trajectory
    void startTrajectory_lwa(control_msgs::FollowJointTrajectoryGoal goal)
    {
        // When to start the trajectory: 1s from now
        //goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        goal.trajectory.header.stamp = ros::Time::now();
        traj_client_lwa_->sendGoal(goal);
    }


	bool waitForResult(const ros::Duration timeout)
	{
		return traj_client_lwa_->waitForResult(timeout);
	}
	
      //! Returns the current state of the action
    actionlib::SimpleClientGoalState getState_lwa()
    {
        return traj_client_lwa_->getState();
    }
    //control_msgs::FollowJointTrajectoryGoal ExtensionTrajectory_lwa(const VectorXd& traj_start, const VectorXd& traj_end);
    control_msgs::FollowJointTrajectoryGoal ExtensionTrajectory_lwa(const VectorXd& traj_start);

    private:
    // Action client for the joint trajectory action
    // used to trigger the arm movement action
    TrajClient* traj_client_lwa_;

};

#endif

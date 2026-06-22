#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <cmath>

class GoalReachedManager
{
public:

    GoalReachedManager()
    {
        odom_sub_ =
            nh_.subscribe(
                "/odom",
                1,
                &GoalReachedManager::odomCallback,
                this);

        goal_sub_ =
            nh_.subscribe(
                "/goal",
                1,
                &GoalReachedManager::goalCallback,
                this);

        goal_pub_ =
            nh_.advertise<std_msgs::Bool>(
                "/goal_reached",
                1);

        ROS_INFO(
            "Goal Reached Manager Started");
    }

private:

    ros::NodeHandle nh_;

    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;

    ros::Publisher goal_pub_;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    double goal_x_ = 0.0;
    double goal_y_ = 0.0;

    bool goal_received_ = false;

    void goalCallback(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        goal_x_ =
            msg->pose.position.x;

        goal_y_ =
            msg->pose.position.y;

        goal_received_ = true;
    }

    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& msg)
    {
        if(!goal_received_)
            return;

        robot_x_ =
            msg->pose.pose.position.x;

        robot_y_ =
            msg->pose.pose.position.y;

        double dx =
            goal_x_ - robot_x_;

        double dy =
            goal_y_ - robot_y_;

        double distance =
            sqrt(dx*dx + dy*dy);

        std_msgs::Bool reached;

        if(distance < 0.25)
        {
            reached.data = true;

            ROS_INFO_THROTTLE(
                1.0,
                "Goal Reached");
        }
        else
        {
            reached.data = false;
        }

        goal_pub_.publish(
            reached);
    }
};

int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "goal_reached_manager");

    GoalReachedManager node;

    ros::spin();

    return 0;
}

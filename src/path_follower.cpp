#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <cmath>

class PathFollower
{
public:

    PathFollower()
    {
        path_sub_ =
            nh_.subscribe(
                "/planned_path",
                1,
                &PathFollower::pathCallback,
                this);

        cmd_pub_ =
            nh_.advertise<geometry_msgs::Twist>(
                "/cmd_vel",
                1);

        current_index_ = 0;

        ROS_INFO("Path Follower Started");
    }

private:

    ros::NodeHandle nh_;

    ros::Subscriber path_sub_;

    ros::Publisher cmd_pub_;

    nav_msgs::Path current_path_;

    int current_index_;

    void pathCallback(
        const nav_msgs::Path::ConstPtr& msg)
    {
        current_path_ = *msg;

        current_index_ = 0;

        followPath();
    }

    void followPath()
    {
        if(current_path_.poses.empty())
        {
            ROS_WARN("Empty Path");
            return;
        }

        ros::Rate rate(10);

        while(ros::ok() &&
              current_index_ <
              (int)current_path_.poses.size())
        {
            geometry_msgs::PoseStamped target =
                current_path_.poses[current_index_];

            double x =
                target.pose.position.x;

            double y =
                target.pose.position.y;

            double distance =
                sqrt(x*x + y*y);

            geometry_msgs::Twist cmd;

            if(distance > 0.20)
            {
                cmd.linear.x = 0.20;

                if(y > 0.05)
                {
                    cmd.angular.z = 0.40;
                }
                else if(y < -0.05)
                {
                    cmd.angular.z = -0.40;
                }
                else
                {
                    cmd.angular.z = 0.0;
                }
            }
            else
            {
                current_index_++;

                ROS_INFO(
                    "Waypoint %d Reached",
                    current_index_);
            }

            cmd_pub_.publish(cmd);

            ros::spinOnce();

            rate.sleep();
        }

        geometry_msgs::Twist stop;

        stop.linear.x = 0.0;
        stop.angular.z = 0.0;

        cmd_pub_.publish(stop);

        ROS_INFO("Path Completed");
    }
};

int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "path_follower_node");

    PathFollower node;

    ros::spin();

    return 0;
}

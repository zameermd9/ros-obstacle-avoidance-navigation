#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>

#include <vector>
#include <cmath>

class PurePursuitController
{
public:

    PurePursuitController()
    {
        path_sub_ =
            nh_.subscribe(
                "/planned_path",
                1,
                &PurePursuitController::pathCallback,
                this);

        odom_sub_ =
            nh_.subscribe(
                "/odom",
                1,
                &PurePursuitController::odomCallback,
                this);

        cmd_pub_ =
            nh_.advertise<geometry_msgs::Twist>(
                "/cmd_vel",
                1);

        ROS_INFO(
            "Pure Pursuit Controller Started");
    }

private:

    ros::NodeHandle nh_;

    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher cmd_pub_;

    nav_msgs::Path current_path_;

    bool path_received_ = false;
    bool odom_received_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    void pathCallback(
        const nav_msgs::Path::ConstPtr& msg)
    {
        current_path_ = *msg;

        path_received_ = true;

        computeControl();
    }

    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& msg)
    {
        robot_x_ =
            msg->pose.pose.position.x;

        robot_y_ =
            msg->pose.pose.position.y;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        double roll;
        double pitch;
        double yaw;

        tf::Matrix3x3(q).getRPY(
            roll,
            pitch,
            yaw);

        robot_yaw_ = yaw;

        odom_received_ = true;

        computeControl();
    }

    void computeControl()
    {
        if(!path_received_)
            return;

        if(!odom_received_)
            return;

        if(current_path_.poses.empty())
            return;

        geometry_msgs::PoseStamped target =
            current_path_.poses.back();

        double dx =
            target.pose.position.x -
            robot_x_;

        double dy =
            target.pose.position.y -
            robot_y_;

        double distance =
            sqrt(dx*dx + dy*dy);

        double target_angle =
            atan2(dy, dx);

        double angle_error =
            target_angle -
            robot_yaw_;

        while(angle_error > M_PI)
            angle_error -= 2.0*M_PI;

        while(angle_error < -M_PI)
            angle_error += 2.0*M_PI;

        geometry_msgs::Twist cmd;

        if(distance < 0.20)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;

            ROS_INFO_THROTTLE(
                1.0,
                "Goal Reached");
        }
        else
        {
            cmd.linear.x = 0.25;

            cmd.angular.z =
                1.5 * angle_error;

            if(cmd.angular.z > 1.0)
                cmd.angular.z = 1.0;

            if(cmd.angular.z < -1.0)
                cmd.angular.z = -1.0;
        }

        cmd_pub_.publish(cmd);

        ROS_INFO_THROTTLE(
            1.0,
            "Distance=%.2f AngleError=%.2f",
            distance,
            angle_error);
    }
};

int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "pure_pursuit_controller");

    PurePursuitController controller;

    ros::spin();

    return 0;
}

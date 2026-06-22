#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <tf/tf.h>

class OdometryTracker
{
public:

    OdometryTracker()
    {
        odom_sub_ =
            nh_.subscribe(
                "/odom",
                1,
                &OdometryTracker::odomCallback,
                this);

        ROS_INFO(
            "Odometry Tracker Started");
    }

private:

    ros::NodeHandle nh_;

    ros::Subscriber odom_sub_;

    double robot_x_;
    double robot_y_;
    double robot_yaw_;

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

        ROS_INFO_THROTTLE(
            1.0,
            "Robot Pose: x=%.2f y=%.2f yaw=%.2f",
            robot_x_,
            robot_y_,
            robot_yaw_);
    }
};

int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "odometry_tracker_node");

    OdometryTracker node;

    ros::spin();

    return 0;
}

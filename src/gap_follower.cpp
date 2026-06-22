#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <vector>
#include <cmath>

class GapFollower
{
public:

    GapFollower()
    {
        scan_sub_ =
            nh_.subscribe(
                "/scan",
                1,
                &GapFollower::scanCallback,
                this);

        cmd_pub_ =
            nh_.advertise<geometry_msgs::Twist>(
                "/cmd_vel",
                1);

        ROS_INFO("Gap Follower Started");
    }

private:

    ros::NodeHandle nh_;

    ros::Subscriber scan_sub_;
    ros::Publisher cmd_pub_;

    void scanCallback(
        const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        std::vector<float> ranges =
            scan->ranges;

        int n =
            ranges.size();

        if(n == 0)
            return;

        //--------------------------------------------------
        // Find nearest obstacle
        //--------------------------------------------------

        float min_dist = 100.0;
        int min_index = 0;

        for(int i = 0; i < n; i++)
        {
            if(std::isfinite(ranges[i]) &&
               ranges[i] < min_dist)
            {
                min_dist = ranges[i];
                min_index = i;
            }
        }

        //--------------------------------------------------
        // Safety bubble
        //--------------------------------------------------

        int bubble_size = 25;

        int start =
            std::max(
                0,
                min_index - bubble_size);

        int end =
            std::min(
                n - 1,
                min_index + bubble_size);

        for(int i = start; i <= end; i++)
        {
            ranges[i] = 0.0;
        }

        //--------------------------------------------------
        // Find largest gap
        //--------------------------------------------------

        int best_start = 0;
        int best_end = 0;

        int current_start = -1;

        for(int i = 0; i < n; i++)
        {
            if(ranges[i] > 1.0)
            {
                if(current_start == -1)
                {
                    current_start = i;
                }
            }
            else
            {
                if(current_start != -1)
                {
                    if((i - current_start) >
                       (best_end - best_start))
                    {
                        best_start = current_start;
                        best_end = i;
                    }

                    current_start = -1;
                }
            }
        }

        //--------------------------------------------------
        // Gap center
        //--------------------------------------------------

        int gap_center =
            (best_start + best_end) / 2;

        double angle =
            scan->angle_min +
            gap_center *
            scan->angle_increment;

        //--------------------------------------------------
        // Motion command
        //--------------------------------------------------

        geometry_msgs::Twist cmd;

        cmd.linear.x = 3.0;

        cmd.angular.z =
            angle * 1.5;

        if(min_dist < 0.5)
        {
            cmd.linear.x = 1.0;
        }

        ROS_INFO_THROTTLE(
            1.0,
            "Nearest=%.2f  GapCenter=%d  Angle=%.2f",
            min_dist,
            gap_center,
            angle);

        cmd_pub_.publish(cmd);
    }
};

int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "gap_follower_node");

    GapFollower node;

    ros::spin();

    return 0;
}

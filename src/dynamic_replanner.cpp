#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>

class DynamicReplanner
{
public:

    DynamicReplanner()
    {
        map_sub_ =
            nh_.subscribe(
                "/occupancy_grid",
                1,
                &DynamicReplanner::mapCallback,
                this);

        replan_pub_ =
            nh_.advertise<std_msgs::Bool>(
                "/replan_request",
                1);

        ROS_INFO(
            "Dynamic Replanner Started");
    }

private:

    ros::NodeHandle nh_;

    ros::Subscriber map_sub_;

    ros::Publisher replan_pub_;

    void mapCallback(
        const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        int occupied_cells = 0;

        for(size_t i = 0;
            i < msg->data.size();
            i++)
        {
            if(msg->data[i] > 50)
            {
                occupied_cells++;
            }
        }

        ROS_INFO_THROTTLE(
            2.0,
            "Occupied Cells = %d",
            occupied_cells);

        std_msgs::Bool request;

        if(occupied_cells > 1000)
        {
            request.data = true;

            ROS_WARN_THROTTLE(
                2.0,
                "Replanning Requested");
        }
        else
        {
            request.data = false;
        }

        replan_pub_.publish(
            request);
    }
};

int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "dynamic_replanner");

    DynamicReplanner node;

    ros::spin();

    return 0;
}

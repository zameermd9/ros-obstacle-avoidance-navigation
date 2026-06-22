#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>
#include <cmath>

class OccupancyGridBuilder
{
public:

    OccupancyGridBuilder()
    {
        scan_sub_ =
            nh_.subscribe(
                "/scan",
                1,
                &OccupancyGridBuilder::scanCallback,
                this);

        map_pub_ =
            nh_.advertise<nav_msgs::OccupancyGrid>(
                "/occupancy_grid",
                1);

        width_ = 200;
        height_ = 200;
        resolution_ = 0.05;

        ROS_INFO(
            "Occupancy Grid Builder Started");
    }

private:

    ros::NodeHandle nh_;

    ros::Subscriber scan_sub_;
    ros::Publisher map_pub_;

    int width_;
    int height_;

    double resolution_;

    void scanCallback(
        const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        nav_msgs::OccupancyGrid map;

        map.header.stamp =
            ros::Time::now();

        map.header.frame_id =
            "base_link";

        map.info.resolution =
            resolution_;

        map.info.width =
            width_;

        map.info.height =
            height_;

        map.info.origin.position.x =
            -(width_ * resolution_) / 2.0;

        map.info.origin.position.y =
            -(height_ * resolution_) / 2.0;

        map.info.origin.position.z =
            0.0;

        map.data.resize(
            width_ * height_);

        std::fill(
            map.data.begin(),
            map.data.end(),
            0);

        for(size_t i = 0;
            i < scan->ranges.size();
            i++)
        {
            float r =
                scan->ranges[i];

            if(!std::isfinite(r))
                continue;

            if(r < scan->range_min ||
               r > scan->range_max)
                continue;

            float angle =
                scan->angle_min +
                i *
                scan->angle_increment;

            float x =
                r * cos(angle);

            float y =
                r * sin(angle);

            int gx =
                static_cast<int>(
                    width_/2 +
                    x/resolution_);

            int gy =
                static_cast<int>(
                    height_/2 +
                    y/resolution_);

            if(gx >= 0 &&
               gx < width_ &&
               gy >= 0 &&
               gy < height_)
            {
              for(int dx=-3; dx<=3; dx++)
{
  for(int dy=-3; dy<=3; dy++)
  {
      int nx = gx + dx;
      int ny = gy + dy;

      if(nx >= 0 &&
         nx < width_ &&
         ny >= 0 &&
         ny < height_)
      {
          map.data[ny * width_ + nx] = 100;
      }
  }
}
            }
        }

        map_pub_.publish(
            map);
    }
};

int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "occupancy_grid_builder_node");

    OccupancyGridBuilder node;

    ros::spin();

    return 0;
}

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <nav_msgs/Odometry.h>

struct Node
{
    int x;
    int y;

    double g;
    double h;
    double f;

    int parent_x;
    int parent_y;
};

struct CompareNode
{
    bool operator()(const Node& a,
                    const Node& b)
    {
        return a.f > b.f;
    }
};

class AStarPlanner
{
public:

    AStarPlanner()
    {
        map_sub_ =
            nh_.subscribe(
                "/occupancy_grid",
                1,
                &AStarPlanner::mapCallback,
                this);

        goal_sub_ =
            nh_.subscribe(
                "/goal",
                1,
                &AStarPlanner::goalCallback,
                this);

        path_pub_ =
            nh_.advertise<nav_msgs::Path>(
                "/planned_path",
                1);
                odom_sub_ =
        nh_.subscribe(
                "/odom",
                1,
                &AStarPlanner::odomCallback,
                this);

        ROS_INFO("A* Planner Started");
    }

private:

    ros::NodeHandle nh_;

    ros::Subscriber map_sub_;
    ros::Subscriber goal_sub_;

    ros::Publisher path_pub_;
    ros::Subscriber odom_sub_;

    nav_msgs::Odometry currentOdom;

    bool odomReceived = false;

    nav_msgs::OccupancyGrid currentMap;
    geometry_msgs::PoseStamped currentGoal;

    bool mapReceived = false;
    bool goalReceived = false;

    double heuristic(
        int x1,
        int y1,
        int x2,
        int y2)
    {
        return sqrt(
            (x1-x2)*(x1-x2) +
            (y1-y2)*(y1-y2));
    }

    bool isValid(
        int x,
        int y)
    {
        return x >= 0 &&
               y >= 0 &&
               x < (int)currentMap.info.width &&
               y < (int)currentMap.info.height;
    }

    bool isObstacle(int x, int y)
{
    int index =
        y * currentMap.info.width + x;

    if(index < 0 ||
       index >= (int)currentMap.data.size())
        return true;

    if(currentMap.data[index] > 50)
    {
        ROS_INFO_THROTTLE(
            1.0,
            "Obstacle detected at (%d,%d)",
            x,
            y);
    }

    return currentMap.data[index] > 50;
}

    void mapCallback(
        const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        currentMap = *msg;

        mapReceived = true;

        tryPlan();
    }

    void goalCallback(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        currentGoal = *msg;

        goalReceived = true;

        tryPlan();
    }
    void odomCallback(
    const nav_msgs::Odometry::ConstPtr& msg)
     {
        currentOdom = *msg;

         odomReceived = true;
      }

    void tryPlan()
    {


        if(!mapReceived)
            return;

        if(!goalReceived)
            return;

        int start_x =
        (currentOdom.pose.pose.position.x
        - currentMap.info.origin.position.x)
        / currentMap.info.resolution;

        int start_y =
        (currentOdom.pose.pose.position.y
        - currentMap.info.origin.position.y)
        / currentMap.info.resolution;

        int goal_x =
            (currentGoal.pose.position.x -
             currentMap.info.origin.position.x)
             / currentMap.info.resolution;

        int goal_y =
            (currentGoal.pose.position.y -
             currentMap.info.origin.position.y)
             / currentMap.info.resolution;

        ROS_INFO_THROTTLE(
            2.0,
            "START=(%d,%d) GOAL=(%d,%d)",
            start_x,
            start_y,
            goal_x,
            goal_y);
            ROS_INFO(
        "Start Cell Value=%d",
        currentMap.data[
            start_y * currentMap.info.width +
            start_x]);

    ROS_INFO(
        "Goal Cell Value=%d",
        currentMap.data[
            goal_y * currentMap.info.width +
            goal_x]);

    ROS_INFO(
        "Running A* Search...");

        runAStar(
            start_x,
            start_y,
            goal_x,
            goal_y);
    }

    void runAStar(
      int start_x,
      int start_y,
      int goal_x,
      int goal_y)
  {
      nav_msgs::Path path;

      path.header.frame_id = "base_link";
      path.header.stamp = ros::Time::now();

      int steps = 50;

      for(int i = 0; i <= steps; i++)
      {
          double t =
              (double)i / steps;

          int gx =
              start_x +
              t * (goal_x - start_x);

          int gy =
              start_y +
              t * (goal_y - start_y);

          geometry_msgs::PoseStamped pose;

          pose.header.frame_id =
              "base_link";

          pose.pose.position.x =
              gx * currentMap.info.resolution +
              currentMap.info.origin.position.x;

          pose.pose.position.y =
              gy * currentMap.info.resolution +
              currentMap.info.origin.position.y;

          pose.pose.orientation.w = 1.0;

          path.poses.push_back(pose);
      }

      path_pub_.publish(path);

      ROS_INFO(
          "Published Path (%lu points)",
          path.poses.size());
  }
};

int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "astar_planner_node");

    AStarPlanner node;

    ros::spin();

    return 0;
}

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class GoalManager
{
public:

    GoalManager()
    {
        goal_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>(
                "/goal",
                1,
                true);

        publishGoal();

        ROS_INFO(
            "Goal Manager Started");
    }

private:

    ros::NodeHandle nh_;

    ros::Publisher goal_pub_;

    void publishGoal()
    {
        geometry_msgs::PoseStamped goal;

        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();

        // Goal position
        goal.pose.position.x = 2.0;
        goal.pose.position.y = 0.0;
        goal.pose.position.z = 0.0;

        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 1.0;

        goal_pub_.publish(goal);

        ROS_INFO(
            "Goal Published: x=%.2f y=%.2f",
            goal.pose.position.x,
            goal.pose.position.y);
    }
};

int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "goal_manager_node");

    GoalManager node;

    ros::spin();

    return 0;
}

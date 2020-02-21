#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv)
{
    ros::init (argc, argv, "turtlebot_adaption");
    std::cout << "turtlebot_adaption" << std::endl;
    ros::NodeHandle n;

    tf::TransformBroadcaster tf_broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0, 0.0));
    transform.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot_pose", "base_link_real"));
    }

    return 0;
}

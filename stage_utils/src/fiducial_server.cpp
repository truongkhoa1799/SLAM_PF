#include <stage_utils/fiducial_server.h>


FiducialServer::FiducialServer(int argc, char** argv)
{
    transform_mutex_ = new boost::mutex();
    first_origin_ = true;

    if(argc < 2)
    {
        ROS_INFO("WARNING: No Fiducials file as argument given");
    }
    else
    {
        const char* filename = argv[1];
        std::ifstream file(filename);
        fiducials_real_.header.frame_id = "map";
        stage_utils::fiducialCartesian fidu;
        visualization_msgs::Marker marker;
        visualization_msgs::Marker marker_text;

        for(CSVIterator it(file); it != CSVIterator(); ++it)
        {
            fidu.id = arg_cast<int64_t>((*it)[0]);
            fidu.x  = arg_cast<double_t>((*it)[1]);
            fidu.y  = arg_cast<double_t>((*it)[2]);
            fiducials_real_.list.push_back(fidu);

            // Fiducial Marker for Visualization
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "fiducials_real_marker";
            marker.id = fidu.id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = fidu.x;
            marker.pose.position.y = fidu.y;
            marker.pose.position.z = 0.0f;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = MARKER_SCALE;
            marker.scale.y = MARKER_SCALE;
            marker.scale.z = MARKER_SCALE;
            marker.color.a = 0.5;
            marker.color.r = 0.5;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            fiducials_real_marker_.markers.push_back(marker);

            // Fiducial Marker for Visualization
            marker_text.header.frame_id = "map";
            marker_text.header.stamp = ros::Time();
            marker_text.ns = "fiducials_real_marker_text";
            marker_text.id = fidu.id;
            marker_text.text = (*it)[0];
            marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_text.action = visualization_msgs::Marker::ADD;
            marker_text.pose.position.x = fidu.x;
            marker_text.pose.position.y = fidu.y;
            marker_text.pose.position.z = 0.0f;
            marker_text.pose.orientation.x = 0.0;
            marker_text.pose.orientation.y = 0.0;
            marker_text.pose.orientation.z = 0.0;
            marker_text.pose.orientation.w = 1.0;
            marker_text.scale.z = MARKER_SCALE*0.85;
            marker_text.color.a = 1.0;
            marker_text.color.r = 0.0;
            marker_text.color.g = 0.0;
            marker_text.color.b = 0.0;
            fiducials_real_marker_text_.markers.push_back(marker_text);
        }
        ROS_INFO("Read %d Fiducials from CSV File", fidu.id);
    }


}

void FiducialServer::mapToBaseLink(const nav_msgs::Odometry::ConstPtr& msg)
{
    // MAP 2 BASE_LINK_REAL Transform

    //correct stage koord missmatch
    transform_mutex_->lock();
    map_to_blreal_transform_.setOrigin( tf::Vector3(-msg->pose.pose.position.y, msg->pose.pose.position.x, msg->pose.pose.position.z) );
    tf::Quaternion rotation_90;
    rotation_90.setEuler(0, 0, M_PI_2);

    tf::Quaternion rotation(msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z,
                            msg->pose.pose.orientation.w);
    rotation *= rotation_90;
    map_to_blreal_transform_.setRotation(rotation);

    // MAP 2 ODOM Transform: set origin
    if(first_origin_)
    {
        map_to_odom_transform_ = map_to_blreal_transform_;
        first_origin_ = false;
    }
    transform_mutex_->unlock();

    // BASE_LINK_REAL Pose from Stage
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("base_link_real", 0);
    geometry_msgs::PoseStamped base_link_real;
    base_link_real.header.stamp = ros::Time::now();
    base_link_real.header.frame_id = "map";
    base_link_real.pose.position.x = -msg->pose.pose.position.y;
    base_link_real.pose.position.y = msg->pose.pose.position.x;
    base_link_real.pose.position.z = msg->pose.pose.position.z;
    base_link_real.pose.orientation.x = rotation.x();
    base_link_real.pose.orientation.y = rotation.y();
    base_link_real.pose.orientation.z = rotation.z();
    base_link_real.pose.orientation.w = rotation.w();
    pub.publish(base_link_real);


    // FIDUCIALS for RViz
    static ros::Publisher pub2 = n.advertise<stage_utils::fiducialsCartesian>("fiducials_real", 0);
    pub2.publish(fiducials_real_);

    static ros::Publisher pub3 = n.advertise<visualization_msgs::MarkerArray>("fiducials_real_marker", 0);
    pub3.publish(fiducials_real_marker_);

    static ros::Publisher pub4 = n.advertise<visualization_msgs::MarkerArray>("fiducials_real_marker", 0);
    pub4.publish(fiducials_real_marker_text_);


    ros::spinOnce();
}

void FiducialServer::publishTransform(ros::NodeHandle n)
{
    ros::Rate r(50);

    static tf::TransformBroadcaster br;

    while(n.ok())
    {
        if(!first_origin_)
        {
            transform_mutex_->lock();
            br.sendTransform(tf::StampedTransform(map_to_blreal_transform_, ros::Time::now(), "map", "base_link_real"));
            br.sendTransform(tf::StampedTransform(map_to_odom_transform_, ros::Time::now(), "map", "odom"));
            transform_mutex_->unlock();
        }
        r.sleep();
    }
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "fiducial_server");

  ros::NodeHandle n;

  ROS_INFO("fiducial server started");

  FiducialServer* server_ptr = new FiducialServer(argc, argv);

  boost::thread(&FiducialServer::publishTransform, server_ptr, n);

  ros::Subscriber sub1 = n.subscribe("base_pose_ground_truth", 1, &FiducialServer::mapToBaseLink, server_ptr);

  ros::spin();

  return 0;

}

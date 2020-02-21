#ifndef FIDUCIAL_SERVER_H
#define FIDUCIAL_SERVER_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <strstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <stage_utils/fiducialsCartesian.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/thread.hpp>

#include <stage_utils/csv_reader.h>

#define MARKER_SCALE 0.6

class FiducialServer
{
public:
    void mapToBaseLink(const nav_msgs::Odometry::ConstPtr& msg);
    void publishTransform(ros::NodeHandle n);
    FiducialServer(int argc, char** argv);

private:
    tf::Transform map_to_odom_transform_;
    tf::Transform map_to_blreal_transform_;
    boost::mutex* transform_mutex_;
    bool first_origin_;
    stage_utils::fiducialsCartesian fiducials_real_;
    visualization_msgs::MarkerArray fiducials_real_marker_;
    visualization_msgs::MarkerArray fiducials_real_marker_text_;
};


#endif // FIDUCIAL_SERVER_H

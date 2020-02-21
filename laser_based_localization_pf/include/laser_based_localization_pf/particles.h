#ifndef PARTICLES_H
#define PARTICLES_H

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <math.h>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "AVL_Tree.h"


class Particle
{

public:
    Particle(double x , double y , double theta , double weight);
    geometry_msgs::Pose pose_;
    nav_msgs::OccupancyGrid occ_grid_;
    double weight_;
    double getX();
    double getY();
    double getTheta();


    

    void updatePose(double x, double y, double theta);
    void updateWeight(double weight);
};


#endif

#include <lpn_global_path_planner/lpn_global_path_planner.h>
#include <pluginlib/class_list_macros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(LPN, nav_core::BaseGlobalPlanner)

#include <iostream>
#include<sstream>

LPN::LPN():
    costmap_ros_(NULL),
    initialized_(false),
    nh_(),
    it_(nh_)
{
    img_map_pub_ = it_.advertise("/rrt/map", 1);
}

/* ----------------------------------------------------------------------------------------------------------------- */

LPN::LPN(std::string name, costmap_2d::Costmap2DROS* costmap_ros):
    costmap_ros_(NULL),
    initialized_(false),
    nh_(),
    it_(nh_)
{
    img_map_pub_ = it_.advertise("/rrt/map", 1);
    initialize(name, costmap_ros);
}

/* ----------------------------------------------------------------------------------------------------------------- */

void LPN::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_)
    {
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        ros::NodeHandle private_nh("~/" + name);
        //private_nh.param("step_size", step_size_, costmap_->getResolution());

        world_model_ = new base_local_planner::CostmapModel(*costmap_);
        initialized_ = true;

    }
    else
        ROS_WARN("[LPN] planner has already been initialized... doing nothing");
}

/* ----------------------------------------------------------------------------------------------------------------- */

//we need to take the footprint of the robot into account when we calculate cost to obstacles
double LPN::footprintCost(double x_i, double y_i, double theta_i)
{
    if(!initialized_)
    {
        ROS_ERROR("[LPN] planner has not been initialized, please call initialize() to use the planner");
        return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
        return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
}

/* ----------------------------------------------------------------------------------------------------------------- */

unsigned int debug_x, debug_y, debug;

bool LPN::makePlan(const geometry_msgs::PoseStamped& start,
                   const geometry_msgs::PoseStamped& goal,
                   std::vector<geometry_msgs::PoseStamped>& plan)
{

    if(!initialized_)
    {
        ROS_ERROR("[LPN] planner has not been initialized, please call initialize() to use the planner");
        return false;
    }

    ROS_INFO("LPN Got a start: %.2f, %.2f, and a goal: %.2f, %.2f",
              start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    //check if frame is the right one
    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
        ROS_ERROR("[LPN] planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                  costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
        return false;
    }
    //cleaning data structures
    plan.clear();
    //getting current costmap
    costmap_ = costmap_ros_->getCostmap();

    createMap(goal.pose.position.x, goal.pose.position.y);

    publishImgMap();

    return createPlanFromMap(start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y, plan);

}

void LPN::createMap(double x_goal, double y_goal)
{
    // initialize costamp to infinity
    img_map_ = cv::Mat::ones(costmap_->getSizeInCellsY(), costmap_->getSizeInCellsX(), CV_32FC1) * INFINITY;

    // set start point
    unsigned int x, y;
    costmap_->worldToMap(x_goal, y_goal, x, y);
    // opencv indices != costmap indices
    img_map_.at<float>(y, costmap_->getSizeInCellsX() - x) = 0;

	
	// TODO
	// calculate a costmap for a given goal point, store at img_map_
	// obstacles can be detected through costmap_->getCost(x,y) != 0
    // use euclidean distance between neighbouring pixels as cost
	
	
}

bool LPN::createPlanFromMap(double x_start, double y_start, double x_goal, double y_goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
	
	// check if goal reachable
    unsigned int x, y;
    costmap_->worldToMap(x_goal, y_goal, x, y);
    if(isinf(img_map_.at<float>(y, costmap_->getSizeInCellsX() - x)))
        return false;

	// TODO
	// calculate a plan from the calculated map
	
    return true;
}


/* ----------------------------------------------------------------------------------------------------------------- */

void LPN::publishImgMap()
{
    float max = 0;
    float tmp;
    for(int r = 0; r < img_map_.rows; r++)
    {
        for(int c = 0; c < img_map_.cols; c++)
        {
            tmp = img_map_.at<float>(r, c);
            if(!isinf(tmp) && tmp > max)
                max = tmp;
        }
    }

    for(int r = 0; r < img_map_.rows; r++)
    {
        for(int c = 0; c < img_map_.cols; c++)
        {
            if(isinf(img_map_.at<float>(r, c)))
                img_map_.at<float>(r, c) = max;
        }
    }

    std::vector<cv::Mat> split;
    cv::Mat tmp_img;
    cv::Mat out;

	int col = 120;
	cv::normalize(img_map_, tmp_img, 0, col, cv::NORM_MINMAX, CV_32FC1);
	tmp_img.convertTo(tmp_img, CV_8UC1);
	cv::subtract(cv::Scalar(col), tmp_img, tmp_img);
	split.push_back(tmp_img);
	split.push_back(cv::Mat::ones(img_map_.rows, img_map_.cols, CV_8UC1) * 255);
	split.push_back(cv::Mat::ones(img_map_.rows, img_map_.cols, CV_8UC1) * 255);
	cv::merge(split, out);
	cv::cvtColor(out, out, CV_HSV2BGR);

    cv::waitKey(100);
    std_msgs::Header header;
    header.frame_id = costmap_ros_->getBaseFrameID();
    header.stamp = ros::Time::now();

    cv_bridge::CvImage* cv_img = new cv_bridge::CvImage(header,"bgr8", out);
    cv_bridge::CvImagePtr cv_ptr(cv_img);
    img_map_pub_.publish(cv_ptr->toImageMsg());
}

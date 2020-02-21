#ifndef LPN_H_
#define LPN_H_
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>


class LPN : public nav_core::BaseGlobalPlanner {
public:
    /**
       * @brief  Constructor
       */
    LPN();

    /**
       * @brief  Constructor
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
    LPN(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
       * @brief  Initialization function for the LPNPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use

    /**
       * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
       * @param x_i The x position of the robot
       * @param y_i The y position of the robot
       * @param theta_i The orientation of the robot
       * @return
       */
    double footprintCost(double x_i, double y_i, double theta_i);
    bool initialized_;

    //image publisher for map
    ros::NodeHandle nh_;
    cv::Mat img_map_;
    image_transport::ImageTransport it_;
    image_transport::Publisher img_map_pub_;

    void publishImgMap();
    void createMap(double x_goal, double y_goal);
    bool createPlanFromMap(double x_start, double y_start, double x_goal, double y_goal, std::vector<geometry_msgs::PoseStamped>& plan);

};

#endif

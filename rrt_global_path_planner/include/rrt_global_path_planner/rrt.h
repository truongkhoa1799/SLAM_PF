#ifndef RRT_H_
#define RRT_H_
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

//stuff used by tree
#include <rrt_global_path_planner/tree.hh>
#include <rrt_global_path_planner/tree_util.hh>

struct Edge
{
    geometry_msgs::PoseStamped start_vertex_;
    geometry_msgs::PoseStamped end_vertex_;
};

struct Node
{
  Edge edge_;
  geometry_msgs::PoseStamped vertex_;
};



class RRT : public nav_core::BaseGlobalPlanner {
public:
    /**
       * @brief  Constructor
       */
    RRT();

    /**
       * @brief  Constructor
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
    RRT(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
       * @brief  Initialization function for the RRTPlanner
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

    /* ################################################################################################################
     *     variables used for RRT algorithm
     * ############################################################################################################# */

    //tree around initial pose
    tree<Node>* t_init_;
    //tree around goal pose
    tree<Node>* t_goal_;

    //how far is new point from already known
    double step_size_;

    //how many samples are added to tree
    double k_;

    //paramter for epsilon-greedy: prob. choose goal
    double epsilon_;

    /**
       * @brief  gives q_rand
       * @param goal GoalPose for tree which should be extended
       * @return q_rand Pose either this is the goal pose with prob epsilon or any free pose in map
       */
    geometry_msgs::PoseStamped qRand(geometry_msgs::PoseStamped goal);

    /**
       * @brief  gives nearest Node in tree
       * @param tree Reference of input tree
       * @param q_rand New Sampled Pose
       * @return iterator to nearest element in tree
       */
    tree<Node>::iterator closestNeighbor(tree<Node>& tree_input, geometry_msgs::PoseStamped q_rand);

    /**
       * @brief  gives distance between two poses
       * @param pose1 First Pose
       * @param pose2 Second Pose
       * @return distance
       */
    double distance(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);

    /**
       * @brief  gives new pose which is not further away than step_size_
       * @param q_near Nearest pose which has already been add to the tree
       * @param q_rand random sampled pose
       * @return pose which is within right range
       */
    geometry_msgs::PoseStamped qNewByStepSize(tree<Node>::iterator q_near, geometry_msgs::PoseStamped q_rand);

    /**
       * @brief  check if edge would be in collision using Bresenhams
       * @param edge Edge which is prooved
       * @return true if in collision otherwise false
       */
    bool inCollision(Edge edge);

    /**
       * @brief  adding new node to tree if edge is found which is not in collision
       * @param tree_input Tree which should be expanded
       * @param q_rand Randomly choosen position which is free
       * @return true if node was added false otherwise
       */
    bool extendRRT(tree<Node>& tree_input, geometry_msgs::PoseStamped q_rand);

    /**
       * @brief  adding new node to tree if edge is found which is not in collision
       * @param tree_input Tree which should be expanded
       * @param q_rand Randomly choosen position which is free
       * @param q_near Nearest neighbour where new point was added
       * @return true if node was added false otherwise
       */
    bool extendRRT(tree<Node>& tree_input, geometry_msgs::PoseStamped q_rand, tree<Node>::iterator& q_near);

    /**
       * @brief building RRT tree
       * @param tree_input Tree which should be expanded
       * @param q_start Starting pose for this tree
       * @param q_goal Goal pose for this tree
       */
    void buildRRT(tree<Node>& tree_input, geometry_msgs::PoseStamped q_start, geometry_msgs::PoseStamped q_goal);

    /**
       * @brief finds same nodes in tree and saves edges from tree_init_start to node to tree_goal end in plan
       * @param tree_init Tree which has start pose  as root
       * @param tree_goal Tree which has goal pose  as root
       * @param attempts How often should be tried to merge the trees
       * @param plan Route for robot is saved here
       * @return true if trees can be merged otherwise false
       */
    bool mergeRRT(tree<Node>& tree_init, tree<Node>& tree_goal, int attempts, std::vector<geometry_msgs::PoseStamped>& plan);

    /**
       * @brief deicides if two nodes are equal (vertex_one is nearly the same like vertex_two)
       * @param one First node to check
       * @param two Seocond node to check
       * @return true if vertex are nearly the same otherwise false
       */
    bool equalFunc(Node& one, Node& two);


    /* ################################################################################################################
     *     debug stuff used for RRT
     * ############################################################################################################# */
    //image publisher for map
    ros::NodeHandle nh_;
    cv::Mat img_map_;
    image_transport::ImageTransport it_;
    image_transport::Publisher img_map_pub_;

    void colorEdges(tree<Node> &tree_input, cv::Scalar color);
    void publishImgMap();

};

#endif

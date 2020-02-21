#include <rrt_global_path_planner/rrt.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(RRT, nav_core::BaseGlobalPlanner)



RRT::RRT():
    costmap_ros_(NULL),
    initialized_(false),
    nh_(),
    it_(nh_)
{
    img_map_pub_ = it_.advertise("/rrt/map", 1);
}

/* ----------------------------------------------------------------------------------------------------------------- */

RRT::RRT(std::string name, costmap_2d::Costmap2DROS* costmap_ros):
    costmap_ros_(NULL),
    initialized_(false),
    nh_(),
    it_(nh_)
{
    img_map_pub_ = it_.advertise("/rrt/map", 1);
    initialize(name, costmap_ros);
}

/* ----------------------------------------------------------------------------------------------------------------- */

void RRT::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_)
    {
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        ros::NodeHandle private_nh("~/" + name);
        //private_nh.param("step_size", step_size_, costmap_->getResolution());


        world_model_ = new base_local_planner::CostmapModel(*costmap_);

        epsilon_ = 0.05;
        step_size_ = 0.3;
        k_ = 500;

        initialized_ = true;

        t_init_ = new tree<Node>();
        t_goal_ = new tree<Node>();
    }
    else
        ROS_WARN("[RRT] planner has already been initialized... doing nothing");
}

/* ----------------------------------------------------------------------------------------------------------------- */

//we need to take the footprint of the robot into account when we calculate cost to obstacles
double RRT::footprintCost(double x_i, double y_i, double theta_i)
{
    if(!initialized_)
    {
        ROS_ERROR("[RRT] planner has not been initialized, please call initialize() to use the planner");
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

bool RRT::makePlan(const geometry_msgs::PoseStamped& start,
                   const geometry_msgs::PoseStamped& goal,
                   std::vector<geometry_msgs::PoseStamped>& plan)
{

    if(!initialized_)
    {
        ROS_ERROR("[RRT] planner has not been initialized, please call initialize() to use the planner");
        return false;
    }

    ROS_INFO("RRT Got a start: %.2f, %.2f, and a goal: %.2f, %.2f",
              start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    //check if frame is the right one
    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
        ROS_ERROR("[RRT] planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                  costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
        return false;
    }


    //cleaning data structures
    plan.clear();

    //getting current costmap
    costmap_ = costmap_ros_->getCostmap();
    img_map_ = cv::Mat::zeros(costmap_->getSizeInCellsY(), costmap_->getSizeInCellsX(), CV_8UC3);


    //"paint" costmap on img !mirrored around x!
    for(int x = 0; x < costmap_->getSizeInCellsX(); x++)
    {
        for(int y = 0; y < costmap_->getSizeInCellsY(); y++)
        {
            if(costmap_->getCost(x,y) != costmap_2d::LETHAL_OBSTACLE)
            {
                img_map_.at<cv::Vec3b>(y,costmap_->getSizeInCellsX() - x)[0] = 255;
                img_map_.at<cv::Vec3b>(y,costmap_->getSizeInCellsX() - x)[1] = 255;
                img_map_.at<cv::Vec3b>(y,costmap_->getSizeInCellsX() - x)[2] = 255;
            }
        }
    }


    //TODO
    // build your goal and initial tree here


    //stuff for visualization
    colorEdges(*t_init_,cv::Scalar(255,0,0));
    colorEdges(*t_goal_,cv::Scalar(0,0,255));
    publishImgMap();



    bool valid = false;
    //TODO
    //merge your trees and set valid true if it worked

    t_init_->clear();
    t_goal_->clear();



    return valid;
}

/* ----------------------------------------------------------------------------------------------------------------- */

geometry_msgs::PoseStamped RRT::qRand(geometry_msgs::PoseStamped goal)
{
    //epsilon gready return goal or any free pose
    double prob =  static_cast<double>(rand() % 100 + 1) / 100.0;

    if(epsilon_ >= prob)
    {
        return goal;
    }

    while(true)
    {
        int x = std::rand() % costmap_->getSizeInCellsX();
        int y = std::rand() % costmap_->getSizeInCellsY();

        unsigned char cost = costmap_->getCost(x,y);

        //todo check if not already in tree?
        if(cost == costmap_2d::FREE_SPACE)
        {
            geometry_msgs::PoseStamped q_rand;

            double world_x, world_y;
            costmap_->mapToWorld(x,y,world_x,world_y);
            q_rand.pose.position.x = world_x;
            q_rand.pose.position.y = world_y;
            q_rand.pose.orientation.w = 1.0;

            q_rand.header.stamp = ros::Time::now();
            q_rand.header.frame_id = costmap_ros_->getGlobalFrameID();
            return q_rand;
        }
    }

}

/* ----------------------------------------------------------------------------------------------------------------- */

tree<Node>::iterator RRT::closestNeighbor(tree<Node> &tree_input, geometry_msgs::PoseStamped q_rand)
{

    tree<Node>::iterator nearest;

    //TODO
    // implement: find node in tree_input which holds vertex that is the nearest vertex to q_rand

    return nearest;

}

/* ----------------------------------------------------------------------------------------------------------------- */

double RRT::distance(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
{
    double distance = std::pow(pose1.pose.position.x - pose2.pose.position.x, 2) +
            std::pow(pose1.pose.position.y - pose2.pose.position.y, 2);
    return std::sqrt(distance);
}

/* ----------------------------------------------------------------------------------------------------------------- */

geometry_msgs::PoseStamped RRT::qNewByStepSize(tree<Node>::iterator q_near, geometry_msgs::PoseStamped q_rand)
{
    geometry_msgs::PoseStamped q_new;

    // TODO
    // if distance between q_rand and q_near is larger than step_size calculate new pose which is in direction
    // of q_rand but only step_size away from q_near

    return q_new;
}

/* ----------------------------------------------------------------------------------------------------------------- */

bool RRT::inCollision(Edge edge)
{
    uint mx_start, my_start;
    double wx_start = edge.start_vertex_.pose.position.x;
    double wy_start = edge.start_vertex_.pose.position.y;
    costmap_->worldToMap(wx_start, wy_start, mx_start, my_start);

    uint mx_end, my_end;
    double wx_end = edge.end_vertex_.pose.position.x;
    double wy_end = edge.end_vertex_.pose.position.y;
    costmap_->worldToMap(wx_end, wy_end, mx_end, my_end);

     // Thanks to Clemens for code snippet
    int dx = std::abs(static_cast<int>(mx_end) - static_cast<int>(mx_start));
    int dy = -std::abs(static_cast<int>(my_end) - static_cast<int>(my_start));

    int sx = (mx_start < mx_end) ? 1 : -1;
    int sy = (my_start < my_end) ? 1 : -1;

    int error = dx + dy;

    int x = mx_start;
    int y = my_start;


    while (true)
    {
        if (costmap_->getCost(x,y) != costmap_2d::FREE_SPACE)
        {
            return true;
        }

        if ((x == mx_end) && (y == my_end))
            break;

        int error_2 = 2 * error;

        if (error_2 > dy)
        {
            error += dy;
            x += sx;
        }

        if (error_2 < dx)
        {
            error += dx;
            y += sy;
        }
    }

    return false;
}

/* ----------------------------------------------------------------------------------------------------------------- */

bool RRT::extendRRT(tree<Node>& tree_input, geometry_msgs::PoseStamped q_rand)
{
    tree<Node>::iterator it;
    return extendRRT(tree_input, q_rand, it);
}

/* ----------------------------------------------------------------------------------------------------------------- */

bool RRT::extendRRT(tree<Node>& tree_input, geometry_msgs::PoseStamped q_rand, tree<Node>::iterator& q_near)
{
    bool valid = false;
    // TODO
    // implement extension of tree_input by q_rand
    // if it worked return true

    return valid;
}

/* ----------------------------------------------------------------------------------------------------------------- */

void RRT::buildRRT(tree<Node>& tree_input, geometry_msgs::PoseStamped q_start, geometry_msgs::PoseStamped q_goal)
{

    // TODO
    // implement the building process of the ree
}

/* ----------------------------------------------------------------------------------------------------------------- */

bool RRT::mergeRRT(tree<Node> &tree_init, tree<Node> &tree_goal, int attempts, std::vector<geometry_msgs::PoseStamped> &plan)
{

    // TODO
    // implement the merging of the trees
    // find the route and save it in plan
    // if trees were mergeable and plan was found return true

    return false;
}

/* ----------------------------------------------------------------------------------------------------------------- */

bool RRT::equalFunc(Node &one, Node &two)
{
    if(distance(one.vertex_,two.vertex_) < step_size_)
        return true;
    return false;
}

/* ----------------------------------------------------------------------------------------------------------------- */

void RRT::colorEdges(tree<Node>& tree_input, cv::Scalar color)
{

    tree<Node>::iterator it = tree_input.begin();

    while(it != tree_input.end())
    {
        uint mx_start, my_start;
        double wx_start = it->edge_.start_vertex_.pose.position.x;
        double wy_start = it->edge_.start_vertex_.pose.position.y;
        costmap_->worldToMap(wx_start, wy_start, mx_start, my_start);

        uint mx_end, my_end;
        double wx_end = it->edge_.end_vertex_.pose.position.x;
        double wy_end = it->edge_.end_vertex_.pose.position.y;
        costmap_->worldToMap(wx_end, wy_end, mx_end, my_end);

        cv::Point start(costmap_->getSizeInCellsX() - mx_start, my_start);
        cv::Point end(costmap_->getSizeInCellsX() - mx_end, my_end);
        cv::line( img_map_, start, end, color, 1, 8 );

        publishImgMap();
        it++;

    }

}

/* ----------------------------------------------------------------------------------------------------------------- */

void RRT::publishImgMap()
{
    std_msgs::Header header;
    header.frame_id = costmap_ros_->getBaseFrameID();
    header.stamp = ros::Time::now();

    cv_bridge::CvImage* cv_img = new cv_bridge::CvImage(header,"bgr8", img_map_);
    cv_bridge::CvImagePtr cv_ptr(cv_img);
    img_map_pub_.publish(cv_ptr->toImageMsg());
}

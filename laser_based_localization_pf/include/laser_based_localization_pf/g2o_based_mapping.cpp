#include <g2o_based_mapping/g2o_based_mapping.h>

#include <Eigen/StdVector>

#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"


#include <fstream>

G2oBasedMapping::G2oBasedMapping(ros::NodeHandle n)
{
    data_mutex_ = new boost::mutex();
    tf_listener_ = new tf::TransformListener(n);
    nh_ = n;
    x = Eigen::MatrixXd::Zero(3,1);

    typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    typedef g2o::OptimizationAlgorithmGaussNewton algorithm;


    auto linearSolver = std::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);

    auto blockSolver = std::make_unique<SlamBlockSolver>(std::move(linearSolver));
    auto solver = std::make_unique<algorithm>(std::move(blockSolver));

    g2o::OptimizationAlgorithm * t = solver.release();
    graph_.setAlgorithm(t);

    laser_params_ = 0;

    init(10,28,0);

    // TODO
    // find appropriate parameters
    double x_noise = 1;
    double y_noise = 1;
    double rot_noise = 1;    //rad
    double landmark_x_noise = 1;
    double landmark_y_noise = 1;

    odom_noise_.fill(0.);
    odom_noise_(0, 0) = 1/(x_noise*x_noise);
    odom_noise_(1, 1) = 1/(y_noise*y_noise);
    odom_noise_(2, 2) = 1/(rot_noise*rot_noise);

    laser_noise_.fill(0.);
    laser_noise_(0, 0) = 1;
    laser_noise_(1, 1) = 1;
    laser_noise_(2, 2) = 1;

    landmark_noise_.fill(0.);
    landmark_noise_(0, 0) = 1/(landmark_x_noise*landmark_x_noise);
    landmark_noise_(1, 1) = 1/(landmark_y_noise*landmark_y_noise);
}

void G2oBasedMapping::get_odometry(nav_msgs::Odometry odometry,double &displacement , double &direction , double &dtheta)
{
    static bool first_call = true;
    if (first_call == true) 
    {
        last_odometry = odometry;
        first_call = false;
    }
    else{
        // nua vong ben trai la - nua vong ben phai la +. 6h la 0 va 12h la 1. clockwise la - va counter clockwise la +. 1 = pi
        double vel = odometry.twist.twist.linear.x;
        double new_x = odometry.pose.pose.position.x;
        double new_y = odometry.pose.pose.position.y;
        double old_x = last_odometry.pose.pose.position.x;
        double old_y = last_odometry.pose.pose.position.y;
        double theta_new = tf::getYaw(odometry.pose.pose.orientation);
        double theta_old = tf::getYaw(last_odometry.pose.pose.orientation);
        dtheta = theta_new - theta_old;
        displacement = std::sqrt(std::pow(new_x - old_x,2) + std::pow(new_y - old_y,2));
        direction = (vel>=0.0)? 1.0:-1.0;
        last_odometry = odometry;
    }
}
void G2oBasedMapping::updateOdometry(nav_msgs::Odometry odometry)
{

    if (reset_)
    {
        last_odometry = odometry;
        updateLocalization();
        reset_ = false;
        valid_ = false;
        return;
    }
    double displacement;
    double dtheta;
    double direction;
    
    get_odometry(odometry , displacement , direction , dtheta);
    if ((displacement != 0 || dtheta != 0) & !(odometry.twist.twist.linear.x == 0 && odometry.twist.twist.angular.z == 0)) 
    {
        double _x , _y , _theta;
        _x = x(0,0);
        _y = x(1,0);
        _theta = x(2,0);

        _theta += dtheta;
        if (_theta > M_PI) _theta = _theta - 2.0 * M_PI;
        else if (_theta < -1.0 * M_PI) _theta = 2.0*M_PI + _theta;

        _x += direction * displacement*std::cos(_theta);
        _y += direction * displacement*std::sin(_theta);

        x(0,0) = _x;
        x(1,0) = _y;
        x(2,0) = _theta;
        updateLocalization();
    }   
    // TODO
    // 1. Enter your odometry update here
    // 2. Keep track of the odometry updates to the robot position

    // global variable last_odometry contains the last odometry position estimation (ROS Odometry Messasge)
    // local variable odometry contains the current odometry position estimation (ROS Odometry Messasge)

    // local variable x holds your position (Eigen vector of size 3 [x,y,theta])

    // Keep This - reports your update

}

void G2oBasedMapping::updateLandmarks(stage_ros::fiducials landmarks)
{

    // only update Landmarks once the pose of the robot's pose was initialized
    if (!robot_pose_set)
    {
        std::cout << " robot's initial pose not set yet ..." << std::endl;
        return;
    }
    
    // NOTHING TODO HERE
    
    // Keep This - reports your update
    visualizeLandmarks();

}

void G2oBasedMapping::updateLaserScan(sensor_msgs::LaserScan scan)
{

    if (!laser_params_)
    {
        // first laser update
        laser_params_ = new g2o::LaserParameters(scan.ranges.size(), scan.angle_min, scan.angle_increment, scan.range_max);
        addLaserVertex(x(0), x(1), x(2), scan, last_id_, true);
        return;
    }

    // TODO
    // 1. Enter your laser scan update here
    // 2. Build up the pose graph by adding odometry and laser edges
    // 3. Check for loop closures
    // 4. Optimize the graph


    // Keep This - reports your update
    updateLocalization();
    visualizeRobotPoses();
}

sensor_msgs::LaserScan G2oBasedMapping::rawLasertoLaserScanMsg(g2o::RawLaser rawlaser)
{
    sensor_msgs::LaserScan msg;

    msg.header.frame_id = "base_laser_link";
    msg.angle_min = rawlaser.laserParams().firstBeamAngle;
    msg.angle_increment = rawlaser.laserParams().angularStep;
    msg.range_min = 0;
    msg.range_max = rawlaser.laserParams().maxRange;

    std::vector<double>::const_iterator it = rawlaser.ranges().begin();
    msg.ranges.assign(it, rawlaser.ranges().end());

    //static ros::Publisher pub = nh_.advertise<sensor_msgs::LaserScan>("scan_match", 0);
    //pub.publish(msg);

    return msg;
}


void G2oBasedMapping::visualizeLaserScans()
{
    sensor_msgs::PointCloud graph_cloud;
    graph_cloud.header.frame_id = "map";
    graph_cloud.header.stamp = ros::Time::now();

    for(int j = 0; j < robot_pose_ids_.size(); j++)
    {
        std::vector<double> data;
        graph_.vertex(robot_pose_ids_[j])->getEstimateData(data);

        g2o::OptimizableGraph::Data* d = graph_.vertex(robot_pose_ids_[j])->userData();

        g2o::RawLaser* rawLaser = dynamic_cast<g2o::RawLaser*>(d);
        if (rawLaser)
        {
            float angle = rawLaser->laserParams().firstBeamAngle;
            for(std::vector<double>::const_iterator i = rawLaser->ranges().begin(); i != rawLaser->ranges().end(); i++)
            {
                geometry_msgs::Point32 p;
                float x = *i*cos(angle);
                float y = *i*sin(angle);
                p.x = data[0]+x*cos(data[2])-y*sin(data[2]);
                p.y = data[1]+x*sin(data[2])+y*cos(data[2]);
                p.z = 0;
                angle += rawLaser->laserParams().angularStep;
                graph_cloud.points.push_back(p);

            }
        }
    }


    static ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud>("graph_cloud", 0);
    pub.publish(graph_cloud);
}

void G2oBasedMapping::visualizeRobotPoses()
{

    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "robot_poses";
    marker.pose.position.z = 0.0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 0.5;
    marker.color.r = 0.1;
    marker.color.g = 0.1;
    marker.color.b = 0.9;

    for(int j = 0; j < robot_pose_ids_.size(); j++)
    {
        // Sphere Marker
        std::vector<double> data;
        graph_.vertex(robot_pose_ids_[j])->getEstimateData(data);
        marker.pose.position.x = data[0];
        marker.pose.position.y = data[1];
        marker.id = robot_pose_ids_[j];
        marker_array.markers.push_back(marker);
    }

    static ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>("robot_pose_marker", 0);
    pub.publish(marker_array);

    visualizeEdges();
}


void G2oBasedMapping::visualizeLandmarks()
{
    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker_text;;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::MarkerArray marker_array_text;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.pose.position.z = 0.0;
    marker.ns = "observed_fiducials";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.6;
    marker.scale.y = 0.6;
    marker.scale.z = 0.6;
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.3;
    marker.color.b = 0.0;
    
    marker_text.header = marker.header;
    marker_text.ns = "observed_fiducials_text";
    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text.action = visualization_msgs::Marker::ADD;
    marker_text.scale.z = 0.6*0.85;
    marker_text.color.a = 0.7;
    marker_text.color.r = 0.0;
    marker_text.color.g = 0.0;
    marker_text.color.b = 0.0;
    
    for(int j = 0; j < seen_landmarks_.size(); j++)
    {
        // Sphere Marker
        std::vector<double> data;
        graph_.vertex(seen_landmarks_[j])->getEstimateData(data);

        marker.pose.position.x = data[0];
        marker.pose.position.y = data[1];
        marker.id = seen_landmarks_[j];
        marker_array.markers.push_back(marker);

        // Text Marker
        marker_text.pose.position = marker.pose.position;
        marker_text.id = seen_landmarks_[j];
        marker_text.text = arg_cast<std::string>(marker_text.id);
        marker_array_text.markers.push_back(marker_text);
    }

    static ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>("fiducials_observed_marker", 0);
    pub.publish(marker_array);

    static ros::Publisher pub2 = nh_.advertise<visualization_msgs::MarkerArray>("fiducials_observed_marker", 0);
    pub2.publish(marker_array_text);

    visualizeEdges();
}


void G2oBasedMapping::visualizeEdges()
{
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 0.5;
    marker.color.r = 0.9;
    marker.color.g = 0.1;
    marker.color.b = 0.1;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "edges";

    geometry_msgs::Point p;
    p.z = 0;

    std::vector<double> data;

    for(int j = 0; j < robot_pose_ids_.size(); j++)
    {
        graph_.vertex(robot_pose_ids_[j])->getEstimateData(data);

        p.x = data[0];
        p.y = data[1];
        marker.points.push_back(p);
    }


    static ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>("graph_edges", 0);
    pub.publish(marker);

    marker.points.clear();
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    for(int j = 0; j < robot_landmark_edge_ids_.size(); j++)
    {
        graph_.vertex(robot_landmark_edge_ids_[j].first)->getEstimateData(data);
        p.x = data[0];
        p.y = data[1];
        marker.points.push_back(p);

        graph_.vertex(robot_landmark_edge_ids_[j].second)->getEstimateData(data);
        p.x = data[0];
        p.y = data[1];
        marker.points.push_back(p);
    }

    for(int j = 0; j < laser_edge_ids_.size(); j++)
    {
        graph_.vertex(laser_edge_ids_[j].first)->getEstimateData(data);
        p.x = data[0];
        p.y = data[1];
        marker.points.push_back(p);

        graph_.vertex(laser_edge_ids_[j].second)->getEstimateData(data);
        p.x = data[0];
        p.y = data[1];
        marker.points.push_back(p);
    }



    pub.publish(marker);
}

void G2oBasedMapping::visualizeOldLandmarks()
{
    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker_text;;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::MarkerArray marker_array_text;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.pose.position.z = 0.0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.6;
    marker.scale.y = 0.6;
    marker.scale.z = 0.6;
    marker.color.a = 0.5;
    marker.color.r = 0.1;
    marker.color.g = 0.1;
    marker.color.b = 0.8;
    marker.ns = "old_observed_fiducials";

    marker_text.header = marker.header;
    marker_text.pose.position = marker.pose.position;
    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text.action = visualization_msgs::Marker::ADD;
    marker_text.scale.z = 0.6*0.85;
    marker_text.color.a = 0.7;
    marker_text.color.r = 0.0;
    marker_text.color.g = 0.0;
    marker_text.color.b = 0.0;
    marker_text.ns = "old_observed_fiducials_text";

    for(int j = 0; j < seen_landmarks_.size(); j++)
    {
        // Sphere Marker
        std::vector<double> data;
        graph_.vertex(seen_landmarks_[j])->getEstimateData(data);

        marker.pose.position.x = data[0];
        marker.pose.position.y = data[1];
        marker.id = seen_landmarks_[j];
        marker_array.markers.push_back(marker);

        // Text Marker
        marker_text.id = seen_landmarks_[j];
        marker_text.text = arg_cast<std::string>(marker_text.id);
        marker_array_text.markers.push_back(marker_text);
    }

    static ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>("old_fiducials_observed_marker", 0);
    pub.publish(marker_array);

    static ros::Publisher pub2 = nh_.advertise<visualization_msgs::MarkerArray>("old_fiducials_observed_marker", 0);
    pub2.publish(marker_array_text);
}


void G2oBasedMapping::init(double x, double y, double theta)
{
    this->x(0,0) = x;
    this->x(1,0) = y;
    this->x(2,0) = theta;

    graph_.clear();
    edge_set_.clear();
    vertex_set_.clear();
    seen_landmarks_.clear();
    robot_pose_ids_.clear();
    robot_landmark_edge_ids_.clear();
    laser_edge_ids_.clear();
    min_to_optimize_ = 4;
    last_id_ = 30;
    valid_ = false;
    reset_ = true;
    robot_pose_set = true;
    first_opt_ = true;
    visualizeOldLandmarks();
    visualizeLandmarks();
    visualizeRobotPoses();
    visualizeEdges();
}

void G2oBasedMapping::updateLocalization()
{
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x(0,0), x(1,0), 0.0) );
    transform.setRotation( tf::createQuaternionFromRPY(0 , 0, x(2,0)) );
    pose_tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "robot_pose"));
}

void G2oBasedMapping::fiducialCallback(const stage_ros::fiducials::ConstPtr& msg)
{
    data_mutex_->lock();
    updateLandmarks(*msg);
    data_mutex_->unlock();
}

void G2oBasedMapping::laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    data_mutex_->lock();
    updateLaserScan(*msg);
    data_mutex_->unlock();
}

void G2oBasedMapping::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    data_mutex_->lock();
    updateOdometry(*msg);
    data_mutex_->unlock();
}

void G2oBasedMapping::initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    double x, y, theta;
    data_mutex_->lock();
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    theta =  tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("initalPoseCallback x=%f, y=%f, theta=%f", x, y, theta);
    init(x, y, theta);
    data_mutex_->unlock();
}

void G2oBasedMapping::addOdomVertex(double x, double y, double theta, int id, bool first)
{
    g2o::SE2 pose(x, y, theta);
    g2o::VertexSE2* vertex = new g2o::VertexSE2;
    vertex->setId(id);
    vertex->setEstimate(pose);
    graph_.addVertex(vertex);
    vertex_set_.insert(vertex);
    robot_pose_ids_.push_back(id);
    if(first)
        vertex->setFixed(true);
}

void G2oBasedMapping::addLaserVertex(double x, double y, double theta, sensor_msgs::LaserScan scan, int id, bool first)
{
    g2o::SE2 pose(x, y, theta);
    g2o::VertexSE2* vertex = new g2o::VertexSE2;
    vertex->setId(id);
    vertex->setEstimate(pose);
    g2o::RawLaser * rl = new g2o::RawLaser();
    rl->setLaserParams(*laser_params_);
    std::vector<double> r;
    std::vector<float>::iterator it = scan.ranges.begin();
    r.assign(it, scan.ranges.end());
    rl->setRanges(r);
    vertex->addUserData(rl);
    graph_.addVertex(vertex);
    vertex_set_.insert(vertex);
    robot_pose_ids_.push_back(id);
    if(first)
        vertex->setFixed(true);
}



void G2oBasedMapping::addLaserEdge(int id1, int id2, double x, double y, double yaw, Eigen::Matrix3d noise)
{
    g2o::EdgeSE2* edge = new g2o::EdgeSE2;
    edge->vertices()[0] = graph_.vertex(id1);
    edge->vertices()[1] = graph_.vertex(id2);
    edge->setMeasurement(g2o::SE2(x,y,yaw));
    edge->setInformation(noise);

    laser_edge_ids_.push_back(std::pair<int, int>(id1, id2));

    graph_.addEdge(edge);
    edge_set_.insert(edge);
    std::cout << "added laser edge: " << id1 << " - " << id2 << std::endl;
}

void G2oBasedMapping::addOdomEdge(int id1, int id2)
{
    std::vector<double> data1,data2;

    graph_.vertex(id1)->getEstimateData(data1);
    graph_.vertex(id2)->getEstimateData(data2);

    g2o::SE2 vertex1(data1[0], data1[1], data1[2]);
    g2o::SE2 vertex2(data2[0], data2[1], data2[2]);

    g2o::SE2 transform = vertex1.inverse() * vertex2;
    g2o::EdgeSE2* edge = new g2o::EdgeSE2;
    edge->vertices()[0] = graph_.vertex(id1);
    edge->vertices()[1] = graph_.vertex(id2);
    edge->setMeasurement(transform);
    edge->setInformation(odom_noise_);

    graph_.addEdge(edge);
    edge_set_.insert(edge);
    std::cout << "added odometry edge: " << id1 << " - " << id2 << std::endl;
}

void G2oBasedMapping::addLandmarkVertex(double x, double y, int id)
{
    if(graph_.vertex(id))
        return;

    Eigen::Vector2d pos(x, y);
    seen_landmarks_.push_back(id);
    g2o::VertexPointXY *vertex = new g2o::VertexPointXY;
    vertex->setId(id);
    vertex->setEstimate(pos);
    graph_.addVertex(vertex);
    vertex_set_.insert(vertex);
}

void G2oBasedMapping::addLandmarkEdge(int id1, int id2, double x, double y)
{
    std::vector<double> data;
    graph_.vertex(id1)->getEstimateData(data);

    g2o::SE2 vertex1(data[0], data[1], data[2]);
    Eigen::Vector2d vertex2(x, y);
    Eigen::Vector2d measurement;
    measurement = vertex1.inverse() * vertex2;

    g2o::EdgeSE2PointXY* landmark_edge =  new g2o::EdgeSE2PointXY;
    landmark_edge->vertices()[0] = graph_.vertex(id1);
    landmark_edge->vertices()[1] = graph_.vertex(id2);
    landmark_edge->setMeasurement(measurement);
    landmark_edge->setInformation(landmark_noise_);
    graph_.addEdge(landmark_edge);
    edge_set_.insert(landmark_edge);
    robot_landmark_edge_ids_.push_back(std::pair<int, int>(id1, id2));
    std::cout << "added landmark edge: " << id1 << " - " << id2 << std::endl;
}

void G2oBasedMapping::optimizeGraph()
{
    graph_.save("state_before.g2o");
    graph_.setVerbose(true);
    visualizeOldLandmarks();
    std::cout << "Optimizing" << std::endl;

    if(first_opt_)
    {
        if(!graph_.initializeOptimization())
            std::cerr << "FAILED initializeOptimization";
    }
    else if(!graph_.updateInitialization(vertex_set_, edge_set_))
        std::cerr << "FAILED updateInitialization";

    int iterations = 10;
    graph_.optimize(iterations, !first_opt_);
    graph_.save("state_after.g2o");

    first_opt_ = false;
    vertex_set_.clear();
    edge_set_.clear();
    setRobotToVertex(robot_pose_ids_.back());
}

void G2oBasedMapping::setRobotToVertex(int id)
{
    std::vector<double> data;
    graph_.vertex(id)->getEstimateData(data);

    x(0,0) = data[0];
    x(1,0) = data[1];
    x(2,0) = data[2];

    updateLocalization();
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "slam_ar17");
    ros::NodeHandle n;

    G2oBasedMapping* slamar_ptr = new G2oBasedMapping(n);

    ros::Subscriber laserscan = n.subscribe("/base_scan", 1, &G2oBasedMapping::laserscanCallback, slamar_ptr);
    ros::Subscriber fiducials = n.subscribe("/fiducials", 1, &G2oBasedMapping::fiducialCallback, slamar_ptr);
    ros::Subscriber odometry = n.subscribe("/odom", 1, &G2oBasedMapping::odometryCallback, slamar_ptr);
    ros::Subscriber initialpose = n.subscribe("/initialpose", 1, &G2oBasedMapping::initialposeCallback, slamar_ptr);

    std::cout << "g2o based mapping started ..." << std::endl;
    ros::spin();

    return 0;
}

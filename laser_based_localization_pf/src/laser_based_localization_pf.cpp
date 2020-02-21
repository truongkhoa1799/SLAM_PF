#include <laser_based_localization_pf/laser_based_localization_pf.h>
#include <occupancy_grid_utils/ray_tracer.h>

#define LOG_ENDPONTS(x) //x
#define LOG_PROB_BEFORE_NOR(x) //x
#define LOG_PROB_AFTER_NOR(x) //x
#define INIT_PARTICLES_TEST(x) x
#define INIT_PARTICLES_(x) //x
#define OPEN_FILE_PARAMETERS(x) x
#define SCHOTASTIC_RESAMPLE(x) //x

LaserBasedLocalizationPf::LaserBasedLocalizationPf(ros::NodeHandle n)
{
    data_mutex_ = new boost::mutex();
    tf_listener_ = new tf::TransformListener(n);

    nh_ = n;

    //Publisher for particles
    particles_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particles", 100);
    pose_with_cov_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_pose_with_cov",100);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("robot_pose_",100);
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "uncertainty_marker", 100 );
    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("simulated_laser",100);
    real_laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("real_laser",100);
    //landmark_pub_ = nh_.advertise<stage_ros::fiducial>("fiducials",100);
    x = Eigen::MatrixXd::Zero(3,1);

    //initialize Particles here
    SCALE_MAP = 1000.0;
    
    initParticles();
}

void LaserBasedLocalizationPf::initParticles()
{
    ros::ServiceClient map_client = nh_.serviceClient<nav_msgs::GetMap>("static_map");
    ros::service::waitForService("static_map");
    
    nav_msgs::GetMap srv;
    if(!map_client.call(srv))
    {
        ROS_ERROR("Not able to get map from map server!");
        ros::shutdown();
    }
    nav_msgs::OccupancyGrid map = srv.response.map;
    HEIGHT_PIXEL = map.info.height;
    WIDTH_PIXEL = map.info.width;
    RESOLUTION = map.info.resolution;
    max_y_position_ = static_cast<int>(HEIGHT_PIXEL * RESOLUTION);
    max_x_position_ = static_cast<int>(WIDTH_PIXEL * RESOLUTION);
    sum_weight = 1.0;
    OPEN_FILE_PARAMETERS(
        float temp[7];
        FILE *fp;
        int bufferLength = 255;
        char buffer[bufferLength];
        fp = fopen("/home/khoa1799/catkin_ws/src/SLAM/laser_based_localization_pf/src/parameters.txt", "r"); // read mode
        if (fp == NULL)
        {
            perror("Error while opening the file.\n");
            exit(EXIT_FAILURE);
        }
        int i = 0;
        while(fgets(buffer, bufferLength, fp)) 
        {
            temp[i] = atof(buffer);
            switch(i){
                case 0:
                    VAR_HIT = temp[i];
                    break;
                case 1:
                    z_hit = temp[i];
                    break;
                case 2:
                    z_random = temp[i];
                    break;
                case 3:
                    z_max = temp[i];
                    break; 
                case 4:
                    num_particles_ = temp[i];
                    break;
                case 5:
                    threshold_resample = temp[i];
                    break;
            }
            i++;
        }
        fclose(fp);
    );

    INIT_PARTICLES_(
        float area = (max_x_position_ +1)* (max_y_position_+1);
        float p =  area / num_particles_;
        float s = max_x_position_ / max_y_position_;
        float step_y = std::sqrt(p/s);
        float step_x = s*step_y;
        double _x = 0;
        double _y = 0; 
        ROS_INFO("max_x: %d , max_y: %d , p : %f , s: %f , step x: %f, step y: %f", max_x_position_ , max_y_position_ , p , s, step_x , step_y);
        std::uniform_real_distribution<double> distribution_theta(-0.2 , 0.2);
        
        for (int i=0; i < num_particles_ ; i++)
        {
            double theta = distribution_theta(generator1);
            Particle temp_par( _x , _y , theta , 1.0/num_particles_);
            particles_.push_back(temp_par);
            _x += step_x;
            if (_x > max_x_position_)
            {
                _x = 0;
                _y += step_y;
            }
        }
    );
    INIT_PARTICLES_TEST(
        Particle temp_par_(14.0 , 14.0 , 0.0 , 1.0/num_particles_);
        particles_.push_back(temp_par_);
        std::uniform_real_distribution<double> distribution_x(12,16);
        std::uniform_real_distribution<double> distribution_y(12,16);
        std::uniform_real_distribution<double> distribution_theta(-0.2 , 0.2);
        for (int i=0; i < num_particles_ -1; i++)
        {
            double x = distribution_x(generator1);
            double y = distribution_y(generator1);
            double theta = distribution_theta(generator1);
            Particle temp_par(x , y , theta , 1.0/num_particles_);
            particles_.push_back(temp_par);
            ROS_INFO("x:%f ,y:%f , theta:%f",x,y,theta);
        }
    );
    normalizeParticleWeights();
    updateLocalization(x,particles_);
}

void LaserBasedLocalizationPf::get_odometry(nav_msgs::Odometry odometry,double &displacement , double &direction , double &dtheta)
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
void LaserBasedLocalizationPf::updateOdometry(nav_msgs::Odometry odometry)
{
    //todo move particles the same way robot moved
    double displacement;
    double dtheta;
    double direction;
  
    get_odometry(odometry , displacement , direction , dtheta);
    if ((displacement != 0 || dtheta != 0) & !(odometry.twist.twist.linear.x == 0 && odometry.twist.twist.angular.z == 0)) 
    {
        for (int i = 0; i<num_particles_ ; i++)
        {
            double theta = particles_[i].getTheta() + dtheta;
            if (theta > M_PI) theta = theta - 2.0 * M_PI;
            else if (theta < -1.0 * M_PI) theta = 2.0*M_PI + theta;

            double update_x = direction * displacement*std::cos(theta) + particles_[i].getX();
            double update_y = direction * displacement*std::sin(theta) + particles_[i].getY();
            if (update_x > max_x_position_ || update_x<0) update_x = particles_[i].getX();
            if (update_y > max_y_position_ || update_x<0) update_y = particles_[i].getY();
            particles_[i].updatePose(update_x , update_y , theta);
        }
        updateLocalization(x,particles_);
        allow_resample = true;
    }
    return;
}


void LaserBasedLocalizationPf::visualizeSeenLaser(sensor_msgs::LaserScan laser)
{
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.05, 0, 0.3) );
    transform.setRotation( tf::createQuaternionFromRPY(0 , 0, 0) );
    pose_tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link_real", "laser_link_real"));

    laser.header.frame_id = "laser_link_real";
    real_laser_pub_.publish(laser);
}

void LaserBasedLocalizationPf::measures_evaluations()
{
    // for (int i =0; i<num_particles_; i++)
    // {
    double x_ = particles_[0].getX();
    double y_ = particles_[0].getY();
    double theta_ = particles_[0].getTheta();
    for (int j =0; j<num_laser_points; j ++)
    {
        double point_range = laser_info_.ranges[j * speed_up_laser];
        double point_theta = laser_info_.angle_increment * j + laser_info_.angle_min;
        int range =  1.0 / RESOLUTION;
        double point_x = x_ + laser_x_dist*std::cos(theta_) - laser_y_dist*std::sin(theta_) + point_range*std::cos(theta_+point_theta);
        double point_y = y_ + laser_x_dist*std::sin(theta_) + laser_y_dist*std::cos(theta_) + point_range*std::sin(theta_+point_theta);
        
        if (point_x>=1 && point_y>=1)
        {
            for (int i= (point_y-0.5)/RESOLUTION ; i< (point_y + 0.5)/RESOLUTION; i++)
            for (int j= (point_x-0.5)/RESOLUTION ; i< (point_x + 0.5)/RESOLUTION; i++)
                occ_grid_.data[j + i*WIDTH_PIXEL] = 97;
        }
        else if (point_x<1)
        {
            for (int i= (point_y-0.5)/RESOLUTION ; i< (point_y + 0.5)/RESOLUTION; i++)
            for (int j= (point_x)/RESOLUTION ; i< (point_x + 0.5)/RESOLUTION; i++)
                occ_grid_.data[j + i*WIDTH_PIXEL] = 97;
        }
        else if (point_y<1)
        {
            for (int i= (point_y)/RESOLUTION ; i< (point_y + 0.5)/RESOLUTION; i++)
            for (int j= (point_x-0.5)/RESOLUTION ; i< (point_x + 0.5)/RESOLUTION; i++)
                occ_grid_.data[j + i*WIDTH_PIXEL] = 97;
        }
        else
        {
            for (int i= (point_y)/RESOLUTION ; i< (point_y + 0.5)/RESOLUTION; i++)
            for (int j= (point_x)/RESOLUTION ; i< (point_x + 0.5)/RESOLUTION; i++)
                occ_grid_.data[j + i*WIDTH_PIXEL] = 97;
        }
        
    }

    //}
}
void LaserBasedLocalizationPf::updateLaser(sensor_msgs::LaserScan laser)
{
    visualizeSeenLaser(laser);
    if (allow_resample) 
    {
        allow_resample = false;
        sum_weight = 0;
        double prob_laser_measurement[num_particles_];
        for (int i =0; i<num_particles_ ; i++)
        {
            prob_laser_measurement[i] = 1.0;
            // simulate the laser scan
            sensor_msgs::LaserScan::Ptr temp_scan = simulateLaser(particles_[i].getX(),particles_[i].getY(),particles_[i].getTheta(), speed_up_laser);
            // LIKELIHOOD FIELD ALGORITHM
            //ROS_INFO("x: %f , y: %f , theta: %f", particles_[i].getX(),particles_[i].getY(),particles_[i].getTheta());
            for (int j = 0; j<num_laser_points; j++)
            {
                double point_range = temp_scan->ranges[j];
                double real_point_range = laser_info_.ranges[j * speed_up_laser];
                double point_theta = laser_info_.angle_increment * j + laser_info_.angle_min;
                double displacement = std::abs(point_range - real_point_range);
                double prob;
                if (point_range < laser_info_.range_max)
                {
                    prob = probNormalDistribution(displacement , VAR_HIT);
                    prob = z_hit * prob + z_random/z_max;
                    (prob < (1.0 /num_laser_points))? prob = 1.0 / num_laser_points : prob;
                    prob_laser_measurement[i] = prob_laser_measurement[i] * prob;
                }
                else {
                    prob_laser_measurement[i] = prob_laser_measurement[i] * (1.0 / num_laser_points);
                }
                LOG_ENDPONTS(
                    ROS_INFO("\t sti_range: %f , real_range: %f , theta: %f", point_range , real_point_range, point_theta);
                    ROS_INFO("\t \t Displacement : %.10f, PROB: %.20f",displacement, prob);
                );
            }
            particles_[i].weight_ = prob_laser_measurement[i];
            sum_weight += particles_[i].weight_;
        }
    
        LOG_PROB_BEFORE_NOR(
            ROS_INFO("BEFORE NORMALIZE:");
            for (int i =0 ; i<num_particles_ ; i++)
            {
                ROS_INFO("\t X: %.5f , Y: %.5f , THETA: %.5f ", particles_[i].getX(), particles_[i].getY(), particles_[i].getTheta());
                ROS_INFO("\t \t WEIGTH: %.50f" , particles_[i].weight_);
            }
        );
        measures_evaluations();
        normalizeParticleWeights();
        resamplingParticles();
    }
    updateLocalization(x, particles_);
}

void LaserBasedLocalizationPf::resetLocalization(double x, double y, double theta)
{
    this->x(0,0) = x;
    this->x(1,0) = y;
    this->x(2,0) = theta;

    //distribute particles around true pose

    double scale_factor = 1000.0;

    int x_range = static_cast<int>(max_x_position_ / 10.0 * scale_factor);
    int y_range = static_cast<int>(max_y_position_ / 10.0 * scale_factor);
    int theta_range = static_cast<int>(M_PI / 4.0 * scale_factor);
    for(int i = 0; i < num_particles_; i++)
    {
        double new_x = x + (std::rand() % x_range - static_cast<int>(x_range/2.0) ) / scale_factor;
        double new_y = y + (std::rand() % y_range - static_cast<int>(y_range/2.0) ) / scale_factor;
        double new_theta  = theta + (std::rand() % theta_range - static_cast<int>(theta_range/2.0)) / scale_factor;
        particles_[i].updatePose(new_x, new_y, new_theta);
        particles_[i].weight_ = 1.;
    }
}

void LaserBasedLocalizationPf::updateLocalization(Eigen::MatrixXd x, std::vector<Particle>& particles)
{
    //visualisation of pose
    publishPose(x, particles);

    //visualization of particles
    publishParticles(particles);
}

void LaserBasedLocalizationPf::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    data_mutex_->lock();
    static bool first = true;
    if(first)
    {
        laser_info_.angle_max = msg->angle_max;
        laser_info_.angle_min = msg->angle_min;
        laser_info_.header = msg->header;
        laser_info_.range_max = msg->range_max;
        laser_info_.range_min = msg->range_min;
        laser_info_.scan_time = msg->scan_time;
        laser_info_.angle_increment = msg->angle_increment;
        laser_info_.ranges.resize(msg->ranges.size());
        laser_info_.intensities.resize(msg->intensities.size());
        num_laser_points = laser_info_.ranges.size() / speed_up_laser;
        for (int i =0; i<laser_info_.ranges.size() ; i++)
            laser_info_.ranges[i] = msg->ranges[i];
        for (int i =0; i<laser_info_.intensities.size() ; i++)
            laser_info_.intensities[i] = msg->intensities[i];
        first = false;
        ROS_INFO("no of laser points: %d , angle max: %f , angle min: %f , angle incr: %f" ,
         num_laser_points , laser_info_.angle_max , laser_info_.angle_min , laser_info_.angle_increment);
    }
    else{
        laser_info_.header = msg->header;
        for (int i =0; i<msg->ranges.size() ; i++)
            laser_info_.ranges[i] = msg->ranges[i];
    }
    updateLaser(*msg);
    data_mutex_->unlock();
}
void LaserBasedLocalizationPf::landmarkCallback(const stage_ros::fiducials::ConstPtr& msg)
{
    data_mutex_->lock();
    landmark_ = *msg;
    data_mutex_->unlock();
}
void LaserBasedLocalizationPf::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    data_mutex_->lock();
    updateOdometry(*msg);
    data_mutex_->unlock();
}

void LaserBasedLocalizationPf::initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    double x, y, theta;
    data_mutex_->lock();
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    theta =  tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("initalPoseCallback x=%f, y=%f, theta=%f", x, y, theta);
    resetLocalization(x, y, theta);
    data_mutex_->unlock();
}

void LaserBasedLocalizationPf::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    data_mutex_->lock();
    occ_grid_ = *msg;
    data_mutex_->unlock();
}

void LaserBasedLocalizationPf::normalizeParticleWeights()
{
    double count = 0.0;
    for (int i = 0; i< num_particles_ ; i++) 
    {
        particles_[i].weight_ = particles_[i].weight_/ sum_weight;
        count += particles_[i].weight_;
    }
    LOG_PROB_AFTER_NOR(
        ROS_INFO("AFTER NORMALIZE:");
        for (int i = 0; i <num_particles_ ; i++)
        {
            ROS_INFO("\t X: %.5f , Y: %.5f , THETA: %.5f", particles_[i].getX(), particles_[i].getY(), particles_[i].getTheta() );
            ROS_INFO("\t \t WEIGHT: %.50f" , particles_[i].weight_);
        }
    ); 
}

void LaserBasedLocalizationPf::resamplingParticles()
{
    SCHOTASTIC_RESAMPLE(
        std::uniform_real_distribution<double> distribution_theta(-0.2 , 0.2);
        double temp_weight = 1.0/num_particles_;
        double weight = temp_weight;
        double c[num_particles_];
        c[0] = particles_[0].weight_;
        for (int j =1; j<num_particles_; j++)
            c[j] = c[j-1] + particles_[j].weight_;
        std::vector<Particle> temp_particles_;
        int i = 0;
        int count =0;
        while (i < num_particles_)
        {
            while (c[i] < weight) 
            {
                if (i == num_particles_ - 1) break;
                i++;
            }
            if (i < num_particles_)
            {
                Particle temp(particles_[i].getX() , particles_[i].getY(), particles_[i].getTheta() , temp_weight);
                temp_particles_.push_back(temp);
                count ++;
            }
            if (count == num_particles_) break;
            weight += temp_weight;
        }
        // ROS_INFO("i: %d , weight: %f" ,i , weight );
        // ROS_INFO("out count: %d" , count);

        for (int j =0; j<num_particles_; j++)
        {
            if (temp_particles_[j].weight_>= temp_weight){
                particles_[j].updatePose(temp_particles_[j].getX() , temp_particles_[j].getY() , temp_particles_[j].getTheta());
                particles_[j].updateWeight(temp_particles_[j].weight_);
            }
            else{
                double x = temp_particles_[j].getX() + probNormalDistribution(temp_particles_[j].getX() , VAR_HIT);
                double y = temp_particles_[j].getY() + probNormalDistribution(temp_particles_[j].getY() , VAR_HIT);
                double theta = temp_particles_[j].getTheta() + probNormalDistribution(temp_particles_[j].getTheta() , VAR_HIT);
                particles_[j].updatePose(x , y , theta);
                particles_[j].updateWeight(temp_weight);
            }
        }
    );
    
}

void LaserBasedLocalizationPf::publishParticles(std::vector<Particle>& particles)
{
    geometry_msgs::PoseArray array;
    array.poses = getParticlePositions(particles);
    array.header.frame_id = "map";
    array.header.stamp = ros::Time(0);

    particles_pub_.publish(array);
}

std::vector<geometry_msgs::Pose> LaserBasedLocalizationPf::getParticlePositions(std::vector<Particle>& particles)
{
    std::vector<geometry_msgs::Pose> positions;

    for(int i = 0; i < particles.size(); i++)
    {
        positions.push_back(particles[i].pose_);
        //ROS_INFO("x y [%d] : %f %f", i , particles[i].getX() , particles[i].getY());
    }

    return positions;
}

void LaserBasedLocalizationPf::publishPose(Eigen::MatrixXd& x, std::vector<Particle>& particles)
{
    //calculate mean of given particles
    double x_mean = 0;
    double y_mean = 0;
    double yaw_mean = 0;
    for (int i =0; i<num_particles_; i++)
    {
        x_mean += particles_[i].getX();
        y_mean += particles_[i].getY();
        yaw_mean += particles_[i].getTheta();
    }

	
	// TODO Calculate the robot pose from the particles
	
    x(0,0) = x_mean / num_particles_;
    x(1,0) = y_mean / num_particles_;
    x(2,0) = yaw_mean / num_particles_;
    ROS_INFO("%f %f %f\n",x(0,0) , x(1,0) , x(2,0));

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x_mean, y_mean, 0.0) );
    transform.setRotation( tf::createQuaternionFromRPY(0 , 0, yaw_mean) );
    pose_tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "robot_pose"));

    //calculate covariance matrix
    double standard_deviation_x = 0;
    double standard_deviation_y = 0;
    double standard_deviation_theta = 0;

    //todo check if uncertainty possible
    for (int i = 0; i < particles.size(); i++)
    {
        standard_deviation_x += std::pow(particles[i].getX() - x_mean, 2);
        standard_deviation_y += std::pow(particles[i].getY() - y_mean, 2);
        standard_deviation_theta += std::pow(particles[i].getTheta() - yaw_mean, 2);
    }
    standard_deviation_theta = std::sqrt(standard_deviation_theta);
    standard_deviation_x = std::sqrt(standard_deviation_x);
    standard_deviation_y = std::sqrt(standard_deviation_y);
    standard_deviation_theta /= static_cast<double>(particles.size() - 1);
    standard_deviation_x /= static_cast<double>(particles.size() - 1);
    standard_deviation_y /= static_cast<double>(particles.size() - 1);
    //ROS_INFO("%f %f %f\n",standard_deviation_x,standard_deviation_y,standard_deviation_theta);

    //need to bound it otherwise calc of uncertainty marker doesn't work
    double thresh = 0.0000001;
    if(standard_deviation_theta < thresh)
        standard_deviation_theta = thresh;
    if(standard_deviation_x < thresh)
        standard_deviation_x = thresh;
    if(standard_deviation_y < thresh)
        standard_deviation_y = thresh;

    //put in right msg
    geometry_msgs::PoseWithCovarianceStamped pose_with_cov;
    pose_with_cov.header.frame_id = "map";
    pose_with_cov.header.stamp = ros::Time(0);

    tf::Quaternion q;
    q = tf::createQuaternionFromYaw(yaw_mean);

    pose_with_cov.pose.pose.position.x = x_mean;
    pose_with_cov.pose.pose.position.y = y_mean;
    pose_with_cov.pose.pose.position.z = 0;
    pose_with_cov.pose.pose.orientation.w = q.getW();
    pose_with_cov.pose.pose.orientation.x = q.getX();
    pose_with_cov.pose.pose.orientation.y = q.getY();
    pose_with_cov.pose.pose.orientation.z = q.getZ();

    pose_with_cov.pose.covariance[0] = std::pow(standard_deviation_x,2);
    pose_with_cov.pose.covariance[7] = std::pow(standard_deviation_y,2);
    pose_with_cov.pose.covariance[35] = std::pow(standard_deviation_theta,2);
    pose_with_cov_pub_.publish(pose_with_cov);

    // Uncertainty Visualization
    Eigen::Matrix2f uncertainty_mat;
    uncertainty_mat(0,0) = standard_deviation_x * RESOLUTION;
    uncertainty_mat(0,1) = thresh;
    uncertainty_mat(1,0) = thresh;
    uncertainty_mat(1,1) = standard_deviation_y * RESOLUTION;

    Eigen::Vector2f uncertainty_position;
    uncertainty_position(0) = x(0,0);
    uncertainty_position(1) = x(1,0);

    visualization_msgs::Marker uncertainly_marker;
    generateUncertaintyMarker(uncertainly_marker, uncertainty_mat, uncertainty_position);
    vis_pub_.publish(uncertainly_marker);
}

void LaserBasedLocalizationPf::generateUncertaintyMarker(visualization_msgs::Marker& marker, Eigen::Matrix2f uncertainly_mat, Eigen::Vector2f position)
{
    Eigen::EigenSolver<Eigen::Matrix2f> solver(uncertainly_mat);
    Eigen::VectorXf uncertainty_eigenvalues = solver.eigenvalues().real(); // matrix 1x2 with 1 is value for x and 2 is
                                                                            // value for y
    //std::cout << std::endl << "Eigenvalues: " << std::endl << uncertainty_eigenvalues.transpose() << std::endl;
    Eigen::MatrixXf uncertainty_eigenvectors = solver.eigenvectors().real(); // matrix 2x2 
    //std::cout << std::endl << uncertainty_eigenvectors << std::endl;

    double phi_ellipse = std::atan2(uncertainty_eigenvectors(0,1), uncertainty_eigenvectors(0,0));

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "ellipses";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Pose ellipse_pose;

    ellipse_pose.position.x = position(0);
    ellipse_pose.position.y = position(1);
    ellipse_pose.position.z = 0;

    tf::Quaternion tf_quat = tf::createQuaternionFromRPY(0, 0, phi_ellipse);
    tf::quaternionTFToMsg(tf_quat, ellipse_pose.orientation);

    marker.pose = ellipse_pose;

    // eigenvalue of uncertainty matrix is the square of the semi-major/minor of the ellipse;
    // 2.447*sigma => 95% area
    marker.scale.x = 2.447*2.0*std::sqrt(uncertainty_eigenvalues(0));
    marker.scale.y = 2.447*2.0*std::sqrt(uncertainty_eigenvalues(1));
    marker.scale.z = 0.1;
    marker.color.a = 0.2;
    marker.color.r = 0.9;
    marker.color.g = 0.0;
    marker.color.b = 0.3;
}

double LaserBasedLocalizationPf::probNormalDistribution(double a, double variance)
{
    if (variance == 0)
        return a;

    return ( 1.0 / (std::sqrt(2.0*M_PI * std::sqrt(variance))) ) * std::exp( -0.5 * (std::pow( a, 2 ) / variance) );

}

double LaserBasedLocalizationPf::sampleNormalDistribution(double variance)
{
    double scaling_factor = 1000.0;
    if (variance <= (1.0/scaling_factor))
        return 0;

    double sum = 0;

    int border = std::sqrt(variance) * static_cast<int>(scaling_factor);
    for (int i = 0; i < 12; i++)
        sum += std::rand() % (2 * border) - border;

    return sum * 0.5 / scaling_factor;

}

// STIMULATE BUT NOT STIMULATE THE FICIALS
sensor_msgs::LaserScan::Ptr LaserBasedLocalizationPf::simulateLaser(double x, double y, double theta, double speedup)
{
    geometry_msgs::Pose laser_pose;
    laser_pose.position.x = x + laser_x_dist*std::cos(theta) - laser_y_dist*std::sin(theta);
    laser_pose.position.y = y + laser_x_dist*std::sin(theta) + laser_y_dist*std::cos(theta);
    laser_pose.position.z = laser_z_dist;
    laser_pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    double inc = laser_info_.angle_increment;
    laser_info_.angle_increment = inc * speedup;
    sensor_msgs::LaserScan::Ptr simulated_laser = occupancy_grid_utils::simulateRangeScan(occ_grid_, laser_pose, laser_info_, true);
    laser_info_.angle_increment = inc;
    return simulated_laser;

}


int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "laser_based_localization");
    ros::NodeHandle n;

    LaserBasedLocalizationPf* lmbl_ptr = new LaserBasedLocalizationPf(n);

    ros::Subscriber odometry = n.subscribe("/odom", 1, &LaserBasedLocalizationPf::odometryCallback, lmbl_ptr);
    ros::Subscriber initialpose = n.subscribe("/initialpose", 1, &LaserBasedLocalizationPf::initialposeCallback, lmbl_ptr);
    ros::Subscriber map = n.subscribe("/map", 1, &LaserBasedLocalizationPf::mapCallback, lmbl_ptr);
    ros::Subscriber laser_sub = n.subscribe("/base_scan",1, &LaserBasedLocalizationPf::laserCallback, lmbl_ptr);
    //ros::Subscriber lanmark_sub = n.subscribe("/fiducials",1,&LaserBasedLocalizationPf::landmarkCallback, lmbl_ptr);
    //boost::thread(&Controller::stateMachine, controller);

    std::cout << "Laser Based Localization started..." << std::endl;

    ros::spin();

    return 0;
}

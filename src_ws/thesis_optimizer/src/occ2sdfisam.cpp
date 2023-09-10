

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>  

//include opencv for image processing
#include <opencv2/opencv.hpp>



//import gpmp2 library
#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/planner/TrajUtils.h>
#include <gpmp2/planner/BatchTrajOptimizer.h>
#include <gpmp2/planner/ISAM2TrajOptimizer.h>


//import gtsam 
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>

//include simulator_utils msg
#include <simulator_utils/Waypoint.h>


//import ros message
#include <rosgraph_msgs/Log.h>
//import std message
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

//time execution
#include <iostream>
#include <chrono>
#include <mutex>

ros::Publisher pub;
ros::Publisher path_pub_init;
ros::Publisher path_pub_opt;

//define mutex
std::mutex sdf_mutex;

gpmp2::SignedDistanceField::shared_ptr sdf;
nav_msgs::Path initial_path;
nav_msgs::Path optimized_path;

gtsam::Vector3 current_position;
gtsam::Values result_isam;

size_t min_index=0;

bool start_flag=true;




double total_time=10.0;
size_t total_step=20;
size_t total_dof=3;
size_t total_link=1;
size_t link_idx=0;
size_t total_check_step=400;
size_t check_interval=total_check_step/total_step - 1 ;

// set up start and end conf
gtsam::Vector start_conf(total_dof);
gtsam::Vector end_conf(total_dof);
gtsam::Vector start_vel(total_dof);
gtsam::Vector end_vel(total_dof);



//define the 3d point robot model
gpmp2::PointRobot3D fk_model(total_dof,total_link);
gpmp2::BodySphereVector body_spheres;
double radius = 0.1;  // Radius of the body sphere
gtsam::Point3 center(0, 0, 0);  // Center of the body sphere (at the centroid of the robot)
gpmp2::PointRobot3DModel robot_model;

//setup noise model gaussian covariance
gtsam::Matrix Qc = gtsam::I_3x3;

//noise model for prior
double pose_sigma=0.00001;
double vel_sigma=0.001;

//obstacle cost settings
double epsilon_dist=0.2;
double cost_sigma=0.05;

size_t iter=100;
double rel_thresh=1e-6;

//define trajectory optimizer setting
gpmp2::TrajOptimizerSetting optimizerSettings(total_dof);


//initialize ISAM2 optimizer
gpmp2::ISAM2TrajOptimizerPR3D optimizer;


//procedure to convert gtsam::Values to geometry_msgs::PoseStamped
nav_msgs::Path ValuestoPath (gtsam::Values traj)
{
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();


    for (const auto& key_value : traj) {
        const gtsam::Key& key = key_value.key;
        const gtsam::Vector& value = key_value.value.template cast<gtsam::Vector>();


        //use gtsam::Symbol to get the key value and char
        gtsam::Symbol symb(key);
        // ROS_INFO("key: %ld", symb.index());
        // ROS_INFO("char: %c", symb.chr());

        //convert gtsam::Vector to geometry_msgs::PoseStamped
        if (symb.chr() == 'x') {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = value(0);
            pose.pose.position.y = value(1);
            pose.pose.position.z = value(2);
            path.poses.push_back(pose);
            ROS_INFO("pose at %c %ld is : %f, %f, %f",symb.chr(), symb.index(), value(0), value(1), value(2));
         
        }
    }



    return path;
}



void PCCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(sdf_mutex);

    // Check if the message is empty
    if (msg->data.size() == 0)
    {
        ROS_WARN("Received an empty SDF");
        return;
    }

    //std::vector<gtsam::Matrix> sdf_data;
 

    if (!sdf)
    {
        ROS_INFO("Initializing SDF");
        auto start = std::chrono::high_resolution_clock::now();

            // Define SDF dimensions and resolution
        int x_count = 120;
        int y_count = 120;
        int z_count = 80;
        double cell_size = 0.05;
        double minX = -1*x_count*cell_size/2;
        double minY = -1*y_count*cell_size/2;
        double minZ = 0;
        std::vector<gtsam::Matrix> data;

        // Iterate through the PCL data in the message and set the values in the data array
        pcl::PointCloud<pcl::PointXYZI> pclCloud;
        pcl::fromROSMsg(*msg, pclCloud);

        for (int z = 0; z < z_count; ++z) {
            gtsam::Matrix layer(x_count, y_count);
            for (int y = 0; y < y_count; ++y) {
                for (int x = 0; x < x_count; ++x) {
                    layer(y, x) = pclCloud.points[x * y_count * z_count + y * z_count + z].intensity;
                }
            }
            data.push_back(layer);
        }


        sdf = boost::make_shared<gpmp2::SignedDistanceField>(
            gtsam::Point3(minX, minY, minZ),
            cell_size,
            data);



        auto end = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        ROS_INFO("SDF computation time: %ld ms", duration.count()); 


        if (start_flag)
        {
            //initialize straight line trajectory
            gtsam::Values traj_values = gpmp2::initPR3DTrajStraightLine(current_position, end_conf, total_step);

            ROS_INFO("initial trajectory: ");    
            initial_path=ValuestoPath(traj_values);

            auto start_opt = std::chrono::high_resolution_clock::now();

            //optimize using batch method
            gtsam::Values result = gpmp2::BatchTrajOptimizePR3D(robot_model, *sdf, current_position, start_vel,
                                                                end_conf, end_vel, traj_values, optimizerSettings);

            //update SDF
            optimizer.updateSDF(*sdf);

            //feed initial values
            optimizer.initFactorGraph(current_position, start_vel, end_conf, end_vel);

            //init values
            optimizer.initValues(result);

            //update values
            optimizer.update();

            auto end_opt = std::chrono::high_resolution_clock::now();

            auto duration_opt = std::chrono::duration_cast<std::chrono::milliseconds>(end_opt - start_opt);

            ROS_INFO("optimization time: %ld ms", duration_opt.count());

            result_isam=optimizer.values();
        
            start_flag=false;
        }
        else
        {
            auto start_opt = std::chrono::high_resolution_clock::now();

            gtsam::Vector3 current_position_gtsam = gtsam::Vector3(current_position(0), current_position(1), current_position(2));

            // fix pose and velocity at current index
            optimizer.fixConfigAndVel(min_index, current_position_gtsam, start_vel);

            //run ISAM2 optimizer change obstacle 
            optimizer.changeObstacleFactor(min_index, *sdf);

            //update values
            optimizer.update();

            auto end_opt = std::chrono::high_resolution_clock::now();

            auto duration_opt = std::chrono::duration_cast<std::chrono::milliseconds>(end_opt - start_opt);

            ROS_INFO("REoptimization time: %ld ms", duration_opt.count());

            result_isam=optimizer.values();

        }

        ROS_INFO("optimized trajectory: ");
        optimized_path=ValuestoPath(result_isam);


        // Publish the path
        path_pub_init.publish(initial_path);
        path_pub_opt.publish(optimized_path);

        //publishSDFasPointCloud(sdf, pub, sdf_data, minX, minY, minZ, resolution);

    }
    
    if (!initial_path.poses.empty() && !optimized_path.poses.empty()) {
        // Publish the path
        ros::Rate rate(10);
        path_pub_init.publish(initial_path);
        path_pub_opt.publish(optimized_path);
    }
}


void BoolCallback(const std_msgs::BoolConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(sdf_mutex);

    if (msg->data == true) 
    {
        ROS_INFO("New SDF Received");
        sdf.reset();
        initial_path.poses.clear();
        optimized_path.poses.clear();
        ROS_INFO("Reseted SDF");
    }
}

void CurrStateCallback(const simulator_utils::Waypoint::ConstPtr& msg)
{
    //read position
    current_position(0)=msg->position.x;
    current_position(1)=-1*msg->position.y;
    current_position(2)=msg->position.z;


    //check the closest index to current position from result_isam, make sure result_isam is not empty
    if (!result_isam.empty())
    {
        double min_dist=1000000;
        for (const auto& key_value : result_isam) {
            const gtsam::Key& key = key_value.key;
            const gtsam::Vector& value = key_value.value.template cast<gtsam::Vector>();

            gtsam::Symbol symb(key);

            if (symb.chr() == 'x') {
                //check for closest index
                double dist=sqrt(pow(current_position(0)-value(0),2)+pow(current_position(1)-value(1),2)+pow(current_position(2)-value(2),2));
                if (dist<min_dist)
                {
                    min_dist=dist;
                    min_index=symb.index();
                }
            }
        }

      //  ROS_INFO("the closest index is: %ld", min_index);
    }

}


int main(int argc, char** argv)
{


    // start_conf<<2.7,2.7,0.5;
    end_conf<<-2.5,-2,2.5;

    // set up start and end vel
    start_vel<<0,0,0;
    end_vel<<0,0,0;

    //calculate abs distance between start position and end position
    double abs_dist_start=sqrt(pow(start_conf(0)-end_conf(0),2)+pow(start_conf(1)-end_conf(1),2)+pow(start_conf(2)-end_conf(2),2));

    //calculate total step, assume each step is 20cm
    total_step=abs_dist_start/0.4;
    //set interval to check for collision 
    check_interval=10;
    size_t delta_t=total_time/total_step;


    optimizerSettings.set_total_step(total_step);
    optimizerSettings.set_total_time(total_time);
    optimizerSettings.set_epsilon(epsilon_dist);
    optimizerSettings.set_cost_sigma(cost_sigma);
    optimizerSettings.set_obs_check_inter(check_interval);
    optimizerSettings.set_conf_prior_model(pose_sigma);
    optimizerSettings.set_vel_prior_model(vel_sigma);
    optimizerSettings.set_Qc_model(Qc);
    optimizerSettings.set_max_iter(iter);
    optimizerSettings.set_rel_thresh(rel_thresh);

    //empty data for initialization
    std::vector<gtsam::Matrix> data_dummy;
    //fill data dummy with 0 layer
    for (int z = 0; z < 80; ++z) {
        gtsam::Matrix layer(120, 120);
        for (int y = 0; y < 120; ++y) {
            for (int x = 0; x < 120; ++x) {
                layer(y, x) = 0;
            }
        }
        data_dummy.push_back(layer);
    }

    //fill sdf with empty data
    sdf = boost::make_shared<gpmp2::SignedDistanceField>(
            gtsam::Point3(0, 0, 0),
            0.05,
            data_dummy);

    body_spheres.push_back(gpmp2::BodySphere(link_idx, radius, center));
    robot_model = gpmp2::PointRobot3DModel(fk_model, body_spheres);
    optimizer = gpmp2::ISAM2TrajOptimizerPR3D(robot_model, *sdf, optimizerSettings);

    ros::init(argc, argv, "edt_node");
    ros::NodeHandle nh;
    
    //ros::Subscriber sub = nh.subscribe("/array_topic", 1000, ArrayCallback);
    ros::Subscriber pc_sub = nh.subscribe("/pcl_topic_2", 1000, PCCallback);
    ros::Subscriber bool_sub = nh.subscribe("/bool_topic", 1000, BoolCallback);
    ros::Subscriber current_state_sub_ = nh.subscribe("/robot_3/current_state", 10, CurrStateCallback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/sdf_cloud", 1);
    path_pub_init = nh.advertise<nav_msgs::Path>("/trajectory_init_isam", 1);
    path_pub_opt = nh.advertise<nav_msgs::Path>("/trajectory_opt_occ2sdfisam", 1);

    
    ros::spin();
    return 0;
}


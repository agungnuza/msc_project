#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>  

//import gpmp2 library
#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/planner/TrajUtils.h>
#include <gpmp2/planner/BatchTrajOptimizerNew.h>


//import gtsam 
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>


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

//include simulator_utils msg
#include <simulator_utils/Waypoint.h>


//global variables initialization
ros::Publisher pub;
ros::Publisher path_pub_init;
ros::Publisher path_pub_opt;
nav_msgs::Path initial_path;
nav_msgs::Path optimized_path;
gtsam::Vector3 current_position;
octomap::OcTree* octree = new octomap::OcTree(0.05);
DynamicEDTOctomap distmap (4, octree, octomap::point3d(-3, -3, 0), octomap::point3d(3, 3, 4), false);
std::mutex sdf_mutex;

void publishSDFasPointCloud(const DynamicEDTOctomap& sdf, ros::Publisher &pub,
                            double minX, double minY, double minZ, double resolution, double maxX, double maxY, double maxZ)
{
    // Initialize Point Cloud
    sensor_msgs::PointCloud2 cloud;
    //point cloud XYZI format
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

    // Iterate through the SDF and set signed distance as intensity
    for (double x = minX; x < maxX; x += resolution)
    {
        for (double y = minY; y < maxY; y += resolution)
        {
            for (double z = minZ; z < maxZ; z += resolution)
            {
                // Get the signed distance
                double dist = sdf.getDistance(octomap::point3d(x, y, z));
                // Set the intensity
                pcl::PointXYZI point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.intensity = dist;
                // Add the point to the Point Cloud
                pcl_cloud.push_back(point);
            }
        }
    }

    // Convert to sensor_msgs::PointCloud2
    pcl::toROSMsg(pcl_cloud, cloud);

    // Set header information
    cloud.header.stamp = ros::Time::now();
    // attach on the same frame as the octomap msg
    cloud.header.frame_id = "map";

    // Publish the Point Cloud
    pub.publish(cloud);
}

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



void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    std::lock_guard<std::mutex> lock(sdf_mutex);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    octomap::Pointcloud octomapCloud;

    //start timer
    auto start = std::chrono::high_resolution_clock::now();

    for(auto p:cloud.points)
    {
        octomapCloud.push_back(p.x, p.y, p.z);
    }

    // Create octomap::pose6d for sensor origin
    octomap::point3d sensorOrigin(0,0,0);

    // Insert point cloud into octree
    octree->insertPointCloud(octomapCloud, sensorOrigin);

    // Update inner nodes
    octree->updateInnerOccupancy();

    // Update distance map
    distmap.update();

    //stop timer
    auto finish = std::chrono::high_resolution_clock::now();


    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start);

    ROS_INFO("SDF computation time: %ld ms", duration.count()); 

    double total_time=10.0;
    size_t total_step=20;
    size_t total_dof=3;
    size_t total_link=1;
    size_t link_idx=0;
    size_t total_check_step=400;
    size_t check_interval;

    // set up start and end conf
    // gtsam::Vector start_conf(total_dof);
    // start_conf<<2.7,2.7,0.5;
    gtsam::Vector end_conf(total_dof);
    end_conf<<-2.5,-2,2.5;

    //calcuate abs distance between current position and end position
    double abs_dist = sqrt(pow(current_position(0)-end_conf(0),2)+pow(current_position(1)-end_conf(1),2)+pow(current_position(2)-end_conf(2),2));
    //calculate total step, assume each step is 20cm, make sure its an integer
    total_step = abs_dist/0.4;
    //set check interval to be 10
    check_interval = 10;


    // set up start and end vel
    gtsam::Vector start_vel(total_dof);
    start_vel<<0,0,0;
    gtsam::Vector end_vel(total_dof);
    end_vel<<0,0,0;


    //define the 3d point robot model
    gpmp2::PointRobot3D fk_model(total_dof,total_link);
    gpmp2::BodySphereVector body_spheres;
    double radius = 0.1;  // Radius of the body sphere
    gtsam::Point3 center(0, 0, 0);  // Center of the body sphere (at the centroid of the robot)
    body_spheres.push_back(gpmp2::BodySphere(link_idx, radius, center));
    gpmp2::PointRobot3DModel robot_model(fk_model, body_spheres);

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

    //initialize straight line trajectory
    gtsam::Values traj_values = gpmp2::initPR3DTrajStraightLine(current_position, end_conf, total_step);

    ROS_INFO("initial trajectory: ");    
    initial_path=ValuestoPath(traj_values);

    auto start_opt = std::chrono::high_resolution_clock::now();

    //optimize using batch method
    gtsam::Values result = gpmp2::BatchTrajOptimizeNewPR3D(robot_model, distmap, current_position, start_vel,
                                                        end_conf, end_vel, traj_values, optimizerSettings);

    auto end_opt = std::chrono::high_resolution_clock::now();

    auto duration_opt = std::chrono::duration_cast<std::chrono::milliseconds>(end_opt - start_opt);

    ROS_INFO("optimization time: %ld ms", duration_opt.count());


    ROS_INFO("optimized trajectory: ");
    optimized_path=ValuestoPath(result);


    // Publish the path
    path_pub_init.publish(initial_path);
    path_pub_opt.publish(optimized_path);

    // publishSDFasPointCloud(distmap, pub, minX, minY, minZ, resolution, maxX, maxY, maxZ);

    if (!initial_path.poses.empty() && !optimized_path.poses.empty()) {
        // Publish the path

        //set publish rate to 10Hz
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
        //sdf.reset();
        initial_path.poses.clear();
        optimized_path.poses.clear();
    }
}

void CurrStateCallback(const simulator_utils::Waypoint::ConstPtr& msg)
{
    //read position
    current_position(0)=msg->position.x;
    current_position(1)=-1*msg->position.y;
    current_position(2)=msg->position.z;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "edt_node");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("/pcl_topic", 1000, pointCloudCallback);
    ros::Subscriber bool_sub = nh.subscribe("/bool_topic", 1000, BoolCallback);
    ros::Subscriber current_state_sub_ = nh.subscribe("/robot_2/current_state", 10, CurrStateCallback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/udf_cloud", 1);
    path_pub_init = nh.advertise<nav_msgs::Path>("/trajectory_init", 1);
    path_pub_opt = nh.advertise<nav_msgs::Path>("/trajectory_opt_octo2sdf", 1);

    
    ros::spin();
    return 0;
}

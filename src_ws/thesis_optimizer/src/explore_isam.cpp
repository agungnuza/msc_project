#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/planner/TrajUtils.h>
#include <gpmp2/planner/BatchTrajOptimizerNew.h>
#include <gpmp2/planner/ISAM2TrajOptimizerNew.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>
#include <simulator_utils/Waypoint.h>
#include <rosgraph_msgs/Log.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include <iostream>
#include <chrono>
#include <mutex>

// Remove the OCTOMAP_DEBUG definition
std::mutex m_edt_mutex;

ros::Publisher pub;
ros::Publisher path_pub_vis;
ros::Publisher path_pub_opt;

ros::Subscriber robot_pose_sub;

ros::Timer timer1;
ros::Timer timer2;
ros::Timer timer3;

ros::ServiceServer service;

octomap::OcTree* m_octree = new octomap::OcTree(0.1);
DynamicEDTOctomap distmap(3.0, m_octree, octomap::point3d(-5,-5,0), octomap::point3d(5,5,3.5), false);
DynamicEDTOctomap* m_edt = new DynamicEDTOctomap(5.0, m_octree, octomap::point3d(-5,-5,0), octomap::point3d(5,5,3.5), false);

gtsam::Vector start_conf = gtsam::Vector3(4.7, 4.8, 0.5);
gtsam::Vector end_conf = gtsam::Vector3(-4.5, -4.7, 2.5);
gtsam::Vector start_vel = gtsam::Vector3(0, 0, 0);
gtsam::Vector end_vel = gtsam::Vector3(0, 0, 0);
gtsam::Vector3 current_position;
gtsam::Values result_isam;
gtsam::Values result_batch;
gtsam::Values init_values;
size_t min_index = 0;
size_t prev_min_index = 0;
bool start_flag = true;
nav_msgs::Path optimized_path;
size_t current_index = 0;
nav_msgs::Path temp_path;
size_t total_dof = 3;
size_t total_link = 1;
size_t link_idx = 0;
double total_time = 10;

gpmp2::TrajOptimizerSetting optimizerSettings = gpmp2::TrajOptimizerSetting(total_dof);
double abs_dist_start = sqrt(pow(start_conf(0)-end_conf(0),2) + pow(start_conf(1)-end_conf(1),2) + pow(start_conf(2)-end_conf(2),2));
size_t total_step = abs_dist_start / 0.4;
size_t check_interval = 10;
size_t delta_t = total_time / total_step;

gpmp2::PointRobot3D fk_model = gpmp2::PointRobot3D(total_dof, total_link);
double radius = 0.1;
gtsam::Point3 center = gtsam::Point3(0, 0, 0);
gpmp2::BodySphereVector body_spheres;
gpmp2::PointRobot3DModel robot_model;

// Initialization structure for global constructs
struct GlobalInit
{
    GlobalInit()
    {
        optimizerSettings.set_total_step(total_step);
        optimizerSettings.set_total_time(total_time);
        optimizerSettings.set_epsilon(0.4);
        optimizerSettings.set_cost_sigma(0.05);
        optimizerSettings.set_obs_check_inter(check_interval);
        optimizerSettings.set_conf_prior_model(0.00001);
        optimizerSettings.set_vel_prior_model(0.001);
        optimizerSettings.set_Qc_model(gtsam::I_3x3);
        optimizerSettings.set_max_iter(100);
        optimizerSettings.set_rel_thresh(1e-6);
        optimizerSettings.setLM();
        body_spheres.push_back(gpmp2::BodySphere(link_idx, radius, center));
        robot_model = gpmp2::PointRobot3DModel(fk_model, body_spheres);
    }
} global_initializer_instance;

gpmp2::ISAM2TrajOptimizerNewPR3D optm = gpmp2::ISAM2TrajOptimizerNewPR3D(robot_model, distmap, optimizerSettings, distmap);


/**
 * Function to convert a gtsam::Values object into a ROS nav_msgs::Path message.
 * 
 * The gtsam::Values object typically represents a set of configurations or states
 * over a trajectory or path. This function extracts these states and converts them
 * into ROS PoseStamped messages, which are then added to a ROS Path message.
 * 
 * @param traj A gtsam::Values object representing the trajectory or path.
 * @return A nav_msgs::Path message representing the same trajectory or path.
 */
nav_msgs::Path ValuestoPath (gtsam::Values traj)
{
    // Initialize an empty ROS Path message
    nav_msgs::Path path;
    path.header.frame_id = "map";  // Set the frame of reference for the path
    path.header.stamp = ros::Time::now();  // Timestamp the path with current time

    // Iterate over all states or configurations in the gtsam::Values object
    for (const auto& key_value : traj) {
        // Extract the key and value from the current key-value pair
        const gtsam::Key& key = key_value.key;
        const gtsam::Vector& value = key_value.value.template cast<gtsam::Vector>();

        // Convert the gtsam key into a gtsam::Symbol, which provides more information about the key
        gtsam::Symbol symb(key);

        // If the current state or configuration corresponds to a robot position (indicated by the 'x' character)
        if (symb.chr() == 'x') {
            // Create a ROS PoseStamped message to represent the current position
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = value(0);  // X-coordinate
            pose.pose.position.y = value(1);  // Y-coordinate
            pose.pose.position.z = value(2);  // Z-coordinate

            // Add the pose to the path
            path.poses.push_back(pose);
        }
    }

    // Return the constructed path
    return path;
}



/**
 * ROS service handler to command the robot to move along the optimized path.
 * 
 * The function checks if the robot should move (based on the service request). 
 * If the robot should move, the function determines which segment of the 
 * optimized path the robot should follow next. It then publishes this 
 * segment for the robot to follow.
 * 
 * @param req Service request, containing a boolean flag indicating if the robot should move.
 * @param res Service response, indicating success and a status message.
 * @return Always returns true to indicate that the service request was processed.
 */
bool Move(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    // Acquire a lock to ensure thread safety when accessing shared resources
    std::lock_guard<std::mutex> lock(m_edt_mutex);

    // If the request data is false, the robot shouldn't move
    if (!req.data)
    {
        res.success = false;
        res.message = "Robot is not moving";
        return true;
    }

    // Use the optimized path as the current path to follow
    temp_path = optimized_path;

    // Determine the pose in the temp_path that is closest to the robot's current position
    double min_distance = std::numeric_limits<double>::max();
    size_t closest_index = 0;
    for (size_t i = 0; i < temp_path.poses.size(); i++)
    {
        double dist = sqrt(pow(temp_path.poses[i].pose.position.x - current_position(0), 2) +
                        pow(temp_path.poses[i].pose.position.y - current_position(1), 2) +
                        pow(temp_path.poses[i].pose.position.z - current_position(2), 2));
        if (dist < min_distance)
        {
            min_distance = dist;
            closest_index = i;
        }
    }

    // Set the current index to the next pose after the closest pose
    current_index = closest_index + 1;

    // Check if we are near the end of the path to avoid accessing out-of-bounds indices
    if (current_index + 5 >= temp_path.poses.size())
    {
        res.success = false;
        res.message = "End of path reached";
        return true;
    }

    // Prepare a path message to publish the next segment of poses for the robot to follow
    nav_msgs::Path path_to_publish;
    path_to_publish.header.frame_id = "map";
    path_to_publish.header.stamp = ros::Time::now();

    // Add the next 5 poses to the path message
    for (int i = 0; i < 5; i++)
    {
        if (current_index + i < temp_path.poses.size())
        {
            path_to_publish.poses.push_back(temp_path.poses[current_index + i]);
        }
    }

    // Publish the segment of the path for the robot to follow
    path_pub_opt.publish(path_to_publish);

    // Increment the current index for the next service call
    current_index += 5;

    // Indicate success in the service response
    res.success = true;
    res.message = "Robot is moving";
    return true;
}



/**
 * Timer callback function to publish the optimized path for visualization.
 * 
 * If there is a valid optimized path available, this function publishes 
 * it to a ROS topic for visualization purposes. It is designed to be 
 * called periodically based on a timer.
 * 
 * @param unused An unused parameter representing the timer event. 
 */
void optimized_path_pub(const ros::TimerEvent&)
{
    // Acquire a lock to ensure thread safety when accessing shared resources
    std::lock_guard<std::mutex> lock(m_edt_mutex);

    // Check if the optimized path is empty. If it is, exit the function.
    if (optimized_path.poses.empty())
    {
        return;
    }

    // If there's a valid optimized path, publish it for visualization
    path_pub_vis.publish(optimized_path);
}


/**
 * Callback function for the robot's pose updates.
 * 
 * This function processes incoming messages about the robot's pose. 
 * It updates the global current_position variable based on the received message.
 * If there is an available trajectory (result_isam), the function determines 
 * the point on this trajectory that is closest to the robot's current position.
 * 
 * @param msg Pointer to the received message containing the robot's pose.
 */
void robotposeCallback(const simulator_utils::Waypoint::ConstPtr& msg) 
{
    // Acquire a lock to ensure thread safety when accessing shared resources
    std::lock_guard<std::mutex> lock(m_edt_mutex);
    
    // Update the global current_position based on the received message
    current_position(0) = msg->position.x;
    current_position(1) = -1 * msg->position.y;  // Note: Y-coordinate is negated
    current_position(2) = msg->position.z;

    // Check if a trajectory (result_isam) is available
    if (!result_isam.empty())
    {
        // Initialize a large minimum distance to find the closest trajectory point
        double min_dist = 1000000;

        // Iterate over all states or configurations in result_isam
        for (const auto& key_value : result_isam) 
        {
            // Extract the key and value from the current key-value pair
            const gtsam::Key& key = key_value.key;
            const gtsam::Vector& value = key_value.value.template cast<gtsam::Vector>();

            // Convert the gtsam key into a gtsam::Symbol for more detailed information
            gtsam::Symbol symb(key);

            // If the current state or configuration corresponds to a robot position (indicated by the 'x' character)
            if (symb.chr() == 'x') 
            {
                // Calculate the Euclidean distance between the current robot position and the trajectory point
                double dist = sqrt(pow(current_position(0) - value(0), 2) +
                                   pow(current_position(1) - value(1), 2) +
                                   pow(current_position(2) - value(2), 2));

                // Update the minimum distance and index if this distance is the smallest so far
                if (dist < min_dist)
                {
                    min_dist = dist;
                    min_index = symb.index();
                }
            }
        }

        // Uncomment to print the closest index for debugging purposes
        // ROS_INFO("the closest index is: %ld", min_index);
    }

    // Uncomment to print the current position for debugging purposes
    // ROS_INFO("current position is: %f, %f, %f", current_position(0), current_position(1), current_position(2));
}


/**
 * Timer callback function to optimize the robot's trajectory.
 * 
 * This function is triggered periodically based on a timer. At each call, 
 * it acquires the latest sensor data in the form of a point cloud, 
 * updates the environmental map with this data, and subsequently optimizes 
 * the robot's trajectory in this updated environment. The optimized trajectory 
 * is then converted to a path and published for visualization or further processing.
 * 
 * @param unused An unused parameter representing the timer event. 
 */
void optimize(const ros::TimerEvent&)
{
    // Acquire a lock to ensure thread safety when accessing shared resources
    std::lock_guard<std::mutex> lock(m_edt_mutex);

    // Get the latest point cloud message
    sensor_msgs::PointCloud2ConstPtr cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/pcl_topic");
    
    // Convert the ROS point cloud message to a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    
    // Convert the PCL point cloud to an OctoMap point cloud
    octomap::Pointcloud octomapCloud;
    for(auto p : cloud.points)
    {
        octomapCloud.push_back(p.x, p.y, p.z);
    }
    
    // Define the sensor's origin
    octomap::point3d sensorOrigin(0,0,0);
    
    // Insert the point cloud data into the octree
    m_octree->insertPointCloud(octomapCloud, sensorOrigin);
    
    // Start the timer to measure the distance map updating time
    auto start = std::chrono::high_resolution_clock::now();

    // Initialize and update the distance map
    m_edt = new DynamicEDTOctomap(5.0, m_octree, octomap::point3d(-5,-5,0), octomap::point3d(5,5,3.5), false);
    m_edt->update();
    
    // Stop the timer and calculate the elapsed time
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    ROS_INFO("Time to update distance map: %f", elapsed.count()*1000);
    
    // Publish the updated octree
    octomap_msgs::Octomap updated_msg;
    updated_msg.header.frame_id = "map";
    updated_msg.header.stamp = ros::Time::now();
    octomap_msgs::binaryMapToMsg(*m_octree, updated_msg);
    pub.publish(updated_msg);

    // If it's the initial optimization
    if (start_flag)
    {
        // Extract and log distance at a specific point
        double dist = distmap.getDistance(octomath::Vector3(0,0,3));
        ROS_INFO("distance at 0,0,3 is: %f", dist);
        
        // Start a timer for the optimization process
        auto start_opt = std::chrono::high_resolution_clock::now();

        // Optimize the trajectory using both batch and isam2 optimizers
        init_values = gpmp2::initArmTrajStraightLine(start_conf, end_conf, optimizerSettings.total_step);
        result_batch = gpmp2::BatchTrajOptimizeNewPR3D(robot_model, *m_edt, start_conf, start_vel, end_conf, end_vel, init_values, optimizerSettings);
        
        // Update and initialize the factor graph for the trajectory
        optm.updateSDF(*m_edt);
        optm.initFactorGraph(start_conf, start_vel, end_conf, end_vel);
        optm.initValues(result_batch);
        optm.update();
        result_isam = optm.values();

        // Stop the timer, calculate and log the optimization time
        auto finish_opt = std::chrono::high_resolution_clock::now();
        auto elapsed_opt = std::chrono::duration_cast<std::chrono::duration<double>>(finish_opt - start_opt);
        ROS_INFO("Time to optimize: %f ms", elapsed_opt.count()*1000);

        start_flag = false;
    }
    else
    {   
        // If not the initial optimization, then it's a re-optimization
        double dist = distmap.getDistance(octomath::Vector3(0,0,3));
        ROS_INFO("distance at 0,0,3 is: %f", dist);

        auto start_opt = std::chrono::high_resolution_clock::now();
        ROS_INFO("current index is: %ld", min_index);

        // Check if current index is unchanged, if changed then update the trajectory
        if (min_index != prev_min_index)
        {
            gtsam::Vector3 current_position_gtsam = gtsam::Vector3(current_position(0), current_position(1), current_position(2));
            optm.fixConfigAndVel(min_index, current_position_gtsam, start_vel);
            prev_min_index = min_index;
        }

        // Re-optimize the trajectory
        optm.changeObstacleFactor(min_index, *m_edt);
        optm.update();
        result_isam = optm.values();

        // Stop the timer, calculate and log the re-optimization time
        auto finish_opt = std::chrono::high_resolution_clock::now();
        auto elapsed_opt = std::chrono::duration_cast<std::chrono::duration<double>>(finish_opt - start_opt);
        ROS_INFO("Time to REoptimize: %f ms", elapsed_opt.count()*1000);
    }
    
    // Convert the optimized trajectory to a path and publish it
    optimized_path = ValuestoPath(result_isam);
    path_pub_vis.publish(optimized_path);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "explore_isam_node");
    ros::NodeHandle nh;
    
    // Subscribe to the robot's current state to get its pose
    robot_pose_sub = nh.subscribe("/robot_5/current_state", 1, robotposeCallback);
    
    // Advertise topics to publish the visualized octomap and paths
    pub = nh.advertise<octomap_msgs::Octomap>("visualized_octomap", 1);                  // For visualizing the octomap
    path_pub_vis = nh.advertise<nav_msgs::Path>("path_visualize", 1);                   // For visualizing the initial path
    path_pub_opt = nh.advertise<nav_msgs::Path>("/trajectory_opt_octo2sdfisam", 1);     // For visualizing the optimized path
    
    // Create a timer to trigger the optimization function periodically
    timer2 = nh.createTimer(ros::Duration(0.5), optimize);
    
    // Advertise a ROS service to command the robot to start moving
    service = nh.advertiseService("/explore/move", Move);

    // Use a multi-threaded spinner to handle callbacks across multiple threads
    ros::MultiThreadedSpinner spinner(8);  // Use 8 threads
    spinner.spin();

    return 0;
}



#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <simulator_utils/Waypoint.h>
#include <vector>


class WaypointController
{
public:
    WaypointController()
    {
        //check robot1/optimizer_type, if 1 then use /trajectory_opt, if 2 then use /trajectory_opt_sdf
        if (nh_.getParam("robot_5/optimizer_type", optimizer_type_))
        {
            if (optimizer_type_ == 1)
            {
                path_sub_ = nh_.subscribe("/trajectory_opt_occ2sdf", 10, &WaypointController::pathCallback, this);
            }
            else if (optimizer_type_ == 2)
            {
                path_sub_ = nh_.subscribe("/trajectory_opt_octo2sdf", 10, &WaypointController::pathCallback, this);
            }
            else if (optimizer_type_ == 3)
            {
                path_sub_ = nh_.subscribe("/trajectory_opt_occ2sdfisam", 10, &WaypointController::pathCallback, this);
            }
            else if (optimizer_type_ == 4)
            {
                path_sub_ = nh_.subscribe("/trajectory_opt_octo2sdfisam", 10, &WaypointController::pathCallback, this);
            }
            else
            {
                ROS_ERROR("Invalid optimizer type parameter. Expected 1,2,3,or 4.");
            }
        }
        else
        {
            ROS_ERROR("Failed to get optimizer type parameter.");
        }
        //path_sub_ = nh_.subscribe("/trajectory_opt_sdf", 10, &WaypointController::pathCallback, this);
        current_state_sub_ = nh_.subscribe("/robot_5/current_state", 10, &WaypointController::currentStateCallback, this);
        desired_state_pub3_ = nh_.advertise<geometry_msgs::Point>("/robot_5/desired_state", 10);
        waypoint_index_ = 0;
        publish_allowed_ = false;
        std::vector<double> position;
        if (nh_.getParam("robot_5/position", position))
        {
            if (position.size() == 3)
            {
                initial_position_.x = position[0];
                initial_position_.y = position[1];
                initial_position_.z = position[2];
            }
            else
            {
                ROS_ERROR("Invalid initial position parameter. Expected 3 values.");
            }
        }
        else
        {
            ROS_ERROR("Failed to get initial position parameter.");
        }

            }

private:
    geometry_msgs::Point initial_position_;
    int optimizer_type_;
    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        path_ = msg->poses;
        for (int i = 0; i < path_.size(); i++)
        {
            path_[i].pose.position.y *= -1;
        }
        publish_allowed_ = false;  // disallow waypoint publishing

        // find the closest waypoint to the current location
        double min_distance = std::numeric_limits<double>::max();
        for (int i = 0; i < path_.size(); i++)
        {
            double distance = distanceBetween(current_location_, path_[i].pose.position);
            if (distance < min_distance)
            {
                min_distance = distance;
                waypoint_index_ = i;
            }
        }

        publish_allowed_ = true;  // allow waypoint publishing
        publishNextWaypoint();
    }

    void currentStateCallback(const simulator_utils::Waypoint::ConstPtr& msg)
    {
        current_location_ = msg->position;  // update current location

        // check if path_ is empty or waypoint publishing is not allowed
        if (path_.empty() || !publish_allowed_) return;

        // check if current state is close to the desired waypoint
        if (isClose(msg->position, path_[waypoint_index_].pose.position))
        {
            waypoint_index_++;
            publishNextWaypoint();
        }
    }

    void publishNextWaypoint()
    {
        // check if there are more waypoints to publish
        if (waypoint_index_ < path_.size())
        {
            geometry_msgs::Point point_msg;
            point_msg.x = path_[waypoint_index_].pose.position.x;
            point_msg.y = path_[waypoint_index_].pose.position.y;
            point_msg.z = path_[waypoint_index_].pose.position.z;
            desired_state_pub3_.publish(point_msg);
        }
        // else
        // {
        //     ROS_INFO("All waypoints have been published. Back to the initial point.");
        //     //back to the initial point
        //     geometry_msgs::Point point_msg;
        //     point_msg.x = initial_position_.x;
        //     point_msg.y = -1*initial_position_.y;
        //     point_msg.z = -1*initial_position_.z;
        //     desired_state_pub3_.publish(point_msg);


        // }
    }

    bool isClose(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
    {
        double distance = distanceBetween(p1, p2);
        return distance < threshold_;
    }

    double distanceBetween(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) +
                         std::pow(p1.y - p2.y, 2) +
                         std::pow(p1.z - p2.z, 2));
    }

    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber current_state_sub_;
    ros::Publisher desired_state_pub3_;
    std::vector<geometry_msgs::PoseStamped> path_;
    geometry_msgs::Point current_location_;
    int waypoint_index_;
    bool publish_allowed_;
    const double threshold_ = 0.1;  // threshold to consider if the drone is close to the waypoint
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_controller_robot5");
    WaypointController wc;
    ros::spin();
    return 0;
}

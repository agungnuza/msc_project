#!/usr/bin/env python3

# ROS and other libraries import
import rospy
from scipy import ndimage
import numpy as np
from std_msgs.msg import Float32, MultiArrayLayout, MultiArrayDimension, Float32MultiArray, Header, Bool
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2 
from std_srvs.srv import Empty, EmptyResponse
from simulator_utils.msg import Waypoint
import time

# Defining the limits for the world dimensions
x_world_limit = 120
y_world_limit = 120
z_world_limit = 80

# Defining cell size
cell_size = 0.05

# Real world limits derived from world limit and cell size
x_real_world_limit = -(x_world_limit/2)*cell_size
y_real_world_limit = -(y_world_limit/2)*cell_size
z_real_world_limit = 0

# Global publisher variables
cell_size_pub = None
pcl_pub = None
bool_pub = None
pcl_pub_2 = None

# Variable to hold robot's position
robot_position = None


# This function receives the current state of the robot and updates the robot's position globally.
# The state data typically comes from a robot state message or similar source.
def robot_state_callback(data):
    global robot_position  # Declare robot_position as global so we can modify it.
    robot_position = data.position  # Update the global robot_position with the position from the data.

# This function inserts an object into a 3D array based on provided min and max coordinates.
# It ensures that the coordinates are within the valid range of the array.
def insert_object(array_data, min_coords, max_coords):
    x_min, y_min, z_min = min_coords  # Extract minimum x, y, z coordinates from min_coords.
    x_max, y_max, z_max = max_coords  # Extract maximum x, y, z coordinates from max_coords.
    
    # Convert the coordinates to integers.
    x_min, y_min, z_min = [int(coord) for coord in min_coords]
    x_max, y_max, z_max = [int(coord) for coord in max_coords]
    
    # Ensure that the coordinates are not negative.
    x_min, y_min, z_min = max(0, x_min), max(0, y_min), max(0, z_min)
    
    # Ensure that the coordinates do not exceed the world limits.
    x_max, y_max, z_max = min(x_world_limit-1, x_max), min(y_world_limit-1, y_max), min(z_world_limit-1, z_max)
    
    # Set the 3D region defined by the coordinates to 1 in the array_data.
    array_data[x_min:x_max+1, y_min:y_max+1, z_min:z_max+1] = 1
    
    return array_data  # Return the modified array_data.

def generate_cloud():
    # Flag to indicate if the function is being called for the first time.
    initiate = True
    
    # Declare global publishers to send data.
    global array_pub, cell_size_pub, pcl_pub, bool_pub, pcl_pub_2
    
    # Check if ROS is running.
    if not rospy.is_shutdown():
        # Initialize publishers if they are None.
        if cell_size_pub is None:
            cell_size_pub = rospy.Publisher('cell_size_topic', Float32, queue_size=10)
        if pcl_pub is None:
            pcl_pub = rospy.Publisher('pcl_topic', PointCloud2, queue_size=10)
        if bool_pub is None:
            bool_pub = rospy.Publisher('bool_topic', Bool, queue_size=10)
        if pcl_pub_2 is None:
            pcl_pub_2 = rospy.Publisher('pcl_topic_2', PointCloud2, queue_size=10)

    # Create a zero-initialized 3D array.
    array_data = np.zeros((x_world_limit,y_world_limit,z_world_limit), dtype=np.uint8)
    
    # Define coordinates for the ground plane.
    min_coords_1 = [0, 0, 0]
    max_coords_1 = [x_world_limit, y_world_limit, 0]
    array_data = insert_object(array_data, min_coords_1, max_coords_1)
    
    # List to store objects' bounding boxes.
    existing_objects = []
    
    # Generate 7 random objects.
    for i in range(7):
        while True:
            # Randomly generate coordinates for a new object.
            x_min, y_min, z_min = np.random.randint(20,100), np.random.randint(20,100), np.random.randint(2,30)
            x_max, y_max, z_max = x_min + 5 + np.random.randint(4,6), y_min + 5 + np.random.randint(4,6), z_min + np.random.randint(20,80)

            # Create a bounding box with additional gap.
            new_object = (x_min-12, y_min-12, z_min-12, x_max+12, y_max+12, z_max+12)

            # Convert robot's position to array indices.
            robot_x = int((robot_position.x - x_real_world_limit) / cell_size)
            robot_y = int((robot_position.y - y_real_world_limit) / cell_size)
            robot_z = int((robot_position.z - z_real_world_limit) / cell_size)

            # Check for overlaps with robot or existing objects.
            if any([(robot_x < x_max and robot_x > x_min) and (robot_y < y_max and robot_y > y_min) and (robot_z < z_max and robot_z > z_min),
                    any(new_object[0] < obj[3] and new_object[3] > obj[0] and new_object[1] < obj[4] and new_object[4] > obj[1] and new_object[2] < obj[5] and new_object[5] > obj[2] for obj in existing_objects)]):
                continue
            else:
                existing_objects.append((x_min, y_min, z_min, x_max, y_max, z_max))  # Store the object without the gap.
                break

        # Insert the new object into the 3D array.
        array_data = insert_object(array_data, [x_min, y_min, z_min], [x_max, y_max, z_max])
        
    # Invert the array data.
    array_data_inv = 1 - array_data

    # Calculate distance transforms for the 3D array.
    start = time.time()
    map_dist = ndimage.distance_transform_edt(array_data_inv, sampling=[cell_size, cell_size, cell_size])
    inv_map_dist = ndimage.distance_transform_edt(array_data, sampling=[cell_size, cell_size, cell_size])

    # Compute the difference field.
    field = map_dist - inv_map_dist
    print("Time taken to generate field: ", time.time() - start)

    # Publish cell size.
    cell_size_pub.publish(cell_size)

    # Lists to store point cloud data.
    points_intensity, points = [], []

    # Iterate over the 3D array to generate point cloud data.
    for x, y, z in np.ndindex(array_data.shape):
        x_coord, y_coord, z_coord = x_real_world_limit + x * cell_size, y_real_world_limit + y * cell_size, z_real_world_limit + z * cell_size
        points_intensity.append([x_coord, y_coord, z_coord, field[x,y,z]])
        if array_data[x, y, z] == 1:
            points.append([x_coord, y_coord, z_coord])

    # Define headers for the point cloud messages.
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    
    # Define fields for the point cloud messages.
    fields_2 = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 8, PointField.FLOAT32, 1), PointField('intensity', 12, PointField.FLOAT32, 1)]
    fields_1 = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 8, PointField.FLOAT32, 1)]
    
    # Create point cloud messages.
    pcl_msg_2 = pc2.create_cloud(header, fields_2, points_intensity)
    pcl_msg = pc2.create_cloud(header, fields_1, points)

    # Publish initialization flag and delay for 10ms.
    bool_pub.publish(initiate)
    time.sleep(0.01)

    # Publish point cloud messages.
    pcl_pub_2.publish(pcl_msg_2)
    pcl_pub.publish(pcl_msg)

    # Clear the data after publishing.
    array_data = np.zeros((x_world_limit, y_world_limit, z_world_limit), dtype=np.uint8)
    field = np.zeros((x_world_limit, y_world_limit, z_world_limit), dtype=np.uint8)

    # Reset the initiation flag.
    initiate = False


def handle_generate_cloud_and_reset_octomap(req):
    generate_cloud()
    return EmptyResponse()

def array_publisher():
    rospy.init_node('array_publisher', anonymous=True)
    rospy.Subscriber("/robot_1/current_state", Waypoint, robot_state_callback)
    reset_and_generate_service = rospy.Service('generate_cloud_and_reset_octomap', Empty, handle_generate_cloud_and_reset_octomap)

    rospy.spin()

if __name__ == '__main__':
    try:
        array_publisher()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

# Import necessary libraries.
import rospy
from scipy import ndimage
import numpy as np
from std_msgs.msg import Float32, MultiArrayLayout, MultiArrayDimension, Float32MultiArray, Header, Bool
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2 
from std_srvs.srv import Empty, EmptyResponse
from simulator_utils.msg import Waypoint
import time

# Define world dimensions in terms of grid cells.
x_world_limit = 100
y_world_limit = 100
z_world_limit = 40

# Define the size of each cell in meters.
cell_size = 0.1

# Calculate real world limits based on grid dimensions and cell size.
x_real_world_limit = -(x_world_limit/2)*cell_size
y_real_world_limit = -(y_world_limit/2)*cell_size
z_real_world_limit = 0  # Z-axis starts at 0.

# Initialize 3D arrays to store binary data for objects and scalar fields.
array_data = np.zeros((x_world_limit,y_world_limit,z_world_limit), dtype=np.uint8)
field = np.zeros((x_world_limit,y_world_limit,z_world_limit), dtype=np.float32)
array_data2 = np.zeros((x_world_limit,y_world_limit,z_world_limit), dtype=np.uint8)
field2 = np.zeros((x_world_limit,y_world_limit,z_world_limit), dtype=np.float32)

# Initialize ROS publishers for different topics.
cell_size_pub = None
pcl_pub = None
bool_pub = None
pcl_pub_2 = None
pcl_all_pub = None

# Variable to store the robot's position.
robot_position = None

# Flag to indicate if the map has been generated.
map_generated = False

def swap():
    # Declare global variables to access and modify them within the function.
    global array_data, field, array_data2, field2
    
    # Swap contents of array_data and array_data2.
    temp_array_data = array_data.copy()  # Create a temporary copy of array_data.
    array_data = array_data2.copy()      # Copy contents of array_data2 to array_data.
    array_data2 = temp_array_data.copy() # Copy the temporary contents to array_data2.

    # Swap contents of field and field2.
    temp_field = field.copy()  # Create a temporary copy of field.
    field = field2.copy()      # Copy contents of field2 to field.
    field2 = temp_field.copy() # Copy the temporary contents to field2.


def robot_state_callback(data):
    global robot_position
    robot_position = data.position


def insert_object(array_data, min_coords, max_coords):
    # Extract minimum x, y, z coordinates from min_coords.
    x_min, y_min, z_min = min_coords
    # Extract maximum x, y, z coordinates from max_coords.
    x_max, y_max, z_max = max_coords
    
    # Convert the coordinates to integers.
    x_min, y_min, z_min = [int(coord) for coord in min_coords]
    x_max, y_max, z_max = [int(coord) for coord in max_coords]
    
    # Ensure that the coordinates are not negative.
    x_min, y_min, z_min = max(0, x_min), max(0, y_min), max(0, z_min)
    
    # Ensure that the coordinates do not exceed the world limits.
    x_max, y_max, z_max = min(x_world_limit-1, x_max), min(y_world_limit-1, y_max), min(z_world_limit-1, z_max)
    
    # Set the 3D region defined by the coordinates to 1 (indicating the presence of an object).
    array_data[x_min:x_max+1, y_min:y_max+1, z_min:z_max+1] = 1
    
    return array_data  # Return the modified array_data.

def remove_object(array_data, min_coords, max_coords):
    # Extract minimum x, y, z coordinates from min_coords.
    x_min, y_min, z_min = min_coords
    # Extract maximum x, y, z coordinates from max_coords.
    x_max, y_max, z_max = max_coords
    
    # Convert the coordinates to integers.
    x_min, y_min, z_min = [int(coord) for coord in min_coords]
    x_max, y_max, z_max = [int(coord) for coord in max_coords]
    
    # Ensure that the coordinates are not negative.
    x_min, y_min, z_min = max(0, x_min), max(0, y_min), max(0, z_min)
    
    # Ensure that the coordinates do not exceed the world limits.
    x_max, y_max, z_max = min(x_world_limit-1, x_max), min(y_world_limit-1, y_max), min(z_world_limit-1, z_max)
    
    # Set the 3D region defined by the coordinates to 0 (indicating the absence of an object).
    array_data[x_min:x_max+1, y_min:y_max+1, z_min:z_max+1] = 0
    
    return array_data  # Return the modified array_data.



def reset_octomap():
    #empty array
    global array_data, field, cell_size, robot_position, map_generated
    array_data = np.zeros((x_world_limit,y_world_limit,z_world_limit), dtype=np.uint8)
    field = np.zeros((x_world_limit,y_world_limit,z_world_limit), dtype=np.float32)
    #call generate map
    generate_map()


def generate_map():
    # Access global variables to modify them within the function.
    global array_data, field, cell_size, array_data2, field2

    # Generate the floor of the world.
    floor_min_coords = [0, 0, 0]
    floor_max_coords = [x_world_limit, y_world_limit, 2]
    array_data = insert_object(array_data, floor_min_coords, floor_max_coords)

    # Generate 40 random objects with size ranging between 8x8x30 and 10x10x30.
    # Place these objects randomly within the world.
    existing_objects = []
    for i in range(40):
        object_size_xy = np.random.randint(4, 6)
        object_size_z = np.random.randint(10, 30)
        object_min_coords = [np.random.randint(0, x_world_limit-object_size_xy), 
                             np.random.randint(0, y_world_limit-object_size_xy), 
                             np.random.randint(2, z_world_limit-object_size_z)]
        object_max_coords = [object_min_coords[0]+object_size_xy, 
                             object_min_coords[1]+object_size_xy, 
                             object_min_coords[2]+object_size_z]
        array_data = insert_object(array_data, object_min_coords, object_max_coords)
        existing_objects.append((object_min_coords, object_max_coords))

    # Invert the 3D array data.
    array_data_inv = 1 - array_data

    # Calculate the distance transforms for the 3D array.
    map_dist = ndimage.distance_transform_edt(array_data_inv, sampling=(cell_size, cell_size, cell_size))
    inv_map_dist = ndimage.distance_transform_edt(array_data, sampling=(cell_size, cell_size, cell_size))

    # Compute the difference field.
    field = map_dist - inv_map_dist

    # Copy the generated world data to a secondary array (array_data2).
    array_data2 = array_data.copy()

    # Remove a large object (size 20x20x20) located at the center of the world from array_data2.
    object_size_xy = 50
    object_size_z = 20
    object_min_coords = [int(x_world_limit/2-object_size_xy/2), 
                         int(y_world_limit/2-object_size_xy/2), 0]
    object_max_coords = [int(x_world_limit/2+object_size_xy/2), 
                         int(y_world_limit/2+object_size_xy/2), object_size_z]
    array_data2 = remove_object(array_data2, object_min_coords, object_max_coords)

    # Calculate the scalar field for the modified world in array_data2.
    array_data_inv2 = 1 - array_data2
    map_dist2 = ndimage.distance_transform_edt(array_data_inv2, sampling=(cell_size, cell_size, cell_size))
    inv_map_dist2 = ndimage.distance_transform_edt(array_data2, sampling=(cell_size, cell_size, cell_size))
    field2 = map_dist2 - inv_map_dist2

    # Print a confirmation message.
    print("Map generated")


def publish_data():
    # Start timer to measure performance.
    start = time.time()

    # Access global variables to modify them within the function.
    global array_data, field, cell_size, robot_position, map_generated

    initiate = True
    
    global cell_size_pub, pcl_pub, bool_pub, pcl_pub_2, pcl_all_pub

    # Ensure the ROS node is running.
    if not rospy.is_shutdown():
        # Initialize ROS publishers for different topics if they haven't been initialized yet.
        if cell_size_pub is None:
            cell_size_pub = rospy.Publisher('cell_size_topic', Float32, queue_size=10)
        if pcl_pub is None:
            pcl_pub = rospy.Publisher('pcl_topic', PointCloud2, queue_size=10)
        if bool_pub is None:
            bool_pub = rospy.Publisher('bool_topic', Bool, queue_size=10)
        if pcl_pub_2 is None:
            pcl_pub_2 = rospy.Publisher('pcl_topic_2', PointCloud2, queue_size=10)
        if pcl_all_pub is None:
            pcl_all_pub = rospy.Publisher('pcl_all_topic', PointCloud2, queue_size=10)

    # Publish cell size to its respective topic.
    cell_size_pub.publish(cell_size)

    # Lists to hold point cloud data.
    points_intensity = []
    points = []
    points_all = []

    # Convert robot's real-world position to its position in the grid.
    robot_x_index = int((robot_position.x - x_real_world_limit) / cell_size)
    robot_y_index = int(((-1 * robot_position.y) - y_real_world_limit) / cell_size)
    robot_z_index = int((robot_position.z - z_real_world_limit) / cell_size)

    # Define the range of the environment to consider around the robot.
    radius_index = 15

    # Determine the boundaries of this region.
    border_min_x = max(0, robot_x_index - radius_index)
    border_max_x = min(x_world_limit, robot_x_index + radius_index)
    border_min_y = max(0, robot_y_index - radius_index)
    border_max_y = min(y_world_limit, robot_y_index + radius_index)
    border_min_z = max(0, robot_z_index - radius_index)
    border_max_z = min(z_world_limit, robot_z_index + radius_index)

    # Slice the main array to get only the region around the robot.
    sliced_array_data = array_data[border_min_x:border_max_x + 1,
                                  border_min_y:border_max_y + 1,
                                  border_min_z:border_max_z + 1]
    sliced_field = field[border_min_x:border_max_x + 1,
                         border_min_y:border_max_y + 1,
                         border_min_z:border_max_z + 1]

    # Generate coordinates for each point in this sliced region.
    x_coords = np.arange(border_min_x, border_max_x + 1) * cell_size + x_real_world_limit
    y_coords = np.arange(border_min_y, border_max_y + 1) * cell_size + y_real_world_limit
    z_coords = np.arange(border_min_z, border_max_z + 1) * cell_size + z_real_world_limit

    # Create a meshgrid based on the shape of the sliced arrays.
    ix, iy, iz = np.meshgrid(np.arange(sliced_array_data.shape[0]),
                             np.arange(sliced_array_data.shape[1]),
                             np.arange(sliced_array_data.shape[2]), indexing='ij')

    # Convert this meshgrid into real-world coordinates.
    x_coords = (ix + border_min_x) * cell_size + x_real_world_limit
    y_coords = (iy + border_min_y) * cell_size + y_real_world_limit
    z_coords = (iz + border_min_z) * cell_size + z_real_world_limit

    # Flatten the coordinates and the sliced arrays for easier processing.
    x_flat = x_coords.flatten()
    y_flat = y_coords.flatten()
    z_flat = z_coords.flatten()
    array_data_flat = sliced_array_data.flatten()
    field_flat = sliced_field.flatten()

    # Create the points_intensity list, which combines coordinates with their respective scalar field values.
    points_intensity = np.column_stack((x_flat, y_flat, z_flat, field_flat))

    # Filter out only the points where an object is present (value is 1) for the points list.
    mask = array_data_flat == 1
    points = np.column_stack((x_flat[mask], y_flat[mask], z_flat[mask]))

    # For the pcl_all topic, we consider the entire world (not just the region around the robot).
    x_coords_all = np.arange(0, x_world_limit) * cell_size + x_real_world_limit
    y_coords_all = np.arange(0, y_world_limit) * cell_size + y_real_world_limit
    z_coords_all = np.arange(0, z_world_limit) * cell_size + z_real_world_limit

    # Create a meshgrid based on the shape of the entire world.
    ix_all, iy_all, iz_all = np.meshgrid(np.arange(array_data.shape[0]),
                                         np.arange(array_data.shape[1]),
                                         np.arange(array_data.shape[2]), indexing='ij')

    # Convert this meshgrid into real-world coordinates.
    x_coords_all = (ix_all) * cell_size + x_real_world_limit
    y_coords_all = (iy_all) * cell_size + y_real_world_limit
    z_coords_all = (iz_all) * cell_size + z_real_world_limit

    # Flatten these coordinates and the entire world array for easier processing.
    x_flat_all = x_coords_all.flatten()
    y_flat_all = y_coords_all.flatten()
    z_flat_all = z_coords_all.flatten()
    array_data_flat_all = array_data.flatten()
    field_flat_all = field.flatten()

    # Create the points_intensity list for the entire world, combining coordinates with their respective scalar field values.
    points_intensity_all = np.column_stack((x_flat_all, y_flat_all, z_flat_all, field_flat_all))

    # Filter out only the points where an object is present (value is 1) for the points_all list.
    mask_all = array_data_flat_all == 1
    points_all = np.column_stack((x_flat_all[mask_all], y_flat_all[mask_all], z_flat_all[mask_all]))

    # Create the point cloud messages using the ROS point_cloud2 functions.
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    fields_2 = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('intensity', 12, PointField.FLOAT32, 1)]
    
    fields_1 = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)]
    
    # Create point cloud messages using the previously generated lists.
    pcl_msg_all = pc2.create_cloud(header, fields_1, points_all)
    pcl_msg_2 = pc2.create_cloud(header, fields_2, points_intensity)
    pcl_msg = pc2.create_cloud(header, fields_1, points)

    # Publish the 'initiate' flag indicating the start of data publication.
    bool_pub.publish(initiate)

    # Publish the point cloud messages to their respective ROS topics.
    pcl_pub_2.publish(pcl_msg_2)
    pcl_pub.publish(pcl_msg)
    pcl_all_pub.publish(pcl_msg_all)

    # Stop the timer and print the time taken to publish data.
    end = time.time()
    # Uncomment the next line if you want to print the time taken for publishing data.
    # print("Time taken to publish data (ms): ", (end-start)*1000)


def handle_generate_cloud_and_reset_octomap(req):
    swap()
    return EmptyResponse()

def array_publisher():
    rospy.init_node('array_publisher', anonymous=True)
    rospy.Subscriber("/robot_5/current_state", Waypoint, robot_state_callback)
    reset_and_generate_service = rospy.Service('generate_cloud_and_reset_octomap', Empty, handle_generate_cloud_and_reset_octomap)

    #wait for the robot position to be published
    while robot_position is None:
        pass

    #generate the map
    generate_map()

    # Create a Rate object with a rate of 0.1hz
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        # Publish the data
        publish_data()
        # Sleep to maintain the desired publish rate
        rate.sleep()

if __name__ == '__main__':
    try:
        array_publisher()
    except rospy.ROSInterruptException:
        pass
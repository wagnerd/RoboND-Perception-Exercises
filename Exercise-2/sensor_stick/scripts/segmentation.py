#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
    pcl_point_cloud = ros_to_pcl(pcl_msg)

    # Voxel Grid Downsampling
    # Create a VoxelGrid filter object for our input point cloud
    vox = pcl_point_cloud.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    # Note: this (1) is a poor choice of leaf size   
    # Experiment and find the appropriate size!
    LEAF_SIZE = 0.01 

    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()
     
    # PassThrough filter
    # Create a PassThrough filter object.
    passthrough = cloud_filtered.make_passthrough_filter()
 
    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough.filter()

    # RANSAC Plane Segmentation
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()
    # Extract inliers and outliers
    # Extract inliers aka table
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)

    # Extract outliers aka objects
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)
    
    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(extracted_outliers)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.05) # 0.001
    ec.set_MinClusterSize(50) # 10
    ec.set_MaxClusterSize(2000) # 250
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    ros_cloud_cluster = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    #pcl_table_pub.publish(pcl_msg)
    pcl_table_pub.publish(ros_cloud_table)
    #pcl_objects_pub.publish(pcl_msg)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_cluster.publish(ros_cloud_cluster)

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    # Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)
    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

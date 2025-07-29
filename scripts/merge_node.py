#!/usr/bin/env python3

# Python libraries
import numpy as np
import threading
import time
#from line_profiler import LineProfiler

# OpenCV
import cv2
import cv_bridge

# ROS libraries
import rospy

# ROS Messages
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

#Custom ROS Messages
from sonar_oculus.msg import OculusPing

# Custom libraries
from merge import MergeFunctions

class MergeNode:
    """
    This ROS node merges sonar, camera, and odometry data to create a fused point cloud 
    and segmented images for enhanced underwater perception.
    """
    def __init__(self, ns="~"):
        """
        Initializes the MergeNode class, setting up ROS publishers, subscribers, and processing parameters.

        Parameters:
        - ns (str): Namespace for retrieving ROS parameters. Default is "~" (private namespace).
        """
        rospy.init_node('merge_node', anonymous=True)
        self.publish_rate = rospy.get_param("~publish_rate", 5)

        # Subscribers
        self.sonar_sub = rospy.Subscriber(rospy.get_param(ns + "sonar_sub"), OculusPing, callback=self.sonar_callback, queue_size=1)
        self.odom_sub  = rospy.Subscriber(rospy.get_param(ns + "odom_sub"), Odometry, callback=self.odom_callback, queue_size=1)
        self.image_sub = rospy.Subscriber(rospy.get_param(ns + "image_sub"), CompressedImage, self.image_callback,  queue_size = 1)
        # Publishers
        self.segmented_image_pub = rospy.Publisher(rospy.get_param(ns + "segmented_image_pub"), CompressedImage, queue_size=1)
        self.merge_cloud_pub = rospy.Publisher(rospy.get_param(ns + "merge_cloud_pub"), PointCloud2, queue_size=1)
        self.feature_image_pub = rospy.Publisher(rospy.get_param(ns + "feature_image_pub"), CompressedImage, queue_size=1)
        
        # Sonar to Camera Transfromation Matrix
        Ts_c = np.array(rospy.get_param(ns + "Ts_c"), copy=False).reshape((4, 4))

        # Get other merge parameters
        min_pix = rospy.get_param(ns + "min_area")
        threshold_inv = rospy.get_param(ns + "threshold_inv")
        boundry = rospy.get_param(ns + "boundry")
        self.fast_performance = rospy.get_param(ns + "fast_performance")
        self.scale_factor = 0.5
        if self.fast_performance:
            boundry = int(boundry*self.scale_factor)
            min_pix = int(min_pix*self.scale_factor)
        self.merge = MergeFunctions(Ts_c, min_pix, threshold_inv, boundry)

        # Get monocular camera parameters
        rgb_width = rospy.get_param(ns + "image_width")
        rgb_height = rospy.get_param(ns + "image_height")
        K = np.array(rospy.get_param(ns + "camera_matrix/data"), copy=False).reshape((3, 3))
        D = np.array(rospy.get_param(ns + "distortion_coefficients/data"), copy=False)
        if self.fast_performance:
            rgb_height = int(rgb_height*self.scale_factor)
            rgb_width = int(rgb_width*self.scale_factor)
            K = K*self.scale_factor
            K[2,2] = 1
        self.merge.set_camera_params(K, D, rgb_width, rgb_height)
       
        # Sonar Prameters
        sonar_range = rospy.get_param(ns + "sonarRange") # default value, reads in new value from msg
        detector_threshold = rospy.get_param(ns + "thresholdHorizontal")
        vertical_FOV = rospy.get_param(ns + "verticalAperture")
        sonar_features = rospy.get_param(ns + "sonar_features")
        self.merge.set_sonar_params(sonar_range, detector_threshold, vertical_FOV, sonar_features, self.fast_performance)
        #read in CFAR parameters
        Ntc = rospy.get_param(ns + "CFAR/Ntc")
        Ngc = rospy.get_param(ns + "CFAR/Ngc")
        Pfa = rospy.get_param(ns + "CFAR/Pfa")
        rank = rospy.get_param(ns + "CFAR/rank")
        # define the CFAR detector
        self.merge.init_CFAR(Ntc, Ngc, Pfa, rank)

        # define laser fields for fused point cloud
        self.laserFields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # the threading lock
        self.lock = threading.Lock()
        self.num_workers = 4 
        #self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=self.num_workers)  # Number of parallel threads
        self.latest_data = None  # Store the latest sensor data
        self.hz = 0.0
        self.cycles = 0.0
        
        # CV bridge
        self.bridge_instance = cv_bridge.CvBridge()

        # Initialize sensor information
        self.image = None
        self.pose = None

        # Reusable header object
        self.header = Header()
        self.header.frame_id = "map"
        self.image_msg = CompressedImage()
        self.image_msg.format = "jpeg"

        #self.profiler = cProfile.Profile()
        #self.lp = LineProfiler()
        #self.lp.add_function(self.merge.merge_data)
        
    def __del__(self):
        mean_hz = self.hz / self.cycles 
        rospy.loginfo(f"Total merge_data execution: {mean_hz:.4f} hz")

        
    def sonar_callback(self, msg:OculusPing)->None:
        with self.lock:
            self.latest_data = (self.image, self.pose, msg)

    def odom_callback(self, msg:Odometry)->None:
        self.pose = msg.pose.pose
    
    def image_callback(self, msg):
        #decode the compressed image
        #self.image = self.bridge_instance.compressed_imgmsg_to_cv2(msg, "bgr8")
        """ Efficiently processes incoming images by only decoding when necessary. """
        if self.image is None or msg.data != self.last_image_data:
            self.image = self.bridge_instance.compressed_imgmsg_to_cv2(msg, "bgr8")
            self.last_image_data = msg.data  # Store last image data for comparison

    def process_data(self):
        """
        Worker function that runs merge_data() in parallel.
        """
        with self.lock:
            if self.latest_data is None:
                return
            image, pose, sonar_msg = self.latest_data
            if image is None or pose is None or sonar_msg is None:
                return  # No new data available, skip this iteration
            self.latest_data = None  # Discard old data after taking it

        #self.lp.enable()
        # Run merging process
        if self.fast_performance:
            # Resize the image
            image = cv2.resize(image, None, fx=self.scale_factor, fy=self.scale_factor, interpolation=cv2.INTER_LINEAR)
        # Start the timer to measure merge_data execution time
        start_time = time.time()
        point_cloud, segmented_image, stamp, feature_image = self.merge.merge_data(image, pose, sonar_msg)
        #self.lp.disable()
        #self.lp.print_stats()

        # Publish results immediately
        if point_cloud.size > 0:
            # Stop the timer
            end_time = time.time()
            elapsed_time = end_time - start_time  # Time taken for merge_data execution
            hz = 1.0/elapsed_time
            self.hz = hz + self.hz
            self.cycles = self.cycles + 1
            # Log the time it took to execute
            rospy.loginfo(f"merge_data execution: {hz:.4f} hz")
            self.header.stamp = stamp
            cloud_msg = pc2.create_cloud(self.header, self.laserFields, point_cloud)
            self.merge_cloud_pub.publish(cloud_msg)

        if segmented_image.size > 0:
            segmented_encoded = np.array(cv2.imencode('.jpg', segmented_image)[1]).tobytes()
            feature_encoded = np.array(cv2.imencode('.jpg', feature_image)[1]).tobytes()

            # Publish segmented image
            self.image_msg.header.stamp = stamp
            self.image_msg.data = segmented_encoded
            self.segmented_image_pub.publish(self.image_msg)

            # Publish feature image
            self.image_msg.data = feature_encoded
            self.feature_image_pub.publish(self.image_msg)

    def run(self) -> None:
        """
        Main function to start multiple parallel workers.
        """
        while not rospy.is_shutdown():
            self.process_data()
        rospy.spin()  # Keep the node alive

if __name__ == "__main__":
    try:
        node = MergeNode()
        rospy.loginfo("Start sonar camera reconstruction node...")
        node.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS Interrupt Exception. Shutting down merge_node.")
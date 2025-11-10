#!/usr/bin/env python3

# Python libraries
import numpy as np
import threading
import time

# OpenCV
import cv2
import cv_bridge

# ROS libraries
import rclpy
from rclpy.node import Node

# ROS Messages
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer

#Custom ROS Messages
from sonar_oculus.msg import OculusPing

# Custom libraries
from sonar_camera_reconstruction_pkg.merge import MergeFunctions

class MergeNode(Node):
    """
    This ROS node merges sonar, camera, and odometry data to create a fused point cloud 
    and segmented images for enhanced underwater perception.
    """
    #ns="~"
    def __init__(self):
        """
        Initializes the MergeNode class, setting up ROS publishers, subscribers, and processing parameters.

        Parameters:
        - ns (str): Namespace for retrieving ROS parameters. Default is "~" (private namespace).
        """
        super().__init__('merge_node')

        self.declare_parameter('publish_rate', 5)
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value

        # Subscriber Parameters
        self.declare_parameter('sonar_topic', '/sonar_oculus_node/M750d/ping')
        sonar_topic = self.get_parameter('sonar_topic').get_parameter_value().string_value
        self.declare_parameter('odom_topic', '/bruce/slam/localization/odom')
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.declare_parameter('img_topic', '/camera/image_raw/compressed')
        image_topic = self.get_parameter('img_topic').get_parameter_value().string_value

        # Subscriptions
        #self.sonar_sub = self.create_subscription(OculusPing, sonar_topic, self.sonar_callback, 1)
        #self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 1)
        #self.image_sub = self.create_subscription(CompressedImage, image_topic, self.image_callback, 1)
        self.odom_sub = Subscriber(self, Odometry, odom_topic)
        self.image_sub = Subscriber(self, CompressedImage, image_topic)
        self.sonar_sub = Subscriber(self, OculusPing, sonar_topic)
        self.ts = ApproximateTimeSynchronizer(
            [self.odom_sub, self.image_sub, self.sonar_sub],
            queue_size=1,
            slop=0.09  # max allowed time difference in seconds
        )
        self.ts.registerCallback(self.synced_callback)
     
        # Publisher Parameters 
        self.declare_parameter('segmented_image_pub', '/sonar_camera_reconstruction/segmented_img/compressed'  )
        segmented_image = self.get_parameter('segmented_image_pub').get_parameter_value().string_value
        self.declare_parameter('merge_cloud_pub', '/sonar_camera_reconstruction/cloud')
        merge_cloud = self.get_parameter('merge_cloud_pub').get_parameter_value().string_value
        self.declare_parameter('feature_image_pub', '/sonar_camera_reconstruction/feature_img/compressed')
        feature_image = self.get_parameter('feature_image_pub').get_parameter_value().string_value

        # Publishers 
        self.segmented_image_pub = self.create_publisher(CompressedImage, segmented_image, 1)
        self.merge_cloud_pub = self.create_publisher(PointCloud2, merge_cloud, 1)
        self.feature_image_pub = self.create_publisher(CompressedImage, feature_image, 1)
        
        # Sonar to Camera Transfromation Matrix
        self.declare_parameter("Ts_c", [0.0,  -1.0,   0.0,  0.0, 
                                        0.0,   0.0,  -1.0,  0.0, 
                                        1.0,   0.0,   0.0,  0.15, 
                                        0.0,   0.0,   0.0,  1.0  ])
        Ts_c = np.array((self.get_parameter("Ts_c").get_parameter_value().double_array_value), copy=False).reshape((4, 4))

        # Get other merge parameters
        self.declare_parameter('min_area', 1000)
        self.declare_parameter('threshold_inv', True)
        self.declare_parameter('boundary', 300)
        self.declare_parameter('fast_performance', False)
        min_pix = self.get_parameter('min_area').get_parameter_value().integer_value 
        threshold_inv = self.get_parameter('threshold_inv').get_parameter_value().bool_value 
        boundary = self.get_parameter('boundary').get_parameter_value().integer_value 
        self.fast_performance = self.get_parameter('fast_performance').get_parameter_value().bool_value
        self.scale_factor = 0.5
        if self.fast_performance:
            boundary = int(boundary*self.scale_factor)
            min_pix = int(min_pix*self.scale_factor)
        self.merge = MergeFunctions(Ts_c, min_pix, threshold_inv, boundary)

        # Get monocular camera parameters
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('camera_matrix/data', [1048.9128,    0.     ,  938.53824,
                                                        0.     , 1044.4716 ,  495.39543,
                                                        0.     ,    0.     ,    1.     ])
        self.declare_parameter('distortion_coefficients/data', [-0.028647, 0.001474, -0.007251, -0.003957, 0.000000])
        
        rgb_width = self.get_parameter('image_width').get_parameter_value().integer_value
        rgb_height = self.get_parameter('image_height').get_parameter_value().integer_value
        K = np.array((self.get_parameter('camera_matrix/data').get_parameter_value().double_array_value), copy=False).reshape((3,3))
        D = np.array((self.get_parameter('distortion_coefficients/data').get_parameter_value().double_array_value), copy=False)
        if self.fast_performance:
            rgb_height = int(rgb_height*self.scale_factor)
            rgb_width = int(rgb_width*self.scale_factor)
            K = K*self.scale_factor
            K[2,2] = 1
        self.merge.set_camera_params(K, D, rgb_width, rgb_height)
       
        # Sonar Prameters
        self.declare_parameter('sonarRange', 3.0)
        self.declare_parameter('thresholdHorizontal', 65)
        self.declare_parameter('verticalAperture', 20.0)
        self.declare_parameter('sonar_features', True)
        sonar_range = self.get_parameter('sonarRange').get_parameter_value().double_value
        detector_threshold = self.get_parameter('thresholdHorizontal').get_parameter_value().integer_value
        vertical_FOV = self.get_parameter('verticalAperture').get_parameter_value().double_value
        sonar_features = self.get_parameter('sonar_features').get_parameter_value().bool_value
        self.merge.set_sonar_params(sonar_range, detector_threshold, vertical_FOV, sonar_features, self.fast_performance)
       
        #read in CFAR parameters
        self.declare_parameter("CFAR/Ntc", 40)
        self.declare_parameter("CFAR/Ngc", 10)
        self.declare_parameter("CFAR/Pfa", 0.1)
        self.declare_parameter("CFAR/rank", 10)

        Ntc = self.get_parameter("CFAR/Ntc").get_parameter_value().integer_value
        Ngc = self.get_parameter("CFAR/Ngc").get_parameter_value().integer_value
        Pfa = self.get_parameter("CFAR/Pfa").get_parameter_value().double_value
        rank = self.get_parameter("CFAR/rank").get_parameter_value().integer_value
        
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
    
    # called when node is deleted 
    def __del__(self):
       mean_hz = self.hz / self.cycles 
       #self.get_logger().info(f"Total merge_data execution: {mean_hz:.4f} hz")

    '''
    # called when the node recieves an OculusPing msg
    def sonar_callback(self, msg:OculusPing)->None:
        with self.lock:
            self.latest_data = (self.image, self.pose, msg)

    # called when the node recieves an Odometry msg
    def odom_callback(self, msg:Odometry)->None:
        self.pose = msg.pose.pose
    
    # called when the node recieves a CompressedImage msg
    def image_callback(self, msg):
        #decode the compressed image
        """ Efficiently processes incoming images by only decoding when necessary. """
        if self.image is None or msg.data != self.last_image_data:
            self.image = self.bridge_instance.compressed_imgmsg_to_cv2(msg, "bgr8")
            self.last_image_data = msg.data  # Store last image data for comparison
    '''
    def synced_callback(self, odom_msg, image_msg, sonar_msg):
        if self.image is None or image_msg.data != self.last_image_data:
            self.image = self.bridge_instance.compressed_imgmsg_to_cv2(image_msg, "bgr8")
            self.last_image_data = image_msg.data  # Store last image data for comparison
        pose = odom_msg.pose.pose
        self.latest_data = (self.image, pose, sonar_msg)
        #self.process_data(image, pose, sonar_msg)

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
        
        # Run merging process
        if self.fast_performance:
            # Resize the image
            image = cv2.resize(image, None, fx=self.scale_factor, fy=self.scale_factor, interpolation=cv2.INTER_LINEAR)
        # Start the timer to measure merge_data execution time
        start_time = time.time()
        point_cloud, segmented_image, stamp, feature_image = self.merge.merge_data(image, pose, sonar_msg)

        # Publish results immediately
        if point_cloud.size > 0:
            # Stop the timer
            end_time = time.time()
            elapsed_time = end_time - start_time  # Time taken for merge_data execution
            hz = 1.0/elapsed_time
            self.hz = hz + self.hz
            self.cycles = self.cycles + 1
            # Log the time it took to execute
            # self.get_logger().info(f"merge_data execution: {hz:.4f} hz")
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


def main(args=None):
    rclpy.init(args=args)
    node = MergeNode()
    node.get_logger().info("Start sonar camera reconstruction node...")
   
    try:
        while rclpy.ok():
            node.process_data()
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().warn("ROS Interrupt Exception. Shutting down merge_node.")
    finally:
        
        rclpy.shutdown()

if __name__ == '__main__':
    main()
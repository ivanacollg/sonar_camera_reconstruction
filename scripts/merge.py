# Python libraries
import numpy as np
from scipy.interpolate import interp1d
import threading
import warnings
#from line_profiler import LineProfiler

# OpenCV
import cv2
import open3d as o3d

# Custom libraries
from imaging_sonar import ImagingSonar
from monocular_camera import MonocularCamera

# Ros 
from tf.transformations import euler_from_quaternion, euler_matrix

class MergeFunctions:
    """
    A class to handle merging of sonar and camera data for underwater robotics applications.

    Attributes:
        Ts_c (np.ndarray): Transformation matrix from sonar to camera frame.
        minpixnum (int): Minimum number of pixels required for a valid segmented region.
        threshold_inv (int): Threshold value for image segmentation.
        sonar_msg (object): Holds sonar message data.
        pose (object): Holds robot pose data.
        image (np.ndarray): Holds the captured image data.
        color_map (np.ndarray): Predefined color mapping for visualizing clusters.
        xyz_aggregated (np.ndarray): Stores the aggregated 3D point cloud data.
        lock (threading.RLock): Lock for handling concurrency.
    """
    def __init__(self, Ts_c, minpixnum, threshold_inv, boundry):
        """
        Initializes the MergeFunctions class.

        Args:
            Ts_c (np.ndarray): Transformation matrix from sonar to camera frame.
            minpixnum (int): Minimum number of pixels for valid segmentation.
            threshold_inv (int): Threshold for image preprocessing.
            boundry (int): Boundry image threshold
        """
        # sonar to camera transfromation
        self.translation = Ts_c[:3, 3]
        self.rotation = Ts_c[:3,:3]
        # Initialize sensor information
        self.sonar_msg = None
        self.pose = None
        self.image = None

        self.minpixnum=minpixnum
        self.threshold_inv = threshold_inv
        self.boundry = boundry

        self.color_map = np.array([
                    [1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [0.0, 1.0, 0.0],
                    [0.25,0.75,0.25],
                    [1.0,1.0,0.0],
                    [.44,.62,.8118],
                    [0.8118,0.44,0.62],
                    [0.62,0.8118,0.44],
                    [0.75,0.25,0.25],
                    [0.25,0.75,0.25],
                    [0.25,0.25,0.75],
                    [0.1,0.45,0.7],
                    [0.7,0.45,0.1],
                    [0.45,0.7,0.1],
                    [0.45,0.1,0.7],
                    [0.1,0.7,0.45]])
                
                
        self.xyz_aggregated = np.zeros(0)
        # the threading lock
        self.lock = threading.Lock()

    
    def set_camera_params(self, K, D, rgb_width, rgb_height):
        """
        Sets camera parameters.

        Args:
            K (np.ndarray): Intrinsic camera matrix.
            D (np.ndarray): Distortion coefficients.
            rgb_width (int): Width of the RGB image.
            rgb_height (int): Height of the RGB image.
        """
        self.monocular_camera = MonocularCamera(K, D, rgb_width, rgb_height)
    
    def set_sonar_params(self, sonar_range, detector_threshold, vertical_FOV, sonar_features, fast_performance):
        """
        Sets sonar parameters.

        Args:
            sonar_range (float): Maximum range of the sonar.
            detector_threshold (float): Detection threshold for sonar processing.
            vertical_FOV (float): Vertical field of view of the sonar.
            sonar_features (bool): Determines if sonar features are highlited n the image.
            fast_performance (bool): Determines if fast or detailed perfomanc paramterers are used.
        """
        self.imaging_sonar = ImagingSonar(sonar_range, detector_threshold, vertical_FOV, sonar_features, fast_performance)
        #self.lp = LineProfiler()
        #self.lp.add_function(self.imaging_sonar.get_sonar_scanline)

    def init_CFAR(self, Ntc, Ngc, Pfa, rank):
        """
        Initializes CFAR (Constant False Alarm Rate) detection for sonar.

        Args:
            Ntc (int): Number of training cells.
            Ngc (int): Number of guard cells.
            Pfa (float): Probability of false alarm.
            rank (int): Ranking order for CFAR detection.
        """
        self.imaging_sonar.init_CFAR(Ntc, Ngc, Pfa, rank)

    '''
    def set_sensor_info(self, image, pose, sonar_msg):
        """
        Updates sensor information.

        Args:
            image (np.ndarray): Camera image.
            pose (object): Pose information of the robot.
            sonar_msg (object): Sonar message data.
        """
        with self.lock:
            self.image= image
            self.pose = pose
            self.sonar_msg = sonar_msg
    '''

    def rotate_cloud(self, t, R, new_cloud):
        """
        Rotates and transforms a point cloud from the body frame to the map frame.

        Args:
            t (np.ndarray): Translation vector (3x1).
            R (np.ndarray): Rotation matrix (3x3).
            new_cloud (np.ndarray): Input point cloud (Nx3).

        Returns:
            np.ndarray: Transformed point cloud (Nx3).
        """
        # Build Homogeneous tranform matrix
        H = np.row_stack((np.column_stack((R, t.T)), np.array([0, 0, 0, 1])))

        # Change of cloud points to homogeneous
        x = new_cloud[:, 0]
        z = new_cloud[:, 2]
        y = new_cloud[:, 1]
        xyzw = np.column_stack((x, y, z, np.ones_like(x)))
       
        # Transform points to map reference frame
        xyzw_map = np.matmul(H, xyzw.T).T
        xyzw_map[:,0] = np.divide(xyzw_map[:, 0], xyzw_map[:, 3])
        xyzw_map[:,1] = np.divide(xyzw_map[:, 1], xyzw_map[:, 3])
        xyzw_map[:,2] = np.divide(xyzw_map[:, 2], xyzw_map[:, 3])

        return xyzw_map[:, 0:3]


    def merge_data(self, image, pose, sonar_msg):
        """
        Merges sonar and camera data to generate a 3D point cloud.

        Returns:
            tuple:
                - np.ndarray: Aggregated 3D point cloud.
                - np.ndarray: Processed image with depth overlay.
                - object: Timestamp of the sonar message.
                - np.ndarray: Feature image from sonar processing.
        """
        #with self.lock:
        #    sonar_msg = self.sonar_msg
        #    image = self.image
        #    pose = self.pose

        if sonar_msg is not None and image is not None and pose is not None:
            stamp = sonar_msg.header.stamp
            #self.lp.enable()
            # Get filtered sonar scanline features
            scan_line, feature_image = self.imaging_sonar.get_sonar_scanline(sonar_msg)
            #self.lp.disable()
            #self.lp.print_stats()

            if scan_line.shape[1] > 0:
                # Apply DBSCAN
                # Number of clusters (excluding noise if present)
                num_clusters, cluster_labels = self.imaging_sonar.cluster_scanline(scan_line[:, 0:3])

                # Filter image -> returns black and white image segmenting foreground and background
                thresholded_image = self.monocular_camera.preprocess(image, self.threshold_inv)
                # Segment Image -> returns labeled segemnted image and the labels
                labels, labeled_image = self.monocular_camera.segment_image(thresholded_image)

                # Initialize variables
                contours_list = []
                depth_img_color = image.copy()
                cloud_from_img = None
                cloud_from_scan = None
                # Initialize blank image to be used to adding true contact regions one at a time
                contact_image = np.zeros((self.monocular_camera.height, self.monocular_camera.width)).astype(np.uint8)
                # Iterate through areas of the image
                for label in labels[2:]:  
                    # Create a binary image in which only the area of the label is in the foreground 
                    #and the rest of the image is in the background   
                    target_label = np.where(labeled_image == label, 255, 0).astype(np.uint8)
                    # Take only the area that is segmented correctly in the binary image and target label image
                    area_image = cv2.bitwise_and(target_label, thresholded_image)
                    # Filter Out edges of Image
                    area_image[-self.boundry:, :] = 0
                    area_image[:, -self.boundry:] = 0
                    area_image[:, 0:self.boundry] = 0
                    # Add area into final contact image
                    contact_image = cv2.bitwise_or(area_image, contact_image)
                    # Count number of contact pixels in the area
                    n_white_pix = np.sum(area_image == 255)
                    # If area is large enough then:
                    if n_white_pix > self.minpixnum: 
                        # Perform contour extraction on the created binary image
                        contours, hierarchy = cv2.findContours(
                            area_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                        )
                        #rospy.loginfo(n_white_pix)
                        contours_list.append(contours[0])

                        # Initialize variables to itirate through clusters
                        min_dist = np.inf
                        chosen_cluster = None
                        chosen_extended_coordinates = None
                        chosen_in_bound_indx = None
                        chosen_indx_coord = None
                        for i in range(0,num_clusters):
                            filtered_scan = scan_line[np.where(i==cluster_labels)[0]]
                            # Compute polar coordinates
                            r = np.sqrt((filtered_scan[:, 0])**2 + (filtered_scan[:, 1])**2)  # Radius
                            theta = np.arctan2(filtered_scan[:, 1], filtered_scan[:, 0])  # Angle in radians
                            # Compute extended cartesian coordomates to include all elevation angles 
                            extended_coordinates = self.imaging_sonar.get_extended_coordinates(r, theta)
                            # Transfroma points from sonar to camera reference frame
                            extended_coordinates = np.matmul(self.rotation, extended_coordinates.T).T + self.translation

                            # Get 2D coordinates
                            xyw = np.matmul(self.monocular_camera.K, extended_coordinates.T).T
                            #xyw[:,0] = np.divide(xyw[:, 0], xyw[:, 2])
                            #xyw[:,1] = np.divide(xyw[:, 1], xyw[:, 2])
                            #xyw[:,2] = np.divide(xyw[:, 2], xyw[:, 2])
                            xyw[:, :2] /= xyw[:, 2:3]  # Vectorized division
                            xy = np.round(xyw)[:,0:2].astype(np.int32)

                            # Change 2D coordinates to pixel coordinates
                            uv = np.column_stack((xy[:,1], xy[:,0]))

                            #uv coordenate indx that are valid
                            in_bound_indx = np.where((uv[:, 0] >= 0) & (uv[:,1]>=0) & (uv[:,0]<self.monocular_camera.height) & (uv[:,1]<self.monocular_camera.width))[0]

                            # uv coordinates that are valid
                            indx_coord = uv[in_bound_indx].astype(int)

                            # Get overlap
                            cluster_image = np.zeros((self.monocular_camera.height, self.monocular_camera.width))
                            cluster_image[indx_coord[:,0], indx_coord[:,1]] = 255
                            cluster_image= cluster_image.astype(np.uint8)
                            overlap_image = cv2.bitwise_and(cluster_image, area_image)
                            n_white_pix = np.sum(overlap_image == 255)

                            # If there is an overlap between cluster and area in the image
                            if n_white_pix > 0:
                                cluster_pose = np.mean(filtered_scan, axis= 0)
                                dist = cluster_pose[0]#np.linalg.norm(cluster_pose)
                                if dist < min_dist:
                                    min_dist = dist
                                    chosen_cluster = i
                                    chosen_extended_coordinates = extended_coordinates
                                    chosen_in_bound_indx = in_bound_indx
                                    chosen_indx_coord = indx_coord


                        if chosen_cluster is not None: 
                            filtered_scan = scan_line[np.where(chosen_cluster==cluster_labels)[0]]

                            distance_values = chosen_extended_coordinates[chosen_in_bound_indx, 2]

                            # Color pixels
                            cluster_image = np.zeros((self.monocular_camera.height, self.monocular_camera.width))
                            cluster_image[chosen_indx_coord[:,0], chosen_indx_coord[:,1]] = 255
                            cluster_image= cluster_image.astype(np.uint8)
                            cluster_distance_image = np.zeros((self.monocular_camera.height, self.monocular_camera.width))
                            cluster_distance_image[chosen_indx_coord[:,0], chosen_indx_coord[:,1]] = distance_values
                            overlap_image = cv2.bitwise_and(cluster_image, area_image)

                            overlap_distance_image = np.ones((self.monocular_camera.height, self.monocular_camera.width))*np.nan # initialize as nan 
                            overlap_distance_image[overlap_image == 255] = cluster_distance_image[overlap_image ==255]

                            # Calculate the mean of nonzero values in each column (avoid division by zero)
                            # Suppress only the specific warning
                            with warnings.catch_warnings():
                                warnings.simplefilter("ignore", category=RuntimeWarning)
                                column_means = np.nanmean(overlap_distance_image, axis=0)
                                column_means = np.nan_to_num(column_means, nan=0)  # Replace NaN with 0 or another default value
                                
                            # Mean depth values are assigned to the entire column in the image
                            mean_distance_image = np.tile(column_means, (self.monocular_camera.height, 1))
                            mean_distance_mask = (mean_distance_image > 0).astype(np.uint8) * 255
                            overlap_image = cv2.bitwise_and(mean_distance_mask, area_image)

                            depth_image = np.zeros((self.monocular_camera.height, self.monocular_camera.width))
                            # Apply condition: If img1 == 255, assign value from img2; otherwise, assign 0
                            depth_image[overlap_image == 255] = mean_distance_image[overlap_image == 255]

                            color_indx = chosen_cluster % len(self.color_map)
                            depth_img_color[np.where(cluster_image==255)]=self.color_map[color_indx]*255

                            # Add all points from RGB area found
                            indx = np.where(depth_image!=0)
                            final_distance_values = depth_image[indx]
                            final_distance_values = final_distance_values[:, np.newaxis] # reshape 
                            xyw = np.array([indx[1], indx[0], np.ones(indx[1].shape)])
                            s = 1
                            coord_3d = (1/s)*(final_distance_values)*(np.matmul(np.linalg.inv(self.monocular_camera.K),xyw).T)

                            if cloud_from_img is None:
                                cloud_from_img = coord_3d
                                cloud_from_scan = filtered_scan#occ_points
                            else:
                                cloud_from_img = np.vstack((cloud_from_img, coord_3d))
                                cloud_from_scan = np.vstack((cloud_from_scan, filtered_scan))#np.vstack((cloud_from_scan, occ_points))

                # Draw the outline
                cv2.drawContours(depth_img_color, contours_list, -1, color=(0, 23, 223), thickness=5)
            
                if cloud_from_img is not None:
                    # Transfrom to Sonar frame
                    xyz_cloud = np.matmul((cloud_from_img-self.translation), self.rotation)

                    # Get translation
                    t = np.array([pose.position.x,pose.position.y,pose.position.z])# Centered on robot center
                    # Get rotation
                    # Convert quaternion to rotation matrix
                    quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                    roll, pitch, yaw = euler_from_quaternion(quaternion)
                    R = euler_matrix(roll, 0, yaw)[:3, :3]
                    xyz_cloud = self.rotate_cloud(t, R, xyz_cloud)

                    if self.xyz_aggregated.size > 0: 
                        new_cloud = np.row_stack((xyz_cloud, self.xyz_aggregated))
                        
                        pcd = o3d.geometry.PointCloud()
                        pcd.points = o3d.utility.Vector3dVector(new_cloud)
                        # Downsample the point cloud using a voxel grid filter
                        voxel_size = 0.01  # Adjust the voxel size to control the downsampling level
                        downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
                        sampled_cloud = np.asarray(downsampled_pcd.points)
                        
                        self.xyz_aggregated = sampled_cloud#np.row_stack((sampled_cloud, self.xyz_aggregated))
                    else:
                        self.xyz_aggregated = xyz_cloud
            
            np.save('aggregated_cloud.npy', self.xyz_aggregated)
            return self.xyz_aggregated, depth_img_color, stamp, feature_image
        else:
            return np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0)
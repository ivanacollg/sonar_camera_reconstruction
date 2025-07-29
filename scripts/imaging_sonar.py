# Python libraries
import numpy as np
from scipy.interpolate import interp1d
from sklearn.cluster import DBSCAN

# OpenCV
import cv2
import open3d as o3d

# Custom libaries
from CFAR import *

cv2.setNumThreads(4)  # Adjust the number based on your CPU cores
cv2.setUseOptimized(True)

class ImagingSonar:
    """Class to handle operations related to an imaging sonar system.

    Attributes:
        sonar_range (float): Maximum sonar range in meters.
        detector_threshold (float): Threshold for CFAR detection.
        vertical_FOV (float): Vertical field of view in degrees.
        res (float): Resolution of the sonar scan (initialized as None).
        height (float): Sonar scan height (initialized as None).
        rows (int): Number of rows in the sonar image (initialized as None).
        width (float): Sonar scan width (initialized as None).
        cols (int): Number of columns in the sonar image (initialized as None).
        map_x (numpy.ndarray): X-coordinate mapping for image transformation.
        map_y (numpy.ndarray): Y-coordinate mapping for image transformation.
        f_bearings (callable): Interpolated function for bearing angles.
        REVERSE_Z (int): Factor to adjust Z-axis direction.
    """

    def __init__(self, sonar_range, detector_threshold, vertical_FOV, sonar_features, fast_performance):
        """
        Initializes the ImagingSonar object.

        Args:
            sonar_range (float): Maximum sonar detection range in meters.
            detector_threshold (float): Threshold value for sonar detection.
            vertical_FOV (float): Vertical field of view in degrees.
            sonar_features (bool): Determines if we highlight sonar featues in image.
            fast_performance (bool): Determines if fast or detaild parameters are used.
        """
        self.sonar_range = sonar_range
        self.detector_threshold = detector_threshold
        self.vertical_FOV = vertical_FOV
        self.sonar_features = sonar_features
        # for remapping from polar to cartisian
        self.res = None
        self.height = None
        self.rows = None
        self.width = None
        self.cols = None
        self.map_x = None
        self.map_y = None
        self.f_bearings = None
        self.to_rad = lambda bearing: bearing * np.pi / 18000
        self.REVERSE_Z = 1
        self.fast_performance = fast_performance

    def init_CFAR(self, Ntc, Ngc, Pfa, rank):
        """
        Initializes the CFAR (Constant False Alarm Rate) detector.

        Args:
            Ntc (int): Number of training cells.
            Ngc (int): Number of guard cells.
            Pfa (float): Probability of false alarm.
            rank (int): Ranking parameter for CFAR.
        """
        self.detector = CFAR(Ntc, Ngc, Pfa, rank)

    def generate_map_xy(self, ping):
        """
        Generates a mesh grid for mapping the sonar image from polar to Cartesian coordinates.

        Args:
            ping (OculusPing): A sonar ping message containing range and bearing data.
        """
        # get the parameters from the ping message
        _res = ping.range_resolution
        _height = ping.num_ranges * _res
        _rows = ping.num_ranges
        _width = (
            np.sin(self.to_rad(ping.bearings[-1] - ping.bearings[0]) / 2) * _height * 2
        )
        _cols = int(np.ceil(_width / _res))
        # check if the parameters have changed
        if (
            self.res == _res
            and self.height == _height
            and self.rows == _rows
            and self.width == _width
            and self.cols == _cols
        ):
            return
        # if they have changed do some work
        self.res, self.height, self.rows, self.width, self.cols = (
            _res,
            _height,
            _rows,
            _width,
            _cols,
        )
        # generate the mapping
        bearings = self.to_rad(np.asarray(ping.bearings, dtype=np.float32))
        f_bearings = interp1d(
            bearings,
            range(len(bearings)),
            kind="linear",
            bounds_error=False,
            fill_value=-1,
            assume_sorted=True,
        )
        # build the meshgrid
        XX, YY = np.meshgrid(range(self.cols), range(self.rows))
        x = self.res * (self.rows - YY)
        y = self.res * (-self.cols / 2.0 + XX + 0.5)
        b = np.arctan2(y, x) * self.REVERSE_Z
        r = np.sqrt(np.square(x) + np.square(y))
        self.map_y = np.asarray(r / self.res, dtype=np.float32)
        self.map_x = np.asarray(f_bearings(b), dtype=np.float32)

        # check for change in max range
        if self.sonar_range != self.height:
            self.sonar_range = self.height

    def extract_line_scan(self,peaks: np.array) -> np.array:
        """
        Extracts a line scan from a downward-looking sonar.

        Args:
            peaks (np.array): The CFAR-detected peak image.

        Returns:
            np.array: The extracted line scan image.
        """
        # extract the first contact in each column 
        peaks_rot = np.rot90(peaks) # rotate the peaks to we can work with columns 
        '''
        blank = np.zeros_like(peaks_rot) # make a blank image copy
        for i,col in enumerate(peaks_rot): # loop
            j = np.argmax(col)
            if peaks_rot[i][j] != 0:
                blank[i][j] = 255
        '''
        idx = np.argmax(peaks_rot, axis=1)  # Get first nonzero pixel in each column
        valid = peaks_rot[np.arange(len(peaks_rot)), idx] != 0  # Filter valid peaks
        blank = np.zeros_like(peaks_rot)
        blank[np.arange(len(peaks_rot))[valid], idx[valid]] = 255
        return np.rot90(blank,3)

    def get_sonar_scanline(self, sonar_msg):
        """
        Processes a sonar ping message to extract scanlines, detect features, 
        and generate a 3D point cloud.

        Args:
            sonar_msg (OculusPing): A sonar ping message.

        Returns:
            tuple:
                - numpy.ndarray: Processed 3D point cloud.
                - numpy.ndarray: Color-mapped sonar image with detected features.
        """
        img = np.frombuffer(sonar_msg.ping.data, np.uint8)
        # decode compresed sonar image
        sonar_img = np.array(cv2.imdecode(img, cv2.IMREAD_COLOR)).astype(np.uint8)
        sonar_img = cv2.cvtColor(sonar_img, cv2.COLOR_BGR2GRAY)
        if sonar_img.size > 0:
            # generate the mapping from polar to cartisian
            self.generate_map_xy(sonar_msg)
            # denoise the horizontal image, consider adding this for the vertical image
            if not self.fast_performance:
                sonar_img = cv2.fastNlMeansDenoising(sonar_img, None, 10, 7, 21)
            # get some features using CFAR
            # Detect targets and check against threshold using CFAR (in polar coordinates)
            peaks = self.detector.detect(sonar_img, "SOCA")
            peaks &= sonar_img > self.detector_threshold

            line_scan = self.extract_line_scan(peaks)

            # create a vis image 
            feature_image = cv2.applyColorMap(sonar_img, 2)
            if self.sonar_features:
                for point in np.c_[np.nonzero(line_scan)]:
                    cv2.circle(feature_image,(point[1],point[0]),3,(0,0,255),-1)
            feature_image = cv2.remap(feature_image, self.map_x, self.map_y, cv2.INTER_LINEAR)

            # extract the line scan
            line_scan = cv2.remap(line_scan, self.map_x, self.map_y, cv2.INTER_LINEAR)        
            locs = np.c_[np.nonzero(line_scan)]
            
            #convert from image coords to meters
            x = locs[:,1] - self.cols / 2.
            x = (-1 * ((x / float(self.cols / 2.)) * (self.width / 2.)))
            y = (-1*(locs[:,0] / float(self.rows)) * self.height) + self.height
            points = np.c_[y,  x, np.zeros(len(x))]

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            # Remove statistical outliers
            # Parameters:
            # nb_neighbors: The number of neighbors to analyze for each point
            # std_ratio: The standard deviation ratio for filtering outliers
            cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

            # Downsample the point cloud using a voxel grid filter
            voxel_size = 0.01  # Adjust the voxel size to control the downsampling level
            downsampled_pcd = cl.voxel_down_sample(voxel_size=voxel_size)
            sampled_cloud = np.asarray(downsampled_pcd.points)

            return sampled_cloud, feature_image
        else:
            return np.zeros((0))

    def cluster_scanline(self, scan_line):
        """
        Applies DBSCAN clustering to a sonar scanline.

        Args:
            scan_line (numpy.ndarray): The extracted scanline.

        Returns:
            tuple:
                - int: Number of detected clusters.
                - numpy.ndarray: Ordered cluster labels.
        """
        dbscan = DBSCAN(eps=0.2, min_samples=2)
        cluster_labels = dbscan.fit_predict(scan_line)

        # Identify unique labels and map them to ordered integers
        unique_labels = np.unique(cluster_labels)
        label_mapping = {label: idx for idx, label in enumerate(unique_labels)}
        ordered_labels = np.array([label_mapping[label] for label in cluster_labels])

        # Number of clusters (excluding noise if present)
        num_clusters = len(unique_labels) - (1 if -1 in unique_labels else 0)

        # Generate colors based on the ordered cluster labels
        ordered_labels = ordered_labels.astype(int)

        return num_clusters, ordered_labels

    
    def get_extended_coordinates(self, r_values, theta_values, num_phi_samples=300):
        """
        Computes 3D coordinates from sonar scan data.

        Args:
            r_values (array-like): Radius values.
            theta_values (array-like): Theta values in radians.
            num_phi_samples (int): Number of phi samples.

        Returns:
            numpy.ndarray: 3D points representing sonar scans.
        """
        if self.fast_performance:
            num_phi_samples = 100
        # Convert r_values and theta_values to NumPy arrays for vectorized operations
        r_values = np.asarray(r_values)[:, None, None]  # Shape (num_r, 1, 1)
        theta_values = np.asarray(theta_values)[None, :, None]  # Shape (1, num_theta, 1)
        
        # Generate phi values
        phi_max = np.radians(self.vertical_FOV/2)
        phi_min = -phi_max
        phi_values = np.linspace(phi_max, phi_min, num_phi_samples)  # Shape (num_phi,)
        # Precompute trigonometric values
        cos_phi = np.cos(phi_values)  # Shape (num_phi,)
        sin_phi = np.sin(phi_values)
        cos_theta = np.cos(theta_values)  # Shape (1, num_theta, 1)
        sin_theta = np.sin(theta_values)

        # Compute P_s using correct broadcasting
        x = cos_phi[None, None, :] * cos_theta  # Shape (1, num_theta, num_phi)
        y = cos_phi[None, None, :] * sin_theta  # Shape (1, num_theta, num_phi)
        z = sin_phi[None, None, :]              # Shape (1, 1, num_phi) ✅ FIXED
        # Ensure z is properly broadcasted to match (1, num_theta, num_phi)
        z = np.broadcast_to(z, x.shape)  # Shape (1, num_theta, num_phi)

        # Stack correctly along the last axis
        P_s = r_values * np.stack((x, y, z), axis=-1)  # Shape (1, num_r, num_phi, 3)
        P_s = P_s[0].reshape(-1, 3)  # Collapse (num_r, num_phi) to (num_r * num_phi, 3)

        return P_s
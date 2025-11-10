# Python libraries
import numpy as np

# OpenCV
import cv2

cv2.setNumThreads(4)  # Adjust the number based on your CPU cores
cv2.setUseOptimized(True)

class MonocularCamera:
    """Class to handle operations related to a monocular camera, including preprocessing 
    and segmentation of images.

    Attributes:
        K (numpy.ndarray): The optimized intrinsic camera matrix.
        height (int): The height of the RGB image.
        width (int): The width of the RGB image.
        kernel (numpy.ndarray): Structuring element used for morphological operations.
    """

    def __init__(self, K, D, rgb_width, rgb_height):
        """
        Initializes the MonocularCamera object.
        """
        self.K, roi = cv2.getOptimalNewCameraMatrix(K, D, (rgb_width, rgb_height), 1, (rgb_width,rgb_height))
        self.height = rgb_height
        self.width = rgb_width

        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))


    def preprocess(self, image, threshold_inv):
        """
        Preprocesses the input image by converting it to grayscale, applying Gaussian 
        blur, adaptive thresholding, and noise removal.

        Args:
            image (numpy.ndarray): The input color image (BGR format).
            threshold_inv (bool): If True, applies inverse adaptive thresholding (for dark areas).
                                  If False, applies regular adaptive thresholding (for light areas).

        Returns:
            numpy.ndarray: The processed binary image.
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Apply Gaussian blur
        gray = cv2.GaussianBlur(gray, (15, 15), 0)

        # Apply adaptive thresholding
        if threshold_inv:
            gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 501, 0)
        else:
            gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 501, 0)

        # Noise removal using morphological opening
        gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, self.kernel, iterations=3)
        return gray
    

    def segment_image(self, image):
        """
        Segments the input binary image using morphological operations, distance 
        transformation, and the watershed algorithm.

        Args:
            image (numpy.ndarray): The preprocessed binary image.

        Returns:
            tuple:
                - numpy.ndarray: An array containing the unique labels of segmented regions.
                - numpy.ndarray: The labeled image after applying the watershed algorithm.
        """
        # Obtain sure background area
        sure_bg = cv2.dilate(image, self.kernel, iterations=2)

        # Compute the distance transform
        dist = cv2.distanceTransform(image, cv2.DIST_L2, 5)

        # Obtain sure foreground area
        _, sure_fg = cv2.threshold(dist, 0.5 * dist.max(), 255, cv2.THRESH_BINARY)
        sure_fg = sure_fg.astype(np.uint8)

        # Determine unknown region
        unknown_area = cv2.subtract(sure_bg, sure_fg)

        # Label connected components
        _, labeled_image = cv2.connectedComponents(sure_fg)

        # Adjust labels: Background becomes 1 instead of 0
        labeled_image += 1

        # Mark unknown region as 0
        labeled_image[unknown_area == 255] = 0

        # Convert grayscale image to color for watershed
        color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        # Apply watershed algorithm
        labeled_image = cv2.watershed(color_image, labeled_image)

        # Get unique labels
        labels = np.unique(labeled_image)

        return labels, labeled_image




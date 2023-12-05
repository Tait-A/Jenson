# A module to take camera input from the robot and process it to extract the features from the image

import cv2
import numpy as np
import os
from collections import deque

# Feature Extractor class to extract features from images taken by the camera


class FeatureExtractor:
    def __init__(self, camera=None, track=None):
        self.camera = camera
        self.track = track
        self.algorithm = cv2.ORB_create()
        self.featureQueue = deque(maxlen=20)
        self.path = "//Output/Features/"

    # Extract features from the image
    def extractFeatures(self, img, img_name):
        # Load the ceiling image
        greyscale_image = cv2.imread(img, cv2.IMREAD_GRAYSCALE)

        brightness_threshold = 200  # Adjust this threshold as needed
        mask = greyscale_image > brightness_threshold

        # Cap the brightness of the brightest pixels
        greyscale_image[mask] = brightness_threshold

        normalised_image = cv2.normalize(greyscale_image, None, 0, 255, cv2.NORM_MINMAX)

        # Detect keypoints and compute descriptors
        keypoints, descriptors = self.algorithm.detectAndCompute(normalised_image, None)

        # Draw keypoints on the original image
        image_with_keypoints = cv2.drawKeypoints(greyscale_image, keypoints, None)

        # Save the image with detected keypoints
        img_path = img_name+"_with_keypoints.jpg"
        print(img_path)
        cv2.imwrite(img_path, image_with_keypoints)

        # Display the resulting image with detected keypoints
        cv2.imshow("Image with Keypoints", image_with_keypoints)
        cv2.waitKey(2)
        cv2.destroyAllWindows()

        # Print the number of detected keypoints
        print("Number of keypoints detected:", len(keypoints))

        return keypoints, descriptors
    

    def enhance_contrast_and_whiten(image, alpha=1.5, beta=25):
        """
        Enhance contrast and whiten an image.

        Parameters:
        - alpha: float, contrast control (default is 1.5).
        - beta: int, brightness control (default is 25).
        """

        # Normalize pixel values to [0, 255]
        normalized_image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)

        # Save the output image
        return normalized_image

# A module to take camera input from the robot and process it to extract the features from the image

import cv2
import numpy as np
from camera import Camera
from collections import deque

# Feature Extractor class to extract features from images taken by the camera


class FeatureExtractor:
    def __init__(self, camera=None, track=None):
        self.camera = camera
        self.track = track
        self.algorithm = cv2.SIFT_create()
        self.featureQueue = deque(maxlen=20)

    # Extract features from the image
    def extractFeatures(self, img):
        # Load the ceiling image
        ceiling_image = cv2.imread(img, cv2.IMREAD_GRAYSCALE)

        # Initialize the SIFT detector

        # Detect keypoints and compute descriptors
        keypoints, descriptors = self.algorithm.detectAndCompute(ceiling_image, None)

        # Draw keypoints on the original image
        ceiling_image_with_keypoints = cv2.drawKeypoints(ceiling_image, keypoints, None)

        # Save the image with detected keypoints
        cv2.imwrite("ceiling_image_with_keypoints.jpg", ceiling_image_with_keypoints)

        # Display the resulting image with detected keypoints
        cv2.imshow("Ceiling Image with Keypoints", ceiling_image_with_keypoints)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Print the number of detected keypoints
        print("Number of keypoints detected:", len(keypoints))

        return keypoints, descriptors

# Get image from raspberry pi camera

# Import robot camera
from picamera import PiCamera
from collections import deque
import numpy as np


# A Class to represent the camera


class Camera:
    def __init__(self, camera=None, track=None):
        self.images = deque(maxlen=20)

    def get_image(self, i):
        return self.images[i]

    def capture_image(self):
        # Create a new PiCamera object
        camera = PiCamera()

        # Set the resolution of the camera
        camera.resolution = (4056, 3040)

        # Capture an image
        image = np.empty((4056, 3040, 3), dtype=np.uint8)
        camera.capture(image, "rgb")

        # Clean up the camera resources
        camera.close()

        # Return the captured image as a numpy array
        self.images.appendleft(image)
        return image

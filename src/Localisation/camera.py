# Get image from raspberry pi camera

# Import robot camera
from picamera import PiCamera
from collections import deque
from time import sleep
import numpy as np


# A Class to represent the camera


class Camera:
    def __init__(self, camera=None, track=None):
        self.images = deque(maxlen=20)

    def get_image(self, i):
        return self.images[i]

    def capture_image(self, output_dir):
        # Create a new PiCamera object
        camera = PiCamera()

        # Set the resolution of the camera
        camera.resolution = (1920, 1080)

        # Capture an image
        image = np.empty((1920, 1080, 3), dtype=np.uint8)
        camera.capture(output_dir)

        # Clean up the camera resources
        camera.close()

        # Return the captured image as a numpy array
        self.images.appendleft(image)
        return image
    
    def save_image(self, image, name):
        image.save(name)


camera = Camera()
sleep(2)
for i in range(10):
    camera.capture_image("photo"+str(i)+".jpg")
    sleep(0.1)
    

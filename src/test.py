import subprocess
import time
from config import BROADCAST_PATH



def test_video_stream():
    """
    Test the video stream by starting the listener as a subprocess, 
    ssh into the pi and start the broadcaster to recieve the images.
    """
    # Start the listener
    listener = subprocess.Popen(
        ["python3", "src/Utils/listener.py"], stdout=subprocess.PIPE
    )
    # Wait for the listener to start
    time.sleep(1)

    # SSH into the pi and start the broadcaster
    ssh = subprocess.Popen(
        [
            "ssh",
            "pi@gomamon",
            "python3 "+ BROADCAST_PATH,
        ])
    
def test_feature_extraction():
    """
    Test the feature extraction by taking an image from and extracting the features
    """
    
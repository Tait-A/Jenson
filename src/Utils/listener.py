import cv2
import numpy as np
import socket
import struct
import sys
sys.path.insert(1,"/afs/inf.ed.ac.uk/user/s20/s2051131/Jenson/src")
from config import HOST_IPV4


class VideoServer:
    def __init__(
        self,
        host,
        port,
    ):
        self.host = host
        self.port = port
        self.frames = []

    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()

        print("Waiting for a client to connect to "+str(HOST_IPV4)+"...")
        self.client_socket, self.client_address = self.server_socket.accept()
        print("Client connected:", self.client_address)

        try:
            self.receive_frames()
        finally:
            self.client_socket.close()
            self.server_socket.close()

    def receive_frames(self):
        connection = self.client_socket.makefile("rb")
        try:
            frame_count = 0
            while True:
                # Read the size of the incoming frame
                size = struct.unpack("<L", connection.read(struct.calcsize("<L")))[0]
                if not size:
                    break

                # Read the frame data and do something with it
                frame_data = connection.read(size)
                self.process_frame(
                    frame_data, frame_count
                )  # Define this function to process the frame
                frame_count += 1

        finally:
            connection.close()

    def process_frame(self, frame_data, frame_count):
        try:
            # Convert the frame data to a NumPy array
            frame = np.frombuffer(frame_data, dtype=np.uint8)

            # Decode the frame as a JPEG image
            image = cv2.imdecode(frame, cv2.IMREAD_COLOR)

            cv2.imshow("Bananas", image)
            # Ensure the image size is 1080p (1920x1080)
            if image.shape[0] == 1080 and image.shape[1] == 1920:
                # Process the 1080p BGR image here
                self.frames.append(image)
                cv2.imwrite("Output/output_image" + str(frame_count) + ".jpg", image)
            else:
                print("Received frame is not 1080p.")
        except Exception as e:
            print("Error processing frame:", str(e))


if __name__ == "__main__":
    server = VideoServer(HOST_IPV4, 65024)  # Customize the IP address and port
    server.start()

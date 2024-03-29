import os
import socket

hostname = socket.gethostname()
IPAddr = socket.gethostbyname(hostname)

RPI_IPV4 = "192.168.105.135"
HOST_IPV4 = IPAddr
MAC_IPV4 = "192.168.105.79"
BROADCAST_PATH = "/Jenson/src/Utils/broadcaster.py"
SRC_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__)))

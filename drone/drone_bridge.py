import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from custom_interfaces.msg import DroneState
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

import socket
import time
import cv2
import threading
import re

hostIP = '0.0.0.0'
hostPort = 9000
hostAddress = (hostIP, hostPort)

telloIP = '192.168.10.1'
controlPort = 8889
telloAddress = (telloIP, controlPort)

statePort = 8890
videoPort = 11111


class DroneBridge(Node):
    def __init__(self):
        super().__init__('drone_bridge')

        #TODO: Define service to request for commands and send it to the drone

        self.UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.UDPServerSocket.bind(hostAddress)

        self.sendCommand('command')
        time.sleep(1)
        self.sendCommand('streamon')
        time.sleep(1)
        
        # Setup UDP State server
        self.UDPStateSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.UDPStateSocket.bind(('', statePort))
        print("Starting state thread")
        self.state_publish_thread = threading.Thread(target=self._state_thread_func)
        self.state_publish_thread.start()
        print("State thread started")


        # Setup to receive camera feed from drone
        self._running = True
        self.video = cv2.VideoCapture("udp://@0.0.0.0:11111")
        print("Starting video thread")
        self.video_publish_thread = threading.Thread(target=self.video_thread_func)
        self.video_publish_thread.start()
        print("Video thread started")

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        
    def sendCommand(self, command):
        """
        Send commands to the drone over the UDP Socket (IP: 192.168.10.1, Port: 8889)
        """
        try:
            encodedCommand = command.encode(encoding='utf-8')
            bytesSent = self.UDPServerSocket.sendto(encodedCommand, telloAddress)
            print("Command - {} with {} bytes sent".format(command, bytesSent))
            return bytesSent
        except Exception as e:
            return -1


    def _state_thread_func(self):
        """ Handler for Tello state message """
        self.state_publisher = self.create_publisher(DroneState, '/drone/state', 10)
        print("STATE THREAD")
        while True:
            print("INSIDE WHILE")
            try:
                self.data, self.address = self.UDPStateSocket.recvfrom(1024)
                print("New state info received:")
                print(self.data)

                # TODO: Extract state data here
                # Publish the extracted state data
                # self.state_publisher.publish(stateMessage)

            except Exception as e:
                print(e)

            # Time intervals between state messages received    
            time.sleep(1)


    def video_thread_func(self):
        """ Handler for Tello video message """
        self.video_publisher = self.create_publisher(Image, '/drone/video', 10)

        while self._running:
            try:
                ret, frame = self.video.read()
                if ret:
                    # Resize frame
                    height, width, _ = frame.shape
                    new_h = int(height/2)
                    new_w = int(width/2)

                    # Resize for improved performance
                    new_frame = cv2.resize(frame, (new_w, new_h))

                    # Display the resulting frame
                    cv2.imshow('Video', new_frame)

                    # TODO: Publish video frame
                    # self.video_publisher.publish(ImageMessage)
                    self.video_publisher.publish(self.br.cv2_to_imgmsg(new_frame))

                # Wait for display image frame
                cv2.waitKey(1)

            except Exception as err:
                print(err)    


def main(args=None):
    rclpy.init(args=args)

    drone_bridge = DroneBridge()
    rclpy.spin(drone_bridge)

    drone_bridge.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

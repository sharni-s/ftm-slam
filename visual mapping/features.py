import socket
import threading
import time
import av
import cv2
import numpy as np

print(cv2.__version__)

hostIP = ''
hostPort = 9000
hostAddress = (hostIP, hostPort)

telloIP = '192.168.10.1'
controlPort = 8889
telloAddress = (telloIP, controlPort)

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.bind(hostAddress)




def sendCommand(command):
    try:
        encodedCommand = command.encode(encoding='utf-8')
        bytesSent = UDPServerSocket.sendto(encodedCommand, telloAddress)
        print("Command - {} with {} bytes sent".format(command, bytesSent))
        return bytesSent
    except Exception as e:
        return -1

def _state_thread():
    UDPStateSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    statePort = 8890
    UDPStateSocket.bind(('', statePort))
    while True:
        try:
            data, address = UDPStateSocket.recvfrom(1024)
            print("New state info received:")
            print(data)
            print(address)
        except Exception as e:
            print(e)
        time.sleep(1)


class StateTello:
    def __init__(self):
        self.UDPStateSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.statePort = 8890
        self.UDPStateSocket.bind(('', self.statePort))

    def terminate(self):
        self.UDPStateSocket.close()

    def recvState(self):
        """ Handler for Tello state message """
        while True:
            try:
                self.data, self.address = self.UDPStateSocket.recvfrom(1024)
                print("New state info received:")
                print(self.data)
                # TODO: Extract state data here

            except Exception as e:
                print(e)

            # Time intervals between state messages received    
            time.sleep(1)


class VideoTello:
    def __init__(self):
        self._running = True
        self.video = cv2.VideoCapture("udp://@0.0.0.0:11111")

        # Initialize the ORB detector
        self.orb = cv2.ORB_create()
        # Create a Brute Force Matcher object
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Initialize the feature vector
        self.curr_features = []
        self.new_features = []

    def terminate(self):
        self._running = False
        self.video.release()
        cv2.destroyAllWindows()

    def recvVideo(self):
        """ Handler for Tello video message """
        x=1
        orb = cv2.ORB_create()
        # Create empty lists for keypoints and descriptors
        prev_kp = []
        prev_des = []

        while self._running:
            try:
                # Read a frame from the video
                ret, frame = self.video.read()

                if ret:
                    # Resize frame
                    height, width, _ = frame.shape
                    new_h = int(height/2)
                    new_w = int(width/2)

                    # Resize for improved performance
                    new_frame = cv2.resize(frame, (new_w, new_h))

                    # Convert frame to grayscale
                    gray = cv2.cvtColor(new_frame, cv2.COLOR_BGR2GRAY)

                    # Detect keypoints and descriptors in the current frame
                    kp, des = orb.detectAndCompute(gray, None)

                    # Create empty images for new keypoints
                    new_kp_img = np.zeros_like(gray)

                    # Create a Brute Force Matcher object
                    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

                    if prev_kp and prev_des:
                        # Match descriptors with the previous frame
                        matches = bf.match(des, prev_des)

                        # Sort matches by distance
                        matches = sorted(matches, key=lambda x: x.distance)

                        # Loop over matches and draw them in green if they are new
                        for match in matches:
                            if match.queryIdx not in prev_kp:
                                new_kp_img = cv2.drawKeypoints(gray, kp[match.queryIdx], new_kp_img, color=(0, 255, 0), flags=0)

                        # Draw matched keypoints in red
                        matched_img = cv2.drawMatches(gray, kp, prev_gray, prev_kp, matches[:50], None, flags=2)

                        # Show images
                        cv2.imshow('matched features', matched_img)
                        cv2.imshow('new features', new_kp_img)
                    else:
                        # Draw all keypoints in red if it is the first frame
                        new_kp_img = cv2.drawKeypoints(gray, kp, new_kp_img, color=(0, 0, 255), flags=0)
                        cv2.imshow('new features', new_kp_img)

                    # Update previous frame keypoints and descriptors
                    if prev_kp:
                        prev_kp = [match.queryIdx for match in matches]
                    else:
                        prev_kp = kp
                    prev_des = des
                    prev_gray = gray

                cv2.waitKey(1)

            # # Exit on 'q' key press
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

            except Exception as e:
                print(e)



        # while self._running:
        #     try:
        #         ret, frame = self.video.read()
        #         if ret:
        #             # Resize frame
        #             height, width, _ = frame.shape
        #             new_h = int(height/2)
        #             new_w = int(width/2)

        #             # Resize for improved performance
        #             new_frame = cv2.resize(frame, (new_w, new_h))

        #             # Convert the frame to grayscale
        #             gray = cv2.cvtColor(new_frame, cv2.COLOR_BGR2GRAY)
        #             # Detect keypoints and compute descriptors
        #             keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        #             # Store the keypoints and descriptors in the feature vector
        #             self.curr_features.append((keypoints, descriptors))
        #             # Display the detected keypoints
        #             img = cv2.drawKeypoints(gray, keypoints, None, color=(0, 255, 0), flags=0)

        #             # Display the resulting frame
        #             cv2.imshow('Video', img)
        #             if x  == 1:
        #                 print(keypoints)
        #                 x=0

        #         # Wait for display image frame
        #         # cv2.waitKey(1)
        #         if cv2.waitKey(1) & 0xFF == ord('q'):
        #             break

        #     except Exception as err:
        #         print(err)




def main(args=None):
    sendCommand('command')
    time.sleep(1)
    sendCommand('streamon')
    time.sleep(1)

    # state_thread = threading.Thread(target=_state_thread)
    # state_thread.daemon = True
    # state_thread.start()

    # print("Starting state thread")
    # telloState = StateTello()
    # state_thread = threading.Thread(target=telloState.recvState)
    # state_thread.start()
    # print("State thread started")

    print("Starting video thread")
    telloVideo = VideoTello()
    video_thread = threading.Thread(target=telloVideo.recvVideo)
    video_thread.start()
    print("Video thread started")

    # video_thread = threading.Thread(target=_video_thread)
    # video_thread.daemon = True
    # video_thread.start()

    # video = VIDEO()

    # sendCommand('takeoff')
    # time.sleep(7.5)
    # sendCommand('forward 20')
    # time.sleep(7.5)
    # sendCommand('cw 180')
    # time.sleep(7.5)
    # sendCommand('land')
    time.sleep(5)

main()

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_interfaces.action import TrialAction
from std_srvs.srv import Empty
from action_interfaces.srv import Reward
import sys
import time
import serial
import numpy as np
import threading
import queue
import cv2

from numpy import unravel_index


class TrialActionActionServer(Node):

    def __init__(self, queue):
        super().__init__('TrialAction_reward_server')
        self.srv = self.create_service(Empty, 'reset_reward', self.reset_rew_callback)
        self.srv = self.create_service(Reward, 'get_reward', self.get_rew_callback)
        self.srv = self.create_service(Reward, 'get_progress', self.get_prog_callback)
        self.q = queue
        self.set_state()
        time.sleep(2)
        self.set_state()
        print("Setup")
        self.image = None
        self.org_state = None
        # self.last_state - None

    def reset_rew_callback(self, request, response):
        self.set_state()
        return response

    def get_rew_callback(self, request, response):
        rew = self.get_reward()
        if abs(rew) < 6:
            rew = -0.5
        elif rew < 0:
            rew *= 2

        response.reward = rew * 0.2
        response.done = False
        if np.linalg.norm(self.org_state-self.state) > 140:
            rew += 30
            response.done = True
        # self.set_state()
        return response

    def get_prog_callback(self, request, response):
        rew = self.get_progress()
        response.reward = rew
        # self.set_state()
        return response

    def d_q_image(self):
        img = None
        q = self.q
        while not q.empty():
            img = q.get()
        time.sleep(0.2)
        print("newinQ", q.qsize())
        while not q.empty():
            img = q.get()
        return img

    def find_reddest_spot(self, image):
        imTP = image[:, :, 2] - image[:, :, 1]*0.3 - image[:, :, 0]*0.3
        newIm = cv2.GaussianBlur(imTP, (3, 3), cv2.BORDER_DEFAULT)
        maxP = unravel_index(np.argmax(newIm), newIm.shape)
        # newIm[maxP[1], maxP[0]] = 256
        imageTS = cv2.circle(newIm, (maxP[1], maxP[0]), radius=4, color=(255, 0, 255), thickness=2)
        self.image = imageTS
        # cv2.imwrite("images/last_image.png", newIm)
        return maxP

    def set_state(self):
        print("RESET!")
        image = self.d_q_image()  # get_image(camera)
        if image is None:
            print("Error, no image available!")
            return
        self.state = np.array(self.find_reddest_spot(image))
        self.org_state = self.state
        imageTS = cv2.putText(self.image, "reset", (50, 50),
                              cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imwrite("/workspaces/ros2_example/imlog/images/last_image_org_%f.png" %
                    (time.time()), imageTS)

    def get_reward(self):
        image = self.d_q_image()  # get_image(camera)
        if image is None:
            print("Error, no image available!")
            return 0
        newstate = np.array(self.find_reddest_spot(image))
        rew = np.linalg.norm(self.state-newstate)
        print(self.state, newstate)
        if self.state[1] < newstate[1]:
            rew *= -1
        imageTS = cv2.putText(self.image, str(round(rew, 2)), (50, 50),
                              cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imwrite("/workspaces/ros2_example/imlog/images/last_image_org_%f.png" %
                    (time.time()), imageTS)
        self.state = newstate
        return rew

    def get_progress(self):
        image = self.d_q_image()  # get_image(camera)
        if image is None:
            print("Error, no image available!")
            return 0
        newstate = np.array(self.find_reddest_spot(image))
        rew = np.linalg.norm(self.org_state-newstate)
        print(self.org_state, newstate)
        if self.org_state[1] < newstate[1]:
            rew *= -1
        imageTS = cv2.putText(self.image, str(round(rew, 2))+"PROG", (50, 50),
                              cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imwrite("/workspaces/ros2_example/imlog/images/last_image_org_%f_PROG.png" %
                    (time.time()), imageTS)
        return rew


class ImageQueuer(threading.Thread):
    def __init__(self, threadID, name, counter, camera, queue):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
        self.camera = camera
        self.done = False
        self.q = queue

    def run(self):
        print("Starting " + self.name)
        i = 0
        if not self.camera.isOpened():
            print("Error, cam not open")
            return
        while i < self.counter and not self.done:
            if self.camera.isOpened():
                (self.status, self.frame) = self.camera.read()
                time.sleep(0.01)
            self.q.put(self.frame)
            print("p%d" % i, end="")
            i += 1
        print("Exiting " + self.name)


def main(args=None):
    print(sys.executable)
    print(sys.version)
    rclpy.init(args=args)
    q = queue.Queue(maxsize=10)
    camera = cv2.VideoCapture(4, apiPreference=cv2.CAP_V4L2)
    camera.set(3, 640)
    camera.set(4, 480)
    camera.set(10, -60)  # brightness     min: 0   , max: 255 , increment:1
    camera.set(11, 10)  # contrast       min: 0   , max: 255 , increment:1
    camera.set(12, 25)  # saturation     min: 0   , max: 255 , increment:1
    camera.set(17, 10)
    retval = camera.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    print("success?", retval)
    thread1 = ImageQueuer(1, "Image Queuer", 2000000, camera, q)
    thread1.setDaemon(True)
    thread1.start()
    time.sleep(2)

    fibonacci_action_server = TrialActionActionServer(q)

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()

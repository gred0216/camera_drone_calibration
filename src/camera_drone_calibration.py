#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation
import ruamel.yaml


from single_rotation_averaging import ChordalL1Mean


class myNode(object):
    def __init__(self, camera_image_topic, vicon_odom_topic, camera_intrinsic_matrix, save_path, H_c2b=None):
        # Params
        self.image = None
        self.loop_rate = rospy.Rate(200)  # Node cycle rate (in Hz).
        self.bridge = CvBridge()
        self.final_optimized = False
        self.save_path = save_path

        np.set_printoptions(precision=3, suppress=True)

        # latest odometry
        self.orientation = Rotation.from_matrix(np.eye(3))
        self.position = [0, 0, 0]

        # Publishers
        self.image_pub = rospy.Publisher(
            'camera/color/image_raw_feature', Image, queue_size=10)

        self.odom_pub = rospy.Publisher(
            '/calibrate_odom', Odometry, queue_size=10)

        self.pub_camera_pose = rospy.Publisher(
            '/camera/body/pose', PoseStamped, queue_size=10)

        # Subscribers
        # self.sub = rospy.Subscriber(
        #     camera_image_topic, Image, self.img_callback)

        # self.odom_sub = rospy.Subscriber(
        #     vicon_odom_topic, Odometry, self.odom_callback)

        # Sync subsribers
        self.image_sub = Subscriber(
            camera_image_topic, Image)
        self.odom_sub = Subscriber(
            vicon_odom_topic, Odometry)

        self.ats = ApproximateTimeSynchronizer(
            [self.image_sub, self.odom_sub], queue_size=5, slop=0.1)
        self.ats.registerCallback(self.odom_img_callback)

        # Timer: 5 second timer will be start when received the first image
        self.timer = None

        # Calibration variable
        # chessboard points in the world (10 x 7 @ 3.6cm chessboard)
        width = 3.6/100  # meter
        height = 0.8/100
        objp = np.zeros((10*7, 3), np.float32)
        objp[:, :2] = np.indices((7, 10)).transpose((1, 2, 0)).reshape(70, 2)
        objp *= width
        # objp[:, 0:2] += width

        objp[:, 1] -= 4.5 * width
        objp[:, 0] -= 3 * width
        objp = objp[::-1]
        objp[:, 2] += height
        self.objp = objp
        # print(self.objp)

        # pre-calibrated camera intrinsic matrix
        self.cameraMatrix = np.array(camera_intrinsic_matrix)

        self.criteria = (cv.TERM_CRITERIA_EPS +
                         cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        self.all_c2b_rotation = []
        self.all_c2b_translation = []
        self.odom_msg = None

        self.h_c2b_calibrated = None
        if H_c2b is not None:
            # self.h_c2b_calibrated = H_c2b
            pass

    def start_timer(self):
        self.timer = rospy.Timer(rospy.Duration(5), self.timer_callback)

    def reset_timer(self):
        '''
        reset timer and clean cached rotation and translation
        '''
        if self.timer is None:
            return
        self.timer.shutdown()
        self.timer = rospy.Timer(rospy.Duration(5), self.timer_callback)
        self.all_c2b_rotation = []
        self.all_c2b_translation = []

    def calculate_pose(self, img):
        '''
        Calculate current timestamp pose of Hc2b, append to the cached rotation and translation

        Input: current frame image
        '''

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret, corners = cv.findChessboardCorners(gray, (10, 7), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            corners2 = cv.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), self.criteria)

            # ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
            #     [self.objp], [corners2], gray.shape[::-1], self.cameraMatrix, None)

            retval, rvecs, tvecs = cv.solvePnP(
                self.objp, corners2, self.cameraMatrix, None)

            # check points align with world coordinates
            # imgpts, jac = cv.projectPoints(
            #     self.objp, rvecs, tvecs, self.cameraMatrix, None)
            # pt0 = np.int0(imgpts[0][0])
            # pt19 = np.int0(imgpts[19][0])
            # # print(pts)
            # img = cv.circle(img, pt0, radius=3,
            #                 color=(0, 165, 255), thickness=-1)
            # img = cv.circle(img, pt19, radius=3,
            #                 color=(0, 165, 255), thickness=-1)

            rotation = Rotation.from_rotvec(
                rvecs.transpose())  # world to camera
            H_w2c = np.zeros((4, 4))
            H_w2c[0:3, 0:3] = rotation.as_matrix()
            H_w2c[0:3, 3:] = tvecs
            H_w2c[3, 3] = 1

            H_c2w = np.linalg.inv(H_w2c)

            # body to word is from vicon odometry
            H_b2w = np.zeros((4, 4))
            H_b2w[0:3, 0:3] = self.orientation.as_matrix()
            H_b2w[0:3, 3] = self.position
            H_b2w[3, 3] = 1

            H_w2b = np.linalg.inv(H_b2w)

            H_b2c = H_w2c @ H_b2w

            H_c2b = np.linalg.inv(H_b2w) @ H_c2w

            self.all_c2b_rotation.append(H_c2b[0:3, 0:3])
            self.all_c2b_translation.append(H_c2b[0:3, 3])

        self.image = img

    def plot_projected_point(self, img):
        '''
        For validating Hc2b, project chessboard points from world to camera using calculated Hc2b
        '''
        if self.h_c2b_calibrated is None:
            return

        # body to word is from vicon odometry
        H_b2w = np.zeros((4, 4))
        H_b2w[0:3, 0:3] = self.orientation.as_matrix()
        H_b2w[0:3, 3] = self.position
        H_b2w[3, 3] = 1

        H_w2b = np.linalg.inv(H_b2w)

        # make chessboard points into 4x70 homogeneous coordinates [x, y, z, 1]
        objp_homo = np.append(
            self.objp, np.ones([len(self.objp), 1]), 1)

        H_b2c_final = np.linalg.inv(self.h_c2b_calibrated)

        # P_c = H_b2c * H_w2b * P_w
        objp_camera = H_b2c_final @ H_w2b @ objp_homo.transpose()
        objp_camera = objp_camera[0:3, :]  # (3, 70)
        objp_pixel = self.cameraMatrix @ objp_camera
        objp_pixel = objp_pixel.transpose()

        for pt in objp_pixel:
            # normalize each point to [u, v, 1]
            pt = pt / pt[2]
            pt = np.int0(pt[0:2])
            img = cv.circle(img, pt, radius=3,
                            color=(0, 0, 255), thickness=-1)

        self.image = img

    def odom_img_callback(self, image, odom):
        '''
        Receive image and odom message at the same time
        '''

        self.odom_callback(odom)
        self.img_callback(image)

    def img_callback(self, image_data):
        '''
        Subscribe camera image, find transformation of camera to body
        If calibrated Hc2b is provided, use Hc2b to reproject chessboard points to the image for validation
        '''

        if self.timer is None:
            # Start counting 5 second for collecting data
            self.start_timer()

        img = np.frombuffer(image_data.data, dtype=np.uint8).reshape(
            image_data.height, image_data.width, -1)
        img = cv.cvtColor(img, cv.COLOR_BGR2RGB)

        if not self.final_optimized:
            self.calculate_pose(img)
        else:

            self.plot_projected_point(img)

        # self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image))

    def timer_callback(self, event):
        '''
        This function will be called when succefully collected 5 seconds of data, perform rotation averaging, save and publish result
        '''
        if self.final_optimized or len(self.all_c2b_rotation) < 120:
            return

        all_rotation = np.array(self.all_c2b_rotation)
        all_translation = np.array(self.all_c2b_translation)
        # Single rotation averaging
        R_final = ChordalL1Mean(all_rotation, True, 20, 0.001)
        T_final = np.mean(all_translation, axis=0)
        H_final = np.eye(4)
        H_final[0:3, 0:3] = R_final
        H_final[0:3, 3] = T_final

        # Publish camera to body pose

        c2b_pose = PoseStamped()
        c2b_pose.header.stamp = self.odom_msg.header.stamp
        rc2b = Rotation.from_matrix(R_final)
        qc2b = list(rc2b.as_quat())
        p = Point(H_final[0, 3], H_final[1, 3], H_final[2, 3])
        q = Quaternion(*qc2b)
        c2b_pose.pose = Pose(p, q)
        self.pub_camera_pose.publish(c2b_pose)

        self.final_optimized = True
        rospy.loginfo('Calibration completed')
        print("Final optimized H camera to body")
        print(H_final)
        print("Result is save in ", self.save_path)

        d = {"H_c2b": H_final.tolist()}
        yaml = ruamel.yaml.YAML()
        yaml.version = (1, 2)
        yaml.default_flow_style = None
        with open(self.save_path, 'w+') as outfile:
            yaml.dump(d, outfile)

        self.h_c2b_calibrated = H_final
        self.timer.shutdown()

    def odom_callback(self, odom):
        '''
        Subscribe vicon odometry, record latest pose
        '''
        # print('odom callback')
        # self.plot_projected_point(self.image)
        self.odom_msg = odom
        orientation = odom.pose.pose.orientation
        position = odom.pose.pose.position
        lin_vel = odom.twist.twist.linear

        # If there a spike in vicon odometry position, reset the timer
        # if (np.linalg.norm([position.x, position.y, position.z]) - np.linalg.norm(self.position)) > 0.1:

        # If the vicon odometry linear velocity norm > 0.1, reset the timer
        if not self.final_optimized and np.linalg.norm([lin_vel.x, lin_vel.y, lin_vel.z]) > 0.1:
            print('Detected spike in linear velocity, timer reset')
            self.reset_timer()

        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.orientation = Rotation.from_quat(quat)
        self.position = [position.x, position.y, position.z]

    def start(self):
        # rospy.spin()
        while not rospy.is_shutdown():
            if self.image is not None:
                # print("pub image")
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image))

            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("camera_drone_calibration", anonymous=True)

    camera_image_topic = rospy.get_param(
        "/camera_drone_calibration/camera_image")
    vicon_odom_topic = rospy.get_param("/camera_drone_calibration/vicon_odom")
    save_path = rospy.get_param("/camera_drone_calibration/save_path")
    camera_intrinsic_matrix = rospy.get_param(
        "/camera_intrinsic_matrix")
    camera_intrinsic_matrix = np.reshape(
        np.array(camera_intrinsic_matrix), (3, 3))

    my_node = myNode(camera_image_topic, vicon_odom_topic,
                     camera_intrinsic_matrix, save_path)
    my_node.start()

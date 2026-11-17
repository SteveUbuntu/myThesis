#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
import tf
import tf2_ros
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge, CvBridgeError

IMAGE_TOPIC = "/firefighter/camera1/image_raw"
CAMERA_INFO_TOPIC = "/firefighter/camera1/camera_info"
MODEL_STATES_TOPIC = "/gazebo/model_states"
CAMERA_LINK_FRAME = "camera_flange"
ROBOT_BASE_FRAME = "g_base"
ROBOT_MODEL_NAME = "cobot"
WORLD_PLANE_Z = 0.2026
CUBE_HEIGHT = 0.015
CUBE_TOP_Z = WORLD_PLANE_Z + CUBE_HEIGHT / 2.0
CAMERA_SENSOR_OFFSET = np.array([0.042, 0.02, 0.0])
CAMERA_SENSOR_YAW = 1.57
MIN_RED_AREA = 50
MIN_RED_VALUE = 100


class ObjectDetection(object):
    def __init__(self, camera_info):
        self.bridge_object = CvBridge()
        self.camera_info = camera_info
        self.tf_listener = tf.TransformListener()
        self.robot_base_pose = None
        self.model_states_sub = rospy.Subscriber(
            MODEL_STATES_TOPIC,
            ModelStates,
            self.model_states_callback,
            queue_size=1,
        )
        self.image_sub = rospy.Subscriber(
            IMAGE_TOPIC,
            Image,
            self.camera_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

    def model_states_callback(self, data):
        try:
            robot_index = data.name.index(ROBOT_MODEL_NAME)
        except ValueError:
            rospy.logwarn_throttle(
                2.0,
                "Robot model %s not found in Gazebo model states.",
                ROBOT_MODEL_NAME,
            )
            return

        pose = data.pose[robot_index]
        self.robot_base_pose = (
            np.array([pose.position.x, pose.position.y, pose.position.z]),
            np.array([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]),
        )

    def estimate_world_position(self, pixel, image_stamp):
        if self.robot_base_pose is None:
            rospy.logwarn_throttle(2.0, "Waiting for the robot pose from Gazebo.")
            return None

        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]
        if fx <= 0.0 or fy <= 0.0:
            rospy.logerr_throttle(2.0, "CameraInfo contains invalid focal lengths.")
            return None

        try:
            self.tf_listener.waitForTransform(
                ROBOT_BASE_FRAME,
                CAMERA_LINK_FRAME,
                image_stamp,
                rospy.Duration(0.5),
            )
            translation, quaternion = self.tf_listener.lookupTransform(
                ROBOT_BASE_FRAME,
                CAMERA_LINK_FRAME,
                image_stamp,
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
            tf2_ros.TransformException,
        ):
            try:
                translation, quaternion = self.tf_listener.lookupTransform(
                    ROBOT_BASE_FRAME,
                    CAMERA_LINK_FRAME,
                    rospy.Time(0),
                )
                rospy.logwarn_throttle(
                    2.0,
                    "No camera TF at image timestamp, using latest TF for position estimate.",
                )
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ) as error:
                rospy.logwarn_throttle(2.0, "Camera TF unavailable: %s", error)
                return None

        link_rotation = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
        sensor_rotation = tf.transformations.euler_matrix(
            0.0,
            0.0,
            CAMERA_SENSOR_YAW,
        )[:3, :3]
        camera_origin_in_base = (
            np.asarray(translation) + link_rotation @ CAMERA_SENSOR_OFFSET
        )
        camera_rotation_in_base = link_rotation @ sensor_rotation

        base_position, base_quaternion = self.robot_base_pose
        base_rotation = tf.transformations.quaternion_matrix(
            base_quaternion,
        )[:3, :3]
        camera_origin_in_world = base_position + base_rotation @ camera_origin_in_base
        camera_rotation_in_world = base_rotation @ camera_rotation_in_base

        pixel_x, pixel_y = pixel
        camera_ray = np.array([
            1.0,
            -(pixel_x - cx) / fx,
            -(pixel_y - cy) / fy,
        ])
        world_ray = camera_rotation_in_world @ camera_ray
        if abs(world_ray[2]) < 1e-6:
            return None

        # Red pixels lie on the top face; intersect there to avoid parallax,
        # then report the cube center height.
        distance = (CUBE_TOP_Z - camera_origin_in_world[2]) / world_ray[2]
        if distance <= 0.0:
            return None

        position = camera_origin_in_world + distance * world_ray
        position[2] = WORLD_PLANE_Z
        return position

    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logerr("Could not convert camera image: %s", error)
            return

        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        red_low_hue = cv.inRange(
            hsv,
            np.array([0, 70, MIN_RED_VALUE], dtype=np.uint8),
            np.array([14, 255, 255], dtype=np.uint8),
        )
        red_high_hue = cv.inRange(
            hsv,
            np.array([166, 70, MIN_RED_VALUE], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )
        red_mask = cv.bitwise_or(red_low_hue, red_high_hue)
        cv.imshow("red mask", red_mask)

        component_count, _, stats, centroids = cv.connectedComponentsWithStats(
            red_mask,
            connectivity=8,
        )
        detections = []
        for component_index in range(1, component_count):
            area = int(stats[component_index, cv.CC_STAT_AREA])
            if area < MIN_RED_AREA:
                continue

            x = int(stats[component_index, cv.CC_STAT_LEFT])
            y = int(stats[component_index, cv.CC_STAT_TOP])
            width = int(stats[component_index, cv.CC_STAT_WIDTH])
            height = int(stats[component_index, cv.CC_STAT_HEIGHT])
            center_x, center_y = centroids[component_index]
            detections.append((
                (x, y, width, height),
                (int(round(center_x)), int(round(center_y))),
            ))
        detections.sort(key=lambda detection: detection[1][0])

        annotated_img = cv_image.copy()
        positions = []

        for cube_index, (bbox, center) in enumerate(detections, start=1):
            x, y, width, height = bbox
            position = self.estimate_world_position(center, data.header.stamp)
            cv.rectangle(
                annotated_img,
                (x, y),
                (x + width, y + height),
                (0, 0, 255),
                2,
            )
            cv.circle(annotated_img, center, 2, (0, 255, 0), -1)
            if position is None:
                label = "cube %d: position unavailable" % cube_index
            else:
                label = "cube %d: (%.3f, %.3f, %.3f) m" % (
                    cube_index,
                    position[0],
                    position[1],
                    position[2],
                )
                positions.append((center, position))
            cv.putText(
                annotated_img,
                label,
                (5, 20 + (cube_index - 1) * 20),
                cv.FONT_HERSHEY_PLAIN,
                1,
                (0, 255, 0),
                1,
            )

        rospy.loginfo_throttle(
            2.0,
            "Detected %d red cube candidates; world positions at z=%.4f: %s",
            len(detections),
            WORLD_PLANE_Z,
            [
                "pixel=%s xyz=(%.4f, %.4f, %.4f)" % (
                    center,
                    position[0],
                    position[1],
                    position[2],
                )
                for center, position in positions
            ],
        )
        cv.imshow("edge detections", annotated_img)
        cv.waitKey(1)



if __name__ == '__main__':
    rospy.init_node('object_detection', anonymous=True)
    try:
        camera_info = rospy.wait_for_message(
            CAMERA_INFO_TOPIC,
            CameraInfo,
            timeout=10.0,
        )
    except rospy.ROSException as error:
        rospy.logfatal(
            "Could not read camera calibration from %s: %s",
            CAMERA_INFO_TOPIC,
            error,
        )
        raise SystemExit(1)

    object_detection = ObjectDetection(camera_info)
    rospy.loginfo(
        "Reading images from %s; reporting object positions in the Gazebo world frame.",
        IMAGE_TOPIC,
    )
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()
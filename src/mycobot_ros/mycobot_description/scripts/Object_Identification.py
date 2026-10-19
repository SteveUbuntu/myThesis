#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import tf
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image


IMAGE_TOPIC = '/firefighter/camera1/image_raw'
CAMERA_INFO_TOPIC = '/firefighter/camera1/camera_info'
ANNOTATED_IMAGE_TOPIC = '/object_identification/annotated'
CAMERA_LINK_FRAME = 'camera_flange'
OUTPUT_FRAME = 'g_base'
CAMERA_SENSOR_OFFSET = np.array([0.042, 0.02, 0.0])
CAMERA_SENSOR_YAW = 1.57
CUBE_CENTER_Z = 0.2026
MIN_RED_AREA = 50
MIN_TABLE_AREA = 1000


def find_color_regions(mask, minimum_area):
	component_count, _, stats, centroids = cv2.connectedComponentsWithStats(
		mask,
		connectivity=8,
	)
	regions = []
	for component_index in range(1, component_count):
		area = int(stats[component_index, cv2.CC_STAT_AREA])
		if area < minimum_area:
			continue

		x = int(stats[component_index, cv2.CC_STAT_LEFT])
		y = int(stats[component_index, cv2.CC_STAT_TOP])
		width = int(stats[component_index, cv2.CC_STAT_WIDTH])
		height = int(stats[component_index, cv2.CC_STAT_HEIGHT])
		center_x, center_y = centroids[component_index]
		regions.append({
			'area': area,
			'bbox': (x, y, width, height),
			'center': (int(round(center_x)), int(round(center_y))),
		})

	return sorted(regions, key=lambda region: region['center'][0])


class ObjectIdentifier:
	def __init__(self, camera_info):
		self.bridge = CvBridge()
		self.camera_info = camera_info
		self.tf_listener = tf.TransformListener()
		self.annotated_publisher = rospy.Publisher(
			ANNOTATED_IMAGE_TOPIC,
			Image,
			queue_size=1,
		)
		self.image_subscriber = rospy.Subscriber(
			IMAGE_TOPIC,
			Image,
			self.image_callback,
			queue_size=1,
			buff_size=2 ** 24,
		)

	def estimate_cube_position(self, pixel, image_stamp):
		fx = self.camera_info.K[0]
		fy = self.camera_info.K[4]
		cx = self.camera_info.K[2]
		cy = self.camera_info.K[5]
		if fx <= 0.0 or fy <= 0.0:
			rospy.logerr('CameraInfo contains invalid focal lengths.')
			return None

		try:
			self.tf_listener.waitForTransform(
				OUTPUT_FRAME,
				CAMERA_LINK_FRAME,
				image_stamp,
				rospy.Duration(0.5),
			)
			translation, quaternion = self.tf_listener.lookupTransform(
				OUTPUT_FRAME,
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
					OUTPUT_FRAME,
					CAMERA_LINK_FRAME,
					rospy.Time(0),
				)
				rospy.logwarn_throttle(
					2.0,
					'No camera TF at image timestamp. Using latest TF for XYZ estimate.',
				)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as error:
				rospy.logwarn_throttle(2.0, 'Camera TF unavailable: %s', error)
				return None

		link_rotation = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
		sensor_rotation = tf.transformations.euler_matrix(
			0.0,
			0.0,
			CAMERA_SENSOR_YAW,
		)[:3, :3]
		camera_origin = np.asarray(translation) + link_rotation @ CAMERA_SENSOR_OFFSET
		camera_rotation = link_rotation @ sensor_rotation

		pixel_x, pixel_y = pixel
		camera_ray = np.array([
			1.0,
			-(pixel_x - cx) / fx,
			-(pixel_y - cy) / fy,
		])
		world_ray = camera_rotation @ camera_ray
		if abs(world_ray[2]) < 1e-6:
			return None

		distance = (CUBE_CENTER_Z - camera_origin[2]) / world_ray[2]
		if distance <= 0.0:
			return None

		return camera_origin + distance * world_ray

	def image_callback(self, image_message):
		try:
			frame = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
		except CvBridgeError as error:
			rospy.logerr('Could not convert camera image: %s', error)
			return

		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		red_low_hue = cv2.inRange(
			hsv,
			np.array([0, 70, 35], dtype=np.uint8),
			np.array([14, 255, 255], dtype=np.uint8),
		)
		red_high_hue = cv2.inRange(
			hsv,
			np.array([166, 70, 35], dtype=np.uint8),
			np.array([179, 255, 255], dtype=np.uint8),
		)
		red_mask = cv2.bitwise_or(red_low_hue, red_high_hue)
		turquoise_mask = cv2.inRange(
			hsv,
			np.array([70, 45, 35], dtype=np.uint8),
			np.array([105, 255, 255], dtype=np.uint8),
		)

		red_cubes = find_color_regions(red_mask, MIN_RED_AREA)
		for region in red_cubes:
			region['xyz'] = self.estimate_cube_position(
				region['center'],
				image_message.header.stamp,
			)
		turquoise_regions = find_color_regions(turquoise_mask, MIN_TABLE_AREA)
		table = max(turquoise_regions, key=lambda region: region['area'], default=None)

		annotated = frame.copy()
		for cube_index, region in enumerate(red_cubes, start=1):
			x, y, width, height = region['bbox']
			cv2.rectangle(annotated, (x, y), (x + width, y + height), (0, 0, 255), 2)
			cv2.putText(
				annotated,
				'Red cube %d' % cube_index,
				(x, max(y - 6, 14)),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.45,
				(0, 0, 255),
				1,
				cv2.LINE_AA,
			)

		if table is not None:
			x, y, width, height = table['bbox']
			cv2.rectangle(annotated, (x, y), (x + width, y + height), (255, 180, 0), 2)
			cv2.putText(
				annotated,
				'Turquoise table',
				(x + 4, max(y + 16, 16)),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.55,
				(255, 180, 0),
				2,
				cv2.LINE_AA,
			)

		cube_report = ', '.join(
			'cube%d pixel=%s xyz=%s' % (
				index,
				region['center'],
				'(%0.3f, %0.3f, %0.3f)' % tuple(region['xyz'])
				if region['xyz'] is not None else 'unavailable',
			)
			for index, region in enumerate(red_cubes, start=1)
		) or 'no red cubes'
		table_report = 'table@%s' % (table['center'],) if table else 'no turquoise table'
		rospy.loginfo_throttle(
			2.0,
			'Vision: %d red cubes (%s); %s.',
			len(red_cubes),
			cube_report,
			table_report,
		)

		annotated_message = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
		annotated_message.header = image_message.header
		self.annotated_publisher.publish(annotated_message)


def main():
	rospy.init_node('object_identification', anonymous=True)
	try:
		camera_info = rospy.wait_for_message(CAMERA_INFO_TOPIC, CameraInfo, timeout=10.0)
	except rospy.ROSException as error:
		rospy.logfatal('Could not read camera calibration from %s: %s', CAMERA_INFO_TOPIC, error)
		return

	ObjectIdentifier(camera_info)
	rospy.loginfo('Reading camera images from %s.', IMAGE_TOPIC)
	rospy.loginfo('Publishing annotated detections on %s.', ANNOTATED_IMAGE_TOPIC)
	rospy.spin()


if __name__ == '__main__':
	main()
#!/usr/bin/env python
import os

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from stereo_camera.msg import RectifiedPair
from cv_bridge import CvBridge, CvBridgeError


def node_main():
    rospy.init_node('images_feeder', anonymous=True)
    params = rospy.get_param('~')
    full_calib_dir = params['full_calib_dir']
    sides = {
        'L': {
            'full_name': 'left',
            'cap': cv2.VideoCapture(params['left_cam_id']),
            'pub': rospy.Publisher('images_' + 'L', Image, queue_size=10),
            'undistortion_map': np.load(os.path.join(full_calib_dir, 'undistortion_map_left.npy')),
            'rectification_map': np.load(os.path.join(full_calib_dir, 'rectification_map_left.npy')),
        },
        'R': {
            'full_name': 'right',
            'cap': cv2.VideoCapture(params['right_cam_id']),
            'pub': rospy.Publisher('images_' + 'R', Image, queue_size=10),
            'undistortion_map': np.load(os.path.join(full_calib_dir, 'undistortion_map_right.npy')),
            'rectification_map': np.load(os.path.join(full_calib_dir, 'rectification_map_right.npy')),
        }
    }
    fps = params['fps']
    resolution = map(int, params['resolution'].split('x'))
    for side in sides.values():
        cap = side['cap']
        cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, resolution[0])
        cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, resolution[1])
        cap.set(cv2.cv.CV_CAP_PROP_FPS, fps)
    pair_pub = rospy.Publisher('rectified_pair', RectifiedPair, queue_size=10)
    assert os.path.isdir(full_calib_dir)
    bridge = CvBridge()
    rate = rospy.Rate(fps)
    while not rospy.is_shutdown():
        rectified_pair = {}
        for side in sides.values():
            res, img = side['cap'].read()
            assert res
            assert img is not None
            image_message = bridge.cv2_to_imgmsg(img, encoding='bgr8')
            side['pub'].publish(image_message)
            rectified = cv2.remap(img, side['undistortion_map'], side['rectification_map'], cv2.INTER_NEAREST)
            rectified_pair[side['full_name']] = bridge.cv2_to_imgmsg(rectified, encoding='bgr8')
        rectified_pair_msg = RectifiedPair()
        rectified_pair_msg.left = rectified_pair['left']
        rectified_pair_msg.right = rectified_pair['right']
        pair_pub.publish(rectified_pair_msg)
        rate.sleep()
    for side in sides.values():
        side['cap'].release()

if __name__ == '__main__':
    try:
        node_main()
    except rospy.ROSInterruptException:
        pass

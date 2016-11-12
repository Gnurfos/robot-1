#!/usr/bin/env python
from functools import partial
import os
import StringIO

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from stereo_camera.msg import RectifiedPair
from stereo_camera.srv import Calib, CalibResponse
from cv_bridge import CvBridge, CvBridgeError


def get_calib(full_calib_dir, request):
    # Maybe not portable but sufficient for now
    disp_to_depth_mat = np.load(os.path.join(full_calib_dir, 'disp_to_depth_mat.npy'))
    serialized = StringIO.StringIO()
    np.save(serialized, disp_to_depth_mat)
    return CalibResponse(
        disp_to_depth_mat=serialized.getvalue())


def node_main():
    rospy.init_node('images_feeder', anonymous=True)
    params = rospy.get_param('~')
    full_calib_dir = params['full_calib_dir']
    calib_service = rospy.Service('camera_calib', Calib, partial(get_calib, full_calib_dir))
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
            image_message = get_single_image_to_publish(img, bridge)
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
    calib_service.shutdown()


def get_single_image_to_publish(img, bridge):
    img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return bridge.cv2_to_imgmsg(img, encoding='rgb8')


if __name__ == '__main__':
    try:
        node_main()
    except rospy.ROSInterruptException:
        pass

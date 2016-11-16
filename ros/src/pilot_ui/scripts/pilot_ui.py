#!/usr/bin/env python
# coding=utf-8
from functools import partial

import sys
import rospy
import numpy as np
import StringIO
from stereo_camera.srv import Calib
from motors_controller.srv import SetSpeed, SetSpeedRequest
from motors_controller.msg import OneMotorSpeed
from sensor_msgs.msg import Image
from PyQt4 import QtGui
from PyQt4 import QtCore
from cv_bridge import CvBridge, CvBridgeError
import time


class Signaller(QtCore.QObject):
    my_signal = QtCore.pyqtSignal(QtGui.QImage)
signaller = Signaller()


class PilotUi(QtGui.QWidget):

    display_signal = QtCore.pyqtSignal(QtGui.QImage, QtCore.QString)

    def __init__(self):
        super(PilotUi, self).__init__()
        self.layout = QtGui.QVBoxLayout()

        self.but_qt = QtGui.QPushButton('Quit')
        self.but_qt.clicked.connect(self.quit)
        self.layout.addWidget(self.but_qt)

        grid = QtGui.QGridLayout()
        self.but_fw = QtGui.QPushButton(u'⇧')
        self.but_fw.pressed.connect(self.send_forward)
        self.but_fw.released.connect(self.send_stop)
        grid.addWidget(self.but_fw, 0, 1)
        self.but_bk = QtGui.QPushButton(u'⇩')
        self.but_bk.pressed.connect(self.send_back)
        self.but_bk.released.connect(self.send_stop)
        grid.addWidget(self.but_bk, 2, 1)
        self.but_lf = QtGui.QPushButton(u'⇦')
        self.but_lf.pressed.connect(self.send_left)
        self.but_lf.released.connect(self.send_stop)
        grid.addWidget(self.but_lf, 1, 0)
        self.but_rg = QtGui.QPushButton(u'⇨')
        self.but_rg.pressed.connect(self.send_right)
        self.but_rg.released.connect(self.send_stop)
        grid.addWidget(self.but_rg, 1, 2)
        self.but_st = QtGui.QPushButton(u'▋▋')
        self.but_st.clicked.connect(self.send_stop)
        grid.addWidget(self.but_st, 1, 1)
        self.layout.addLayout(grid)

        images_layout = QtGui.QHBoxLayout()
        images_layout.addStretch()
        self.img = {}
        for side in ['L', 'R']:
            self.img[side] = QtGui.QLabel()
            images_layout.addWidget(self.img[side])
            rospy.Subscriber('/images_' + side, Image, partial(self.receive_image, side))
        images_layout.addStretch()
        self.layout.addLayout(images_layout)
        self.display_signal.connect(self.display_image)

        self.but_calib = QtGui.QPushButton('Get calib')
        self.but_calib.clicked.connect(self.get_calib)
        self.layout.addWidget(self.but_calib)
        self.info_txt = QtGui.QTextEdit()
        self.info_txt.setReadOnly(True)
        self.layout.addWidget(self.info_txt)

        self.show()
        self.setLayout(self.layout)

    def send_stop(self):
        return self.send_motor_control(stop=True)

    command_duration_ms = 10000
    speed_percent = 80
    speed_percent_rotate = 70
    inverted_forward = True

    def send_right(self):
        return self.send_motor_control(
            right=OneMotorSpeed(direction_forward=False, speed_percent=self.speed_percent_rotate),
            left=OneMotorSpeed(direction_forward=True, speed_percent=self.speed_percent_rotate),
            duration_ms=self.command_duration_ms)

    def send_left(self):
        return self.send_motor_control(
            right=OneMotorSpeed(direction_forward=True, speed_percent=self.speed_percent_rotate),
            left=OneMotorSpeed(direction_forward=False, speed_percent=self.speed_percent_rotate),
            duration_ms=self.command_duration_ms)

    def send_forward(self):
        return self.send_motor_control(
            right=OneMotorSpeed(direction_forward=not self.inverted_forward, speed_percent=self.speed_percent),
            left=OneMotorSpeed(direction_forward=not self.inverted_forward, speed_percent=self.speed_percent),
            duration_ms=self.command_duration_ms)

    def send_back(self):
        return self.send_motor_control(
            right=OneMotorSpeed(direction_forward=self.inverted_forward, speed_percent=self.speed_percent),
            left=OneMotorSpeed(direction_forward=self.inverted_forward, speed_percent=self.speed_percent),
            duration_ms=self.command_duration_ms)

    def send_motor_control(self, **kwargs):
        try:
            s = rospy.ServiceProxy('set_speed', SetSpeed)
            req = SetSpeedRequest(**kwargs)
            ret = s(req)
            self.info_txt.setText(str(ret) + ' <= ' + str(req))
        except Exception as e:
            self.info_txt.setText(str(e))

    def receive_image(self, side, data):
        # runs in a rospy thread, not QT
        bridge = CvBridge()
        cv_img = bridge.imgmsg_to_cv2(data)
        height, width, byte_value = cv_img.shape
        byte_value = byte_value * width
        qt_img = QtGui.QImage(cv_img, width, height, byte_value, QtGui.QImage.Format_RGB888)
        self.display_signal.emit(qt_img, side)

    def display_image(self, qt_img, side):
        pix = QtGui.QPixmap(qt_img)
        self.img[str(side)].setPixmap(pix)

    def get_calib(self):
        try:
            s = rospy.ServiceProxy('camera_calib', Calib)
            val = s()
            val_as_file = StringIO.StringIO(val.disp_to_depth_mat)
            mat = np.load(val_as_file)
            self.info_txt.setText(str(mat))
        except Exception as e:
            self.info_txt.setText(str(e))

    def quit(self):
        self.close()


def node_main():
    rospy.init_node('pilot_ui', anonymous=True)
    # params = rospy.get_param('~')
    app = QtGui.QApplication(sys.argv)
    ui = PilotUi()
    sys.exit(app.exec_())


if __name__ == '__main__':
    try:
        node_main()
    except rospy.ROSInterruptException:
        pass

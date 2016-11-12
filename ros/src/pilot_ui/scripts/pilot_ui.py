#!/usr/bin/env python
from functools import partial

import sys
import rospy
import numpy as np
import StringIO
from stereo_camera.srv import Calib
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
        self.but_fw = QtGui.QPushButton('Forward')
        self.layout.addWidget(self.but_fw)
        self.but_st = QtGui.QPushButton('Stop')
        self.but_st.clicked.connect(self.send_stop)
        self.layout.addWidget(self.but_st)
        self.but_calib = QtGui.QPushButton('Get calib')
        self.but_calib.clicked.connect(self.get_calib)
        self.layout.addWidget(self.but_calib)
        self.calib_txt = QtGui.QTextEdit()
        self.calib_txt.setReadOnly(True)
        self.layout.addWidget(self.calib_txt)
        self.show()
        self.setLayout(self.layout)
        self.img = {}
        for side in ['L', 'R']:
            self.img[side] = QtGui.QLabel()
            self.layout.addWidget(self.img[side])
            rospy.Subscriber('/images_' + side, Image, partial(self.receive_image, side))
        self.display_signal.connect(self.display_image)

    def send_stop(self):
        pass

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
            self.calib_txt.setText(str(mat))
        except Exception as e:
            self.calib_txt.setText(str(e))

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

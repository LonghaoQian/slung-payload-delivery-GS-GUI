#!/usr/bin/env python

import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer, QThread

import ros_node as ros_node
import GUI as gui

if __name__ == "__main__":
    # init ros
    ros_node.rospy.init_node("ground_station_py")
    # define the window
    app = QtWidgets.QApplication(sys.argv)
    WaterSamplingGroundControlStation = QtWidgets.QTabWidget()
    ui = gui.Ui_WaterSamplingGroundControlStation()
    ui.setupUi(WaterSamplingGroundControlStation)

    # define the ros thread
    rosThread = ros_node.RosThread(ui)
    rosThread.start()

    # show the window
    WaterSamplingGroundControlStation.show()
    print("System Started")
    sys.exit(app.exec_())
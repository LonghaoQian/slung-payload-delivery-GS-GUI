#!/usr/bin/env python

import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer, QThread

import gui_thread as gui_thread
import gui as gui

if __name__ == "__main__":
    # init ros
    # ros_node.rospy.init_node("ground_station_py")
    
    # define the window
    app = QtWidgets.QApplication(sys.argv)
    gui_window = QtWidgets.QTabWidget()
    ui = gui.Ui_ground_station()
    ui.setupUi(gui_window)

    # define the ros thread
    # rosThread = ros_node.RosThread(ui)
    # rosThread.start()

    # show the window
    gui_window.show()
    print("ground station gui start...")
    sys.exit(app.exec_())
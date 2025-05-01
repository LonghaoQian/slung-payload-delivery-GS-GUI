#!/usr/bin/env python

"""
MIT License

Copyright (c) 2025 Flight Systems and Control Laboratory

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

"""

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
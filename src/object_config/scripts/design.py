# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'design.ui'
#
# Created: Sun Mar 25 21:38:52 2018
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui
import rospy
import cv2
import numpy as np
import copy

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

cap = cv2.VideoCapture(0)
minH = minS = minV = 0
maxH = maxS = maxV = 255

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(535, 393)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setObjectName(_fromUtf8("label"))
        self.horizontalLayout_2.addWidget(self.label)
        self.min_h = QtGui.QSlider(self.centralwidget)
        self.min_h.setOrientation(QtCore.Qt.Horizontal)
        self.min_h.setObjectName(_fromUtf8("min_h"))
        self.horizontalLayout_2.addWidget(self.min_h)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.horizontalLayout_3.addWidget(self.label_2)
        self.max_h = QtGui.QSlider(self.centralwidget)
        self.max_h.setOrientation(QtCore.Qt.Horizontal)
        self.max_h.setObjectName(_fromUtf8("max_h"))
        self.horizontalLayout_3.addWidget(self.max_h)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.label_3 = QtGui.QLabel(self.centralwidget)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.horizontalLayout_4.addWidget(self.label_3)
        self.min_s = QtGui.QSlider(self.centralwidget)
        self.min_s.setOrientation(QtCore.Qt.Horizontal)
        self.min_s.setObjectName(_fromUtf8("min_s"))
        self.horizontalLayout_4.addWidget(self.min_s)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_5 = QtGui.QHBoxLayout()
        self.horizontalLayout_5.setObjectName(_fromUtf8("horizontalLayout_5"))
        self.label_4 = QtGui.QLabel(self.centralwidget)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.horizontalLayout_5.addWidget(self.label_4)
        self.max_s = QtGui.QSlider(self.centralwidget)
        self.max_s.setOrientation(QtCore.Qt.Horizontal)
        self.max_s.setObjectName(_fromUtf8("max_s"))
        self.horizontalLayout_5.addWidget(self.max_s)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_6 = QtGui.QHBoxLayout()
        self.horizontalLayout_6.setObjectName(_fromUtf8("horizontalLayout_6"))
        self.label_5 = QtGui.QLabel(self.centralwidget)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.horizontalLayout_6.addWidget(self.label_5)
        self.min_v = QtGui.QSlider(self.centralwidget)
        self.min_v.setOrientation(QtCore.Qt.Horizontal)
        self.min_v.setObjectName(_fromUtf8("min_v"))
        self.horizontalLayout_6.addWidget(self.min_v)
        self.verticalLayout.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_7 = QtGui.QHBoxLayout()
        self.horizontalLayout_7.setObjectName(_fromUtf8("horizontalLayout_7"))
        self.label_6 = QtGui.QLabel(self.centralwidget)
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.horizontalLayout_7.addWidget(self.label_6)
        self.max_v = QtGui.QSlider(self.centralwidget)
        self.max_v.setOrientation(QtCore.Qt.Horizontal)
        self.max_v.setObjectName(_fromUtf8("max_v"))
        self.horizontalLayout_7.addWidget(self.max_v)
        self.verticalLayout.addLayout(self.horizontalLayout_7)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.btn1 = QtGui.QPushButton(self.centralwidget)
        self.btn1.setObjectName(_fromUtf8("btn1"))
        self.horizontalLayout.addWidget(self.btn1)
        self.btn2 = QtGui.QPushButton(self.centralwidget)
        self.btn2.setObjectName(_fromUtf8("btn2"))
        self.horizontalLayout.addWidget(self.btn2)
        self.btn3 = QtGui.QPushButton(self.centralwidget)
        self.btn3.setObjectName(_fromUtf8("btn3"))
        self.horizontalLayout.addWidget(self.btn3)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.edt = QtGui.QLineEdit(self.centralwidget)
        self.edt.setObjectName(_fromUtf8("edt"))
        self.verticalLayout.addWidget(self.edt)
        self.verticalLayout_2.addLayout(self.verticalLayout)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 535, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        rospy.init_node('object_config', anonymous=True)

        self.btn1.clicked.connect(self.getPicture)

        self.max_h.valueChanged.connect(self.valueHandlerMaxH)
        self.max_h.setRange(0,255)
        self.max_h.setValue(255)

        self.max_s.valueChanged.connect(self.valueHandlerMaxS)
        self.max_s.setRange(0,255)
        self.max_s.setValue(255)

        self.max_v.valueChanged.connect(self.valueHandlerMaxV)
        self.max_v.setRange(0,255)
        self.max_v.setValue(255)

        self.min_h.valueChanged.connect(self.valueHandlerMinH)
        self.min_h.setRange(0,255)

        self.min_s.valueChanged.connect(self.valueHandlerMinS)
        self.min_s.setRange(0,255)

        self.min_v.valueChanged.connect(self.valueHandlerMinV)
        self.min_v.setRange(0,255)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.label.setText(_translate("MainWindow", "min H", None))
        self.label_2.setText(_translate("MainWindow", "max H", None))
        self.label_3.setText(_translate("MainWindow", "min S", None))
        self.label_4.setText(_translate("MainWindow", "max S", None))
        self.label_5.setText(_translate("MainWindow", "min V", None))
        self.label_6.setText(_translate("MainWindow", "max V", None))
        self.btn1.setText(_translate("MainWindow", "button 1", None))
        self.btn2.setText(_translate("MainWindow", "button 2", None))
        self.btn3.setText(_translate("MainWindow", "button 3", None))

    def valueHandlerMaxH(self,value):
        global maxH
        maxH = value
        rospy.set_param('/vision/maxH', value)
        print maxH

    def valueHandlerMaxS(self,value):
        global maxS
        maxS = value
        rospy.set_param('/vision/maxS', value)
        print maxS

    def valueHandlerMaxV(self,value):
        global maxV
        maxV = value
        rospy.set_param('/vision/maxV', value)
        print maxV

    def valueHandlerMinH(self,value):
        global minH
        minH = value
        rospy.set_param('/vision/minH', value)
        print minH

    def valueHandlerMinS(self,value):
        global minS
        minS = value
        rospy.set_param('/vision/minS', value)
        print minS

    def valueHandlerMinV(self,value):
        global minV
        minV = value
        rospy.set_param('/vision/minV', value)
        print minV

    def findBiggestContour(self , contours):
        indexOfBiggest = -1
        currentIndex = 0
        sizeOfBiggest = 0
        for c in contours:
            if(len(c) > sizeOfBiggest):
                sizeOfBiggest = len(c)
                indexOfBiggest = currentIndex
            currentIndex += 1
        return indexOfBiggest

    def getPicture(self):
            global minH
            global minS
            global minV
            global maxH
            global maxS
            global maxV

        #try:
            global cap
            ret , frame = cap.read()
            ret , frame = cap.read()
            ret , frame = cap.read()
            ret , frame = cap.read()
            ret , frame = cap.read()
            ####################################   blur
            kernel = np.ones((3,3),np.float32)/9
            blur = cv2.filter2D(frame,-1,kernel)
            ####################################   hsv
            hsv = cv2.cvtColor(blur , cv2.COLOR_BGR2HSV)
            lower = np.array([minH,minS,minV])
            upper = np.array([maxH,maxS,maxV])
            mask = cv2.inRange(hsv,lower,upper)
            #################################### erode and dilate
            thresh = cv2.erode(mask,np.ones((1,1)))
            thresh = cv2.dilate(thresh, np.ones((5,5)))
            #######################################   find contours
            temp = copy.deepcopy(thresh)
            contours , hierarchy = cv2.findContours(temp,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            ########################################  biggest
            if len(contours) >0 :
                IndedOfBiggestContour = self.findBiggestContour(contours)
            ########################################
            #for cnt in contours:
                #if(cv2.contourArea(cnt) > 100 ) :

            rect = cv2.minAreaRect(contours[IndedOfBiggestContour])  # return top-left(x,y) , width-height , angle
            box = cv2.cv.BoxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(frame,[box],0,(0,0,255),1)

            cv2.imshow('raw', frame)
            cv2.imshow('thresh', thresh)

            objectWidth , objectHeight = rect[1][0] , rect[1][1]
            if(objectWidth > objectHeight):
                objectWidth , objectHeight = objectHeight , objectWidth

            self.edt.setText(str(minH)+' '+str(maxH)+' '+str(minS)+' '+str(maxS)+' '+str(minV)+' '+str(maxV)+' '+str(int(objectWidth))+' '+str(int(objectWidth))+' '+str(int(objectHeight))+' '+str(int(objectHeight))+' 1')

            print 'done'
        #except :
            #print("Unexpected error:", sys.exc_info()[0])
 

#!/usr/bin/env python
# Qtvcp camview
#
# Copyright (c) 2017  Chris Morley <chrisinnanaimo@hotmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# use open cv to do camera alignment

import sys
from PyQt4 import QtGui, QtCore

# Set up logging
from qtvcp import logger
log = logger.getLogger(__name__)

# If the library is missing don't crash the GUI
# send an error and just make a blank widget.

LIB_GOOD = True
try:
    import cv2
except:
    log.error('Qtvcp Error with camview - is python-opencv installed?')
    LIB_GOOD = False

from qtvcp.widgets.widget_baseclass import _HalWidgetBase
# This avoids segfault when testing directly in python
if __name__ != '__main__':
    from qtvcp.qt_glib import GStat
    GSTAT = GStat()

class CamView(QtGui.QWidget, _HalWidgetBase):
    def __init__(self, parent=None):
        super(CamView, self).__init__(parent)
        self.count = 0
        self.diameter = 20
        self.rotation = 0
        self.scale = 1
        self.gap = 5
        if LIB_GOOD:
            self.cap = cv2.VideoCapture(0)
        self.setWindowTitle('Cam View')
        self.setGeometry(100,100,200,200)
        self.text_color = QtGui.QColor(255,255,255)
        self.font = QtGui.QFont("arial,helvetica", 40)
        self.text = ''
        self.pix = None

    def _hal_init(self):
        if LIB_GOOD:
            GSTAT.connect('periodic', self.nextFrameSlot)

    ##################################
    # no button scroll = circle dismater
    # left button scroll = zoom
    # right button scroll = cross hair rotation
    ##################################
    def wheelEvent(self, event):
        super(CamView, self).wheelEvent(event)
        mouse_state=QtGui.qApp.mouseButtons()
        size = self.size()
        w = size.width()
        if event.delta() <0:
            if mouse_state==QtCore.Qt.NoButton:
                self.diameter -=1
            if mouse_state==QtCore.Qt.RightButton:
                self.rotation -=1
            if mouse_state==QtCore.Qt.LeftButton:
                self.scale -= .1
        else:
            if mouse_state==QtCore.Qt.NoButton:
                self.diameter +=1
            if mouse_state==QtCore.Qt.LeftButton:
                self.scale +=.1
            if mouse_state==QtCore.Qt.RightButton:
                self.rotation +=1
        if self.diameter < 2: self.diameter = 2
        if self.diameter > w: self.diameter = w
        if self.rotation >360: self.rotation = 0
        if self.rotation < 0: self.rotation = 360
        if self.scale < 1: self.scale = 1
        if self.scale > 5: self.scale = 5

    def nextFrameSlot(self,w):
        if not self.isVisible(): return
        # don't update at the full 100ms rate
        self.count +=1
        if self.count <30:return
        self.count = 0

        ############################
        # capture a freme from cam
        ############################
        ret, frame = self.cap.read()
        if not ret: return
        #print 'before',frame.shape
        (oh, ow) = frame.shape[:2]
        #print oh,ow
        #############################
        # scale image bigger
        #############################
        scale = self.scale
        #print scale
        frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation = cv2.INTER_CUBIC)

        ##########################
        # crop to the original size of the frame
        # measure from center so we zoom on center
        # ch = center of current height
        # coh = center of original height
        ##########################
        (h, w) = frame.shape[:2]
        ch = h/2
        cw = w/2
        coh = oh/2
        cow = ow/2
        # NOTE: its img[y: y + h, x: x + w]
        frame = frame[ ch-coh:ch+coh, cw-cow:cw+cow, ]

        ########################################
        # My webcam yields frames in BGR format
        # this may need other options for other cameras
        ########################################
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # fit to our 
        self.pix = QtGui.QImage(frame, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
        self.update()

    def paintEvent(self, event):
        qp = QtGui.QPainter()
        qp.begin(self)
        if self.pix:
            qp.drawImage(self.rect(), self.pix)
        self.drawText(event, qp)
        self.drawCircle(event, qp)
        self.drawCrossHair(event, qp)
        qp.end()

    def deleteLater(self):
        self.cap.release()
        super(QtGui.QWidget, self).deleteLater()

    def drawText(self, event, qp):
        size = self.size()
        w = size.width()
        h = size.height()
        qp.setPen(self.text_color)
        qp.setFont(self.font)
        if self.pix:
            text = self.text
        else:
            text = 'No Image'
        qp.drawText(self.rect(), QtCore.Qt.AlignCenter, text)

    def drawCircle(self, event, gp):
        size = self.size()
        w = size.width()
        h = size.height()
        radx = self.diameter/2
        rady = self.diameter/2
        # draw red circles
        gp.setPen(QtCore.Qt.red)
        center = QtCore.QPoint(w/2,h/2)
        gp.drawEllipse(center, radx, rady)

    def drawCrossHair(self, event, gp):
        size = self.size()
        w = size.width()/2
        h = size.height()/2
        pen = QtGui.QPen(QtCore.Qt.yellow, 1, QtCore.Qt.SolidLine)
        gp.setPen(pen)
        gp.translate(w,h)
        gp.rotate(self.rotation)
        gp.drawLine(-w, 0, 0-self.gap, 0)
        gp.drawLine(0+self.gap, 0, w, 0)
        gp.drawLine(0, 0+self.gap, 0, h)
        gp.drawLine(0, 0-self.gap, 0, -h)

if __name__ == '__main__':

    import sys
    app = QtGui.QApplication(sys.argv)
    capture = CamView()
    capture.show()
    def jump():
        capture.nextFrameSlot(None)
    timer = QtCore.QTimer()
    timer.timeout.connect(jump)
    timer.start(10)
    sys.exit(app.exec_())


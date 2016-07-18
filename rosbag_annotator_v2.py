#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""The Video Widget example shows how to implement a video widget
using QtMultimedia's QAbstractVideoSurface.

The following is a translation into PyQt5 from the C++ example found in
C:\QtEnterprise\5.1.1\msvc2010\examples\multimediawidgets\customvideosurface\customvideowidget."""



import sys
import os
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtMultimedia import *
from PyQt5.QtMultimediaWidgets import *


class VideoWidgetSurface(QAbstractVideoSurface):

    def __init__(self, widget, parent=None):
        super(VideoWidgetSurface, self).__init__(parent)
        self.widget = widget
        self.imageFormat = QImage.Format_Invalid

    def supportedPixelFormats(self, handleType=QAbstractVideoBuffer.NoHandle):
        formats = [QVideoFrame.PixelFormat()]
        if (handleType == QAbstractVideoBuffer.NoHandle):
            for f in [QVideoFrame.Format_RGB32, QVideoFrame.Format_ARGB32, QVideoFrame.Format_ARGB32_Premultiplied, QVideoFrame.Format_RGB565, QVideoFrame.Format_RGB555]:
             formats.append(f)
        #print  formats
        return formats

    def isFormatSupported(self, _format):
        print 'formatSupport'
        imageFormat = QVideoFrame.imageFormatFromPixelFormat(_format.pixelFormat())
        size = _format.frameSize()
        _bool = False
        if (imageFormat != QImage.Format_Invalid and not size.isEmpty() and _format.handleType() == QAbstractVideoBuffer.NoHandle):
            _bool = True
        print _bool
        return _bool

    def start(self, _format):
        imageFormat = QVideoFrame.imageFormatFromPixelFormat(_format.pixelFormat())
        size = _format.frameSize()
        if (imageFormat != QImage.Format_Invalid and not size.isEmpty()):
            self.imageFormat = imageFormat
            self.imageSize = size
            self.sourceRect = _format.viewport()
            QAbstractVideoSurface.start(self, _format)
            self.widget.updateGeometry()
            self.updateVideoRect()
            return True
        else:
            return False

    def stop(self):
        self.currentFrame = QVideoFrame()
        #self.targetRect = QRect()
        QAbstractVideoSurface.stop(self)
        self.widget.update()

    def present(self, frame):
        #print 'present' #Prints
        if (self.surfaceFormat().pixelFormat() != frame.pixelFormat() or self.surfaceFormat().frameSize() != frame.size()):
            self.setError(QAbstractVideoSurface.IncorrectFormatError)
            self.stop()
            return False
        else:
            print 'somthing'
            self.currentFrame = frame
            print self.targetRect
            #self.widget.repaint(self.targetRect)
            #self.widget.repaint(self.currentFrame)
            painter = QPainter(self.widget)
            if (self.widget.surface.isActive()):
                print 'mpike'
                videoRect = self.widget.surface.videoRect()
            if not videoRect.contains(self.targetRect):
                print 'edw'
                region = event.region()
                region.subtract(videoRect)
                brush = self.palette().background()
                for rect in region.rects():
                    painter.fillRect(rect, brush)
                    self.surface.paint(painter)
            else:
                painter.fillRect(self.targetRect, self.widget.palette().window())
            #self.widget.update()
            return True

    def videoRect(self):
        print 'videoRect here'
        return self.targetRect

    def updateVideoRect(self):
        print 'update videorect' #Prints
        size = self.surfaceFormat().sizeHint()
        size.scale(self.widget.size().boundedTo(size), Qt.KeepAspectRatio)
        self.targetRect = QRect(QPoint(0, 0), size);
        self.targetRect.moveCenter(self.widget.rect().center())

    def paint(self, painter):
        print 'paint'
        if (self.currentFrame.map(QAbstractVideoBuffer.ReadOnly)):
            oldTransform = painter.transform()

        if (self.surfaceFormat().scanLineDirection() == QVideoSurfaceFormat.BottomToTop):
            painter.scale(1, -1);
            painter.translate(0, -self.widget.height())

        image = QImage(self.currentFrame.bits(),
                    self.currentFrame.width(),
                    self.currentFrame.height(),
                    self.currentFrame.bytesPerLine(),
                    self.imageFormat
        )

        painter.drawImage(self.targetRect, image, self.sourceRect)
        print self.sourceRect
        painter.setTransform(oldTransform)

        self.currentFrame.unmap()


class VideoWidget(QWidget):

    def __init__(self, parent=None):
        super(VideoWidget, self).__init__(parent)
        self.setAutoFillBackground(False)
        self.setAttribute(Qt.WA_NoSystemBackground, True)
        #self.setAttribute(Qt.WA_OpaquePaintEvent)
        self.setAttribute(Qt.WA_PaintOnScreen, True)
        palette = self.palette()
        palette.setColor(QPalette.Background, Qt.black)
        self.setPalette(palette)
        self.setSizePolicy(QSizePolicy.MinimumExpanding,
        QSizePolicy.MinimumExpanding)
        self.surface = VideoWidgetSurface(self)

    def videoSurface(self):
        return self.surface

    def closeEvent(self, event):
        del self.surface

    def sizeHint(self):
        return self.surface.surfaceFormat().sizeHint()
    '''
    def paintEvent(self, event):
        print 'paintEvent   '
        painter = QPainter(self)
        if (self.surface.isActive()):
            videoRect = self.surface.videoRect()
        if not videoRect.contains(event.rect()):
            print 'edw'
            region = event.region()
            region.subtract(videoRect)
            brush = self.palette().background()
            for rect in region.rects():
                painter.fillRect(rect, brush)
                self.surface.paint(painter)
        else:
            painter.fillRect(event.rect(), self.palette().window())
    '''

    def resizeEvent(self, event):
        print 'resizeEvent'
        QWidget.resizeEvent(self, event)
        self.surface.updateVideoRect()

class VideoPlayer(QWidget):
    def __init__(self, parent=None):
        super(VideoPlayer, self).__init__(parent)

        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)

        self.videoWidget = VideoWidget()
        self.openButton = QPushButton("Open...")
        self.openButton.clicked.connect(self.openFile)
        self.playButton = QPushButton()
        self.playButton.setEnabled(False)
        self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.playButton.clicked.connect(self.play)

        self.positionSlider = QSlider(Qt.Horizontal)
        self.positionSlider.setRange(0, 0)
        self.positionSlider.sliderMoved.connect(self.setPosition)
        self.controlLayout = QHBoxLayout()
        self.controlLayout.setContentsMargins(0, 0, 0, 0)
        self.controlLayout.addWidget(self.openButton)
        self.controlLayout.addWidget(self.playButton)
        self.controlLayout.addWidget(self.positionSlider)

        layout = QVBoxLayout()
        layout.addWidget(self.videoWidget)
        layout.addLayout(self.controlLayout)

        self.setLayout(layout)

        self.mediaPlayer.setVideoOutput(self.videoWidget.videoSurface())
        self.mediaPlayer.stateChanged.connect(self.mediaStateChanged)
        self.mediaPlayer.positionChanged.connect(self.positionChanged)
        self.mediaPlayer.durationChanged.connect(self.durationChanged)
        self.mediaPlayer.error.connect(self.handleError)

    def openFile(self):
        file_name, _ = QFileDialog.getOpenFileName(self, "Open Movie")
        if os.path.exists(file_name):
            self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(file_name)))
        self.playButton.setEnabled(True)

    def play(self):
        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.mediaPlayer.pause()
        else:
            self.mediaPlayer.play()

    def mediaStateChanged(self, state):
        if state == QMediaPlayer.PlayingState:
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPause))
        else:
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))

    def positionChanged(self, position):
        self.positionSlider.setValue(position)

    def durationChanged(self, duration):
        self.positionSlider.setRange(0, duration)

    def setPosition(self, position):
        self.mediaPlayer.setPosition(position)

    #update duration
    def updateDurationInfo(self, currentInfo):
        duration = self.duration
        if currentInfo or duration:
            currentTime = QTime((currentInfo/3600)%60, (currentInfo/60)%60,
                    currentInfo%60, (currentInfo*1000)%1000)
            totalTime = QTime((duration/3600)%60, (duration/60)%60,
                    duration%60, (duration*1000)%1000);

            format = 'hh:mm:ss' if duration > 3600 else 'mm:ss'
            tStr = currentTime.toString(format) + " / " + totalTime.toString(format)
        else:
            tStr = ""
        self.labelDuration.setText(tStr)

    def handleError(self):
        self.playButton.setEnabled(False)
        self.errorLabel.setText("Error: " + self.mediaPlayer.errorString())


if __name__ == '__main__':
    app = QApplication(sys.argv)

    player = VideoPlayer()
    player.resize(640,480)
    player.show()

    sys.exit(app.exec_())


#-------------------------------------------------
#
# Project created by QtCreator 2018-09-30T20:00:09
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = newApp
TEMPLATE = app
CONFIG+=c++11
QMAKE_CFLAGS += -std=c++11

INCLUDEPATH += /usr/local/include\
                /usr/local/include/opencv\
                /usr/local/include/opencv2\
                /usr/local/include/ARToolKitPlus\
                /usr/local/include/IMUDrivers\
                /usr/include/eigen3


LIBS += /usr/local/lib/libopencv_highgui.so\
        /usr/local/lib/libopencv_core.so\
        /usr/local/lib/libopencv_imgproc.so\
        /usr/local/lib/libopencv_calib3d.so\
        /usr/local/lib/libopencv_features2d.so\
        /usr/local/lib/libopencv_flann.so\
        /usr/local/lib/libopencv_imgcodecs.so\
        /usr/local/lib/libopencv_ml.so\
        /usr/local/lib/libopencv_objdetect.so\
        /usr/local/lib/libopencv_photo.so\
        /usr/local/lib/libopencv_shape.so\
        /usr/local/lib/libopencv_stitching.so\
        /usr/local/lib/libopencv_superres.so\
        /usr/local/lib/libopencv_video.so\
        /usr/local/lib/libopencv_videoio.so\
        /usr/local/lib/libopencv_videostab.so\
        /usr/local/lib/libARToolKitPlus.so

SOURCES += main.cpp\
        MainWindow.cpp \
    dataLib.cpp \
    stm32.cpp \
    uart.cpp \
    vp_vision.cpp \
    houghLine.cpp \
    auto_canny.cpp

HEADERS  += MainWindow.h \
    dataLib.h \
    stm32.h \
    commandLib.h \
    uart.h \
    vp_vision.h \
    houghLine.h \
    auto_canny.h

FORMS    += MainWindow.ui

RESOURCES += \
    pitcures.qrc

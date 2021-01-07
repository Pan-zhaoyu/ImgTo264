TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp

HEADERS += \


INCLUDEPATH  += /usr/local/include/opencv \
                /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_shape.so
LIBS += /usr/local/lib/libopencv_videoio.so
LIBS += -lpthread
LIBS += -L /usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui
LIBS += /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_core.so    \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_imgcodecs.so
LIBS += /usr/lib/libx264.so

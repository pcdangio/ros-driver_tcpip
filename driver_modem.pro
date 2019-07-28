TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += \
    /usr/include
    /opt/ros/melodic/include

SOURCES += \
    src/udp_connection.cpp \
    src/tcp_connection.cpp

DISTFILES += \
    CMakeLists.txt \
    LICENSE \
    package.xml

HEADERS += \
    src/connection_type.h \
    src/udp_connection.h \
    src/tcp_connection.h

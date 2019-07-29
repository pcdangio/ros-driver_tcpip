TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += \
    /usr/include \
    /opt/ros/melodic/include \
    ../../devel/include

SOURCES += \
    src/driver.cpp \
    src/main.cpp \
    src/ros_node.cpp \
    src/udp_connection.cpp \
    src/tcp_connection.cpp

DISTFILES += \
    CMakeLists.txt \
    LICENSE \
    msg/DataPacket.msg \
    package.xml \
    srv/TCPtx.srv

HEADERS += \
    src/connection_type.h \
    src/driver.h \
    src/ros_node.h \
    src/udp_connection.h \
    src/tcp_connection.h

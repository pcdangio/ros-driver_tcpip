TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += \
    include \
    /usr/include \
    /opt/ros/melodic/include \
    ../../devel/include

SOURCES += \
    src/driver.cpp \
    src/main.cpp \
    src/modem_interface.cpp \
    src/ros_node.cpp \
    src/udp_connection.cpp \
    src/tcp_connection.cpp

DISTFILES += \
    CMakeLists.txt \
    LICENSE \
    README.md \
    msg/ActiveConnections.msg \
    msg/DataPacket.msg \
    package.xml \
    srv/AddTCPConnection.srv \
    srv/AddUDPConnection.srv \
    srv/GetRemoteHost.srv \
    srv/RemoveConnection.srv \
    srv/SendTCP.srv \
    srv/SetRemoteHost.srv

HEADERS += \
    include/driver_modem/protocol.h \
    include/driver_modem/modem_interface.h \
    include/driver_modem/tcp_role.h \
    src/driver.h \
    src/ros_node.h \
    src/udp_connection.h \
    src/tcp_connection.h

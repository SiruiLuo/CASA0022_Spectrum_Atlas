QT       += widgets
CONFIG   += c++17 link_pkgconfig
TARGET    = ros_gui
TEMPLATE  = app

# Ensure qmake calls pkg-config with ROS paths
ROS_PKGCFG = /opt/ros/melodic/lib/pkgconfig:/opt/ros/melodic/share/pkgconfig
QMAKE_PKG_CONFIG = env PKG_CONFIG_PATH=$$ROS_PKGCFG pkg-config

SOURCES  += \
    main.cpp \
    mainwindow.cpp

HEADERS  += \
    mainwindow.h

FORMS    += \
    mainwindow.ui

# Remove rviz, keep only roscpp and core packages
PKGCONFIG += roscpp tf tf2_ros image_transport
INCLUDEPATH += /opt/ros/melodic/include

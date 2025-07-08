QT       += widgets
CONFIG   += c++17 link_pkgconfig
TARGET    = ros_gui
TEMPLATE  = app

# 让 qmake 调 pkg-config 时带上 ROS 路径
ROS_PKGCFG = /opt/ros/melodic/lib/pkgconfig:/opt/ros/melodic/share/pkgconfig
QMAKE_PKG_CONFIG = env PKG_CONFIG_PATH=$$ROS_PKGCFG pkg-config

SOURCES  += \
    main.cpp \
    mainwindow.cpp

HEADERS  += \
    mainwindow.h

FORMS    += \
    mainwindow.ui

# 去掉 rviz，只保留 roscpp 等
PKGCONFIG += roscpp tf tf2_ros image_transport
INCLUDEPATH += /opt/ros/melodic/include

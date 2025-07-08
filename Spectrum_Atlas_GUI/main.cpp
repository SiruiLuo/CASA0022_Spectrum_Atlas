#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <sstream>
#include <unistd.h>        // execvp

#include <QApplication>
#include <QTimer>
#include <ros/ros.h>

#include "mainwindow.h"

/* ---------- 修改这里即可适配 Noetic 等 ---------- */
static const char* ROS_ROOT_PATH = "/opt/ros/melodic";
static const char* ENV_READY     = "ROS_GUI_ENV_READY";

/* rospack 能否找到 rviz？ */
static bool rvizFound()
{
    FILE* pipe = popen("rospack find rviz 2>/dev/null", "r");
    if (!pipe) return false;
    char buf[8];
    bool ok = (fgets(buf, sizeof(buf), pipe) != nullptr);
    pclose(pipe);
    return ok;
}

/* 前置 PATH / LD_LIBRARY_PATH 等 */
static void prependEnv(const char* key, const std::string& prefix)
{
    const char* old = getenv(key);
    std::string val = prefix;
    if (old && *old) val += ":" + std::string(old);
    setenv(key, val.c_str(), 1);
}

/* --------- 准备 ROS 环境，不够就自重启 --------- */
static void prepareEnvAndReexec(int argc, char** argv)
{
    if (getenv(ENV_READY) || rvizFound())
        return;

    /* Melodic 基础路径 */
    prependEnv("PATH", std::string(ROS_ROOT_PATH) + "/bin");
    prependEnv("PYTHONPATH",
               std::string(ROS_ROOT_PATH) + "/lib/python2.7/dist-packages");
    prependEnv("LD_LIBRARY_PATH",
               std::string(ROS_ROOT_PATH) + "/lib:" +
               std::string(ROS_ROOT_PATH) + "/lib/arm-linux-gnueabihf");
    prependEnv("PKG_CONFIG_PATH",
               std::string(ROS_ROOT_PATH) + "/lib/pkgconfig:" +
               std::string(ROS_ROOT_PATH) + "/share/pkgconfig");
    prependEnv("ROS_PACKAGE_PATH",
               std::string(ROS_ROOT_PATH) + "/share:" +
               std::string(ROS_ROOT_PATH) + "/stacks");

    setenv("ROS_ROOT",
           (std::string(ROS_ROOT_PATH) + "/share/ros").c_str(), 1);
    setenv("ROS_ETC_DIR",
           (std::string(ROS_ROOT_PATH) + "/etc/ros").c_str(), 1);
    setenv("ROS_MASTER_URI", "http://localhost:11311", 0);

    /* 移除 ROS2 路径 */
    if (const char* raw = getenv("LD_LIBRARY_PATH")) {
        std::stringstream ss(raw); std::string seg, clean;
        while (std::getline(ss, seg, ':')) {
            if (seg.find("/opt/ros/foxy")   != std::string::npos ||
                seg.find("/opt/ros/humble") != std::string::npos ||
                seg.find("/opt/ros/iron")   != std::string::npos) continue;
            if (!seg.empty()) clean += seg + ":";
        }
        std::string lead = std::string(ROS_ROOT_PATH) + "/lib:" +
                           std::string(ROS_ROOT_PATH) + "/lib/arm-linux-gnueabihf";
        clean = lead + ":" + clean;
        if (!clean.empty() && clean.back() == ':') clean.pop_back();
        setenv("LD_LIBRARY_PATH", clean.c_str(), 1);
    }

    setenv(ENV_READY, "1", 1);
    fprintf(stderr, "[INFO] Environment prepared, re–exec...\n");
    execvp(argv[0], argv);
    perror("execvp failed");
    std::exit(EXIT_FAILURE);
}

/* ---------------- main ---------------- */
int main(int argc, char** argv)
{
    prepareEnvAndReexec(argc, argv);

    ros::init(argc, argv, "qt_ros_gui");
    QApplication app(argc, argv);

    MainWindow w; w.show();

    QTimer rosTimer;
    QObject::connect(&rosTimer, &QTimer::timeout, []{ ros::spinOnce(); });
    rosTimer.start(30);

    return app.exec();
}

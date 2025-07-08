#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <QTimer>
#include <QSystemTrayIcon>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace Ui { class MainWindow; }

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    void closeEvent(QCloseEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;    // 保证窗口变化时同步缩放

private slots:
    void on_startButton_clicked();
    void on_stopButton_clicked();
    void queryRssi();                              // 周期性读取 RSSI
    void iconActivated(QSystemTrayIcon::ActivationReason reason);

private:               /* ---------- 地图订阅与渲染 ---------- */
    void startMapSubscriber();
    void stopMapSubscriber();
    void onMap(const nav_msgs::OccupancyGridConstPtr& msg);   // 回调

private:               /* ---------- 进程&UI ---------- */
    void killProcesses();

    Ui::MainWindow      *ui;

    QProcess *roscoreProc_ {nullptr};
    QProcess *ldProc_     {nullptr};
    QProcess *hectorProc_ {nullptr};
    QProcess *rssiProc_   {nullptr};

    QTimer     rssiTimer_;
    QSystemTrayIcon tray_;

    /* ROS */
    ros::NodeHandle nh_;
    ros::Subscriber mapSub_;

    /* 最新一帧原始 QImage，用于 resizeEvent 里重复缩放 */
    QImage rawMap_;
};

#endif // MAINWINDOW_H

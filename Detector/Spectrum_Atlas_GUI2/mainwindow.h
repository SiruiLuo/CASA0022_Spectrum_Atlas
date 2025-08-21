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
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void on_startButton_clicked();
    void on_stopButton_clicked();
    void queryRssi();
    void iconActivated(QSystemTrayIcon::ActivationReason reason);

private:                /* ---------- Map subscription and rendering ---------- */
    void startMapSubscriber();
    void stopMapSubscriber();
    void onMap(const nav_msgs::OccupancyGridConstPtr& msg);

private:                /* ---------- ROS/SDR Processes and UI ---------- */
    void killProcesses();

    Ui::MainWindow      *ui;

    QProcess *roscoreProc_ {nullptr};
    QProcess *ldProc_     {nullptr};
    QProcess *hectorProc_ {nullptr};
    QProcess *rssiProc_   {nullptr};
    QProcess *loggerProc_ {nullptr};          // ✚ Added: RTL-SDR logging script

    QTimer     rssiTimer_;
    QTimer     loggerTimer_;
    QSystemTrayIcon tray_;

    /* ROS */
    ros::NodeHandle nh_;
    ros::Subscriber mapSub_;

    /* Latest captured raw QImage */
    QImage rawMap_;
};

#endif // MAINWINDOW_H

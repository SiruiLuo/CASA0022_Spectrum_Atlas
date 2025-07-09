#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QCloseEvent>
#include <QMessageBox>
#include <QThread>
#include <QFile>
#include <QTextStream>
#include <QDir>

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <sstream>
#include <unistd.h>

/* --- 保存地图的默认路径 --- */
const QString kSavedMapDir = "/home/Steven/maps";
const QString kMapPgmPath = kSavedMapDir + "/map.pgm";
const QString kMapYamlPath = kSavedMapDir + "/map.yaml";
const QString kYamlTemplate = R"(image: map.pgm
resolution: {res}
origin: [{ox}, {oy}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
)";

/* ---------------- 构造 ---------------- */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    tray_.setIcon(QIcon::fromTheme("applications-graphics"));
    tray_.setToolTip(tr("ROS Mapping GUI"));
    tray_.show();
    connect(&tray_, &QSystemTrayIcon::activated,
            this, &MainWindow::iconActivated);

    rssiTimer_.setInterval(3000);
    connect(&rssiTimer_, &QTimer::timeout, this, &MainWindow::queryRssi);

    loggerTimer_.setInterval(3000);
    connect(&loggerTimer_, &QTimer::timeout, this, [this]{
        QFile f("/home/Steven/Spectrum_Atlas_GUI2/logger_status.txt");
        if (f.open(QIODevice::ReadOnly | QIODevice::Text)) {
            const QString s = QTextStream(&f).readAll().trimmed();
            ui->loggerStatusLabel->setText(s.isEmpty() ? "扫描状态：--" :
                                                     "扫描状态：" + s);
        }
    });
    loggerTimer_.start();

    ui->mapLabel->setText(tr("等待地图…"));
    ui->mapLabel->setScaledContents(true);
}

MainWindow::~MainWindow()
{
    killProcesses();
    delete ui;
}

void MainWindow::on_startButton_clicked()
{
    if (roscoreProc_) {
        QMessageBox::information(this, tr("提示"), tr("ROS 已经在运行。"));
        return;
    }

    roscoreProc_ = new QProcess(this);
    roscoreProc_->start("/bin/bash", {"-c", "roscore"});
    if (!roscoreProc_->waitForStarted(3000)) {
        QMessageBox::critical(this, tr("错误"), tr("无法启动 roscore！"));
        killProcesses(); return;
    }
    QThread::sleep(2);

    ldProc_ = new QProcess(this);
    const QString ldCmd =
        "source /opt/ros/melodic/setup.bash && "
        "cd ~/ldlidar_ros_ws && "
        "catkin_make && source devel/setup.bash && "
        "roslaunch ldlidar_stl_ros ld19.launch";
    ldProc_->start("/bin/bash", {"-c", ldCmd});

    hectorProc_ = new QProcess(this);
    const QString hectorCmd =
        "source /opt/ros/melodic/setup.bash && "
        "cd ~/ldlidar_ros_ws && "
        "source devel/setup.bash && "
        "roslaunch hector_slam_launch tutorial.launch";
    hectorProc_->start("/bin/bash", {"-c", hectorCmd});

    loggerProc_ = new QProcess(this);
    loggerProc_->start("/bin/bash",
                       {"-c", "python3 /home/Steven/Spectrum_Atlas_GUI2/ros_sdr_logger.py"});

    rssiTimer_.start();
    startMapSubscriber();
}

void MainWindow::on_stopButton_clicked()
{
    killProcesses();
    close();
}

void MainWindow::startMapSubscriber()
{
    if (mapSub_) return;
    mapSub_ = nh_.subscribe("map", 1, &MainWindow::onMap, this);
}

void MainWindow::stopMapSubscriber()
{
    mapSub_.shutdown();
}

void MainWindow::onMap(const nav_msgs::OccupancyGridConstPtr& msg)
{
    const int W = msg->info.width, H = msg->info.height;
    if (!W || !H) return;

    int minX=W, minY=H, maxX=-1, maxY=-1;
    const auto &src = msg->data;
    for (int y=0; y<H; ++y) {
        const int8_t *row = &src[y*W];
        for (int x=0; x<W; ++x) {
            int8_t v = row[x];
            if (v >= 0) { minX=std::min(minX,x); maxX=std::max(maxX,x);
                          minY=std::min(minY,y); maxY=std::max(maxY,y); }
        }
    }
    if (maxX < 0) return;
    const int pad = 5;
    minX = std::max(0, minX-pad); minY = std::max(0, minY-pad);
    maxX = std::min(W-1, maxX+pad); maxY = std::min(H-1, maxY+pad);

    QImage full(W, H, QImage::Format_Indexed8);
    QVector<QRgb> pal(256, qRgb(0,0,0));
    pal[254] = qRgb(255,255,255); pal[205] = qRgb(160,160,160);
    full.setColorTable(pal);

    for (int y=0; y<H; ++y) {
        uchar *dst = full.scanLine(H-1-y);
        const int8_t *row = &src[y*W];
        for (int x=0; x<W; ++x) {
            int8_t v = row[x];
            dst[x] = (v < 0) ? 205 : (v==0 ? 254 : 0);
        }
    }
    rawMap_ = full.copy(minX, H-1-maxY, maxX-minX+1, maxY-minY+1);

    QPixmap pix = QPixmap::fromImage(
        rawMap_.scaled(ui->mapLabel->size(),
                       Qt::KeepAspectRatio, Qt::FastTransformation));
    ui->mapLabel->setPixmap(pix);

    // ✚ 保存地图（.pgm 和 .yaml）
    QDir().mkpath(kSavedMapDir);
    rawMap_.save(kMapPgmPath, "PGM");

    const auto& info = msg->info;
    const double res = info.resolution;
    const double ox = info.origin.position.x;
    const double oy = info.origin.position.y;

    QFile yamlFile(kMapYamlPath);
    if (yamlFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&yamlFile);
        QString yaml = kYamlTemplate;
        yaml.replace("{res}", QString::number(res, 'f', 6));
        yaml.replace("{ox}", QString::number(ox, 'f', 6));
        yaml.replace("{oy}", QString::number(oy, 'f', 6));
        out << yaml;
        yamlFile.close();
    }
}

void MainWindow::resizeEvent(QResizeEvent*)
{
    if (!rawMap_.isNull()) {
        QPixmap pix = QPixmap::fromImage(
            rawMap_.scaled(ui->mapLabel->size(),
                           Qt::KeepAspectRatio, Qt::FastTransformation));
        ui->mapLabel->setPixmap(pix);
    }
}

void MainWindow::killProcesses()
{
    auto safeKill = [](QProcess *&p){
        if (!p) return;
        p->terminate();
        if (!p->waitForFinished(2000)) p->kill();
        delete p; p = nullptr;
    };

    safeKill(loggerProc_);
    safeKill(hectorProc_);
    safeKill(ldProc_);
    safeKill(roscoreProc_);
    safeKill(rssiProc_);

    stopMapSubscriber();
    rssiTimer_.stop();
    loggerTimer_.stop();
    rawMap_ = QImage();

    ui->rssiLabel->setText("RSSI: -- dB");
    ui->loggerStatusLabel->setText("扫描状态：--");
    ui->mapLabel->setText(tr("等待地图…"));
}

void MainWindow::queryRssi()
{
    if (rssiProc_) return;
    rssiProc_ = new QProcess(this);

    connect(rssiProc_,
            static_cast<void(QProcess::*)(int,QProcess::ExitStatus)>
            (&QProcess::finished),
            this,[this](int, QProcess::ExitStatus)
    {
        const QString out = rssiProc_->readAllStandardOutput().trimmed();
        bool ok=false; double rssi = out.toDouble(&ok);
        ui->rssiLabel->setText(ok ?
            QString("RSSI: %1 dB").arg(rssi,0,'f',1) :
            "RSSI: -- dB");
        rssiProc_->deleteLater(); rssiProc_=nullptr;
    });

    rssiProc_->start("/bin/bash",
                     {"-c","python3 /home/Steven/scripts/read_rssi.py"});
}

void MainWindow::iconActivated(QSystemTrayIcon::ActivationReason reason)
{
    if (reason == QSystemTrayIcon::Trigger) {
        showNormal(); raise(); activateWindow();
    }
}

void MainWindow::closeEvent(QCloseEvent *e)
{
    if (QMessageBox::question(this, tr("退出确认"),
                              tr("确定要退出并关闭所有 ROS 节点吗？"))
        == QMessageBox::Yes)
    {
        killProcesses(); tray_.hide(); e->accept();
    } else e->ignore();
}

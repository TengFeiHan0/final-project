/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPlainTextEdit>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *IMG_label;
    QPushButton *land_pb;
    QPushButton *takeoff_pb;
    QPlainTextEdit *msg_plainTextEdit;
    QLabel *label_2;
    QGroupBox *groupBox;
    QRadioButton *manual_rb;
    QRadioButton *none_rb;
    QRadioButton *height_rb;
    QRadioButton *pos_rb;
    QCheckBox *Auto_CB;
    QPushButton *flat_pb;
    QGroupBox *groupBox_2;
    QLabel *label;
    QLabel *batt_label;
    QLabel *rx_label;
    QLabel *label_5;
    QLabel *ry_label;
    QLabel *rz_label;
    QLabel *label_8;
    QLabel *height_label;
    QLabel *label_10;
    QLabel *height_label_2;
    QLabel *label_3;
    QLabel *orb_label;
    QLabel *label_4;
    QLabel *cam_x_label;
    QLabel *cam_y_label;
    QLabel *cam_z_label;
    QLabel *az_label;
    QLabel *label_11;
    QLabel *ax_label;
    QLabel *ay_label;
    QLabel *vz_label;
    QLabel *label_12;
    QLabel *vy_label;
    QLabel *vx_label;
    QLabel *label_13;
    QLabel *YAW_label;
    QPushButton *togglecam_pb;
    QPushButton *orb_pb;
    QPushButton *slam_calib_pb;
    QLabel *label_6;
    QComboBox *x_cn_num;
    QComboBox *height_cn_num;
    QLabel *label_7;
    QLabel *label_9;
    QPushButton *pos_setpoint_pb;
    QComboBox *y_cn_num;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(900, 640);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        IMG_label = new QLabel(centralWidget);
        IMG_label->setObjectName(QString::fromUtf8("IMG_label"));
        IMG_label->setGeometry(QRect(20, 10, 640, 320));
        land_pb = new QPushButton(centralWidget);
        land_pb->setObjectName(QString::fromUtf8("land_pb"));
        land_pb->setGeometry(QRect(10, 430, 61, 27));
        takeoff_pb = new QPushButton(centralWidget);
        takeoff_pb->setObjectName(QString::fromUtf8("takeoff_pb"));
        takeoff_pb->setGeometry(QRect(90, 430, 71, 27));
        msg_plainTextEdit = new QPlainTextEdit(centralWidget);
        msg_plainTextEdit->setObjectName(QString::fromUtf8("msg_plainTextEdit"));
        msg_plainTextEdit->setEnabled(true);
        msg_plainTextEdit->setGeometry(QRect(10, 480, 371, 91));
        msg_plainTextEdit->setReadOnly(true);
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(10, 460, 71, 17));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(420, 470, 271, 101));
        manual_rb = new QRadioButton(groupBox);
        manual_rb->setObjectName(QString::fromUtf8("manual_rb"));
        manual_rb->setGeometry(QRect(10, 30, 91, 22));
        none_rb = new QRadioButton(groupBox);
        none_rb->setObjectName(QString::fromUtf8("none_rb"));
        none_rb->setGeometry(QRect(10, 50, 117, 22));
        height_rb = new QRadioButton(groupBox);
        height_rb->setObjectName(QString::fromUtf8("height_rb"));
        height_rb->setGeometry(QRect(100, 30, 141, 22));
        pos_rb = new QRadioButton(groupBox);
        pos_rb->setObjectName(QString::fromUtf8("pos_rb"));
        pos_rb->setGeometry(QRect(100, 50, 141, 22));
        Auto_CB = new QCheckBox(groupBox);
        Auto_CB->setObjectName(QString::fromUtf8("Auto_CB"));
        Auto_CB->setGeometry(QRect(120, 72, 97, 22));
        flat_pb = new QPushButton(centralWidget);
        flat_pb->setObjectName(QString::fromUtf8("flat_pb"));
        flat_pb->setGeometry(QRect(180, 430, 71, 27));
        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(680, 10, 191, 391));
        label = new QLabel(groupBox_2);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(0, 30, 41, 17));
        batt_label = new QLabel(groupBox_2);
        batt_label->setObjectName(QString::fromUtf8("batt_label"));
        batt_label->setGeometry(QRect(50, 30, 31, 17));
        rx_label = new QLabel(groupBox_2);
        rx_label->setObjectName(QString::fromUtf8("rx_label"));
        rx_label->setGeometry(QRect(0, 90, 38, 17));
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(0, 60, 41, 17));
        ry_label = new QLabel(groupBox_2);
        ry_label->setObjectName(QString::fromUtf8("ry_label"));
        ry_label->setGeometry(QRect(60, 90, 38, 17));
        rz_label = new QLabel(groupBox_2);
        rz_label->setObjectName(QString::fromUtf8("rz_label"));
        rz_label->setGeometry(QRect(120, 90, 38, 17));
        label_8 = new QLabel(groupBox_2);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(0, 120, 51, 17));
        height_label = new QLabel(groupBox_2);
        height_label->setObjectName(QString::fromUtf8("height_label"));
        height_label->setGeometry(QRect(60, 120, 41, 17));
        label_10 = new QLabel(groupBox_2);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(80, 30, 21, 17));
        height_label_2 = new QLabel(groupBox_2);
        height_label_2->setObjectName(QString::fromUtf8("height_label_2"));
        height_label_2->setGeometry(QRect(110, 120, 41, 17));
        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(0, 254, 41, 17));
        orb_label = new QLabel(groupBox_2);
        orb_label->setObjectName(QString::fromUtf8("orb_label"));
        orb_label->setGeometry(QRect(50, 254, 71, 17));
        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(0, 281, 81, 17));
        cam_x_label = new QLabel(groupBox_2);
        cam_x_label->setObjectName(QString::fromUtf8("cam_x_label"));
        cam_x_label->setGeometry(QRect(0, 300, 51, 17));
        cam_y_label = new QLabel(groupBox_2);
        cam_y_label->setObjectName(QString::fromUtf8("cam_y_label"));
        cam_y_label->setGeometry(QRect(60, 300, 51, 17));
        cam_z_label = new QLabel(groupBox_2);
        cam_z_label->setObjectName(QString::fromUtf8("cam_z_label"));
        cam_z_label->setGeometry(QRect(120, 300, 51, 17));
        az_label = new QLabel(groupBox_2);
        az_label->setObjectName(QString::fromUtf8("az_label"));
        az_label->setGeometry(QRect(120, 170, 38, 17));
        label_11 = new QLabel(groupBox_2);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(0, 150, 111, 17));
        ax_label = new QLabel(groupBox_2);
        ax_label->setObjectName(QString::fromUtf8("ax_label"));
        ax_label->setGeometry(QRect(0, 170, 38, 17));
        ay_label = new QLabel(groupBox_2);
        ay_label->setObjectName(QString::fromUtf8("ay_label"));
        ay_label->setGeometry(QRect(60, 170, 38, 17));
        vz_label = new QLabel(groupBox_2);
        vz_label->setObjectName(QString::fromUtf8("vz_label"));
        vz_label->setGeometry(QRect(120, 220, 38, 17));
        label_12 = new QLabel(groupBox_2);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(0, 200, 111, 17));
        vy_label = new QLabel(groupBox_2);
        vy_label->setObjectName(QString::fromUtf8("vy_label"));
        vy_label->setGeometry(QRect(60, 220, 38, 17));
        vx_label = new QLabel(groupBox_2);
        vx_label->setObjectName(QString::fromUtf8("vx_label"));
        vx_label->setGeometry(QRect(0, 220, 38, 17));
        label_13 = new QLabel(groupBox_2);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setGeometry(QRect(0, 330, 41, 17));
        YAW_label = new QLabel(groupBox_2);
        YAW_label->setObjectName(QString::fromUtf8("YAW_label"));
        YAW_label->setGeometry(QRect(45, 330, 51, 17));
        togglecam_pb = new QPushButton(centralWidget);
        togglecam_pb->setObjectName(QString::fromUtf8("togglecam_pb"));
        togglecam_pb->setGeometry(QRect(260, 430, 111, 27));
        orb_pb = new QPushButton(centralWidget);
        orb_pb->setObjectName(QString::fromUtf8("orb_pb"));
        orb_pb->setGeometry(QRect(390, 430, 111, 27));
        slam_calib_pb = new QPushButton(centralWidget);
        slam_calib_pb->setObjectName(QString::fromUtf8("slam_calib_pb"));
        slam_calib_pb->setGeometry(QRect(510, 430, 91, 27));
        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(620, 435, 21, 16));
        x_cn_num = new QComboBox(centralWidget);
        x_cn_num->setObjectName(QString::fromUtf8("x_cn_num"));
        x_cn_num->setGeometry(QRect(640, 430, 51, 27));
        height_cn_num = new QComboBox(centralWidget);
        height_cn_num->setObjectName(QString::fromUtf8("height_cn_num"));
        height_cn_num->setGeometry(QRect(800, 430, 51, 27));
        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(700, 435, 21, 16));
        label_9 = new QLabel(centralWidget);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(780, 435, 21, 16));
        pos_setpoint_pb = new QPushButton(centralWidget);
        pos_setpoint_pb->setObjectName(QString::fromUtf8("pos_setpoint_pb"));
        pos_setpoint_pb->setGeometry(QRect(720, 518, 131, 27));
        y_cn_num = new QComboBox(centralWidget);
        y_cn_num->setObjectName(QString::fromUtf8("y_cn_num"));
        y_cn_num->setGeometry(QRect(716, 430, 51, 27));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 900, 25));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        x_cn_num->setCurrentIndex(15);
        height_cn_num->setCurrentIndex(12);
        y_cn_num->setCurrentIndex(15);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        IMG_label->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        land_pb->setText(QApplication::translate("MainWindow", "Land", 0, QApplication::UnicodeUTF8));
        takeoff_pb->setText(QApplication::translate("MainWindow", "Take off", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Messages:", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindow", "Controll", 0, QApplication::UnicodeUTF8));
        manual_rb->setText(QApplication::translate("MainWindow", "Manual", 0, QApplication::UnicodeUTF8));
        none_rb->setText(QApplication::translate("MainWindow", "None", 0, QApplication::UnicodeUTF8));
        height_rb->setText(QApplication::translate("MainWindow", "Height_Control", 0, QApplication::UnicodeUTF8));
        pos_rb->setText(QApplication::translate("MainWindow", "POS_Control", 0, QApplication::UnicodeUTF8));
        Auto_CB->setText(QApplication::translate("MainWindow", "Auto", 0, QApplication::UnicodeUTF8));
        flat_pb->setText(QApplication::translate("MainWindow", "Flat Trim", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "Status:", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Batt:", 0, QApplication::UnicodeUTF8));
        batt_label->setText(QApplication::translate("MainWindow", "100", 0, QApplication::UnicodeUTF8));
        rx_label->setText(QApplication::translate("MainWindow", "10.40", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "Tilt:", 0, QApplication::UnicodeUTF8));
        ry_label->setText(QApplication::translate("MainWindow", "10.40", 0, QApplication::UnicodeUTF8));
        rz_label->setText(QApplication::translate("MainWindow", "10.40", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MainWindow", "Height:", 0, QApplication::UnicodeUTF8));
        height_label->setText(QApplication::translate("MainWindow", "10.40", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("MainWindow", "%", 0, QApplication::UnicodeUTF8));
        height_label_2->setText(QApplication::translate("MainWindow", "cm", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "ORB:", 0, QApplication::UnicodeUTF8));
        orb_label->setText(QApplication::translate("MainWindow", "Unknown", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "CAM_POS:", 0, QApplication::UnicodeUTF8));
        cam_x_label->setText(QApplication::translate("MainWindow", "10.0", 0, QApplication::UnicodeUTF8));
        cam_y_label->setText(QApplication::translate("MainWindow", "10.0", 0, QApplication::UnicodeUTF8));
        cam_z_label->setText(QApplication::translate("MainWindow", "10.0", 0, QApplication::UnicodeUTF8));
        az_label->setText(QApplication::translate("MainWindow", "10.40", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("MainWindow", "Acceleration:", 0, QApplication::UnicodeUTF8));
        ax_label->setText(QApplication::translate("MainWindow", "10.40", 0, QApplication::UnicodeUTF8));
        ay_label->setText(QApplication::translate("MainWindow", "10.40", 0, QApplication::UnicodeUTF8));
        vz_label->setText(QApplication::translate("MainWindow", "10.40", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("MainWindow", "Velocity:", 0, QApplication::UnicodeUTF8));
        vy_label->setText(QApplication::translate("MainWindow", "10.40", 0, QApplication::UnicodeUTF8));
        vx_label->setText(QApplication::translate("MainWindow", "10.40", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("MainWindow", "YAW:", 0, QApplication::UnicodeUTF8));
        YAW_label->setText(QApplication::translate("MainWindow", "10.0", 0, QApplication::UnicodeUTF8));
        togglecam_pb->setText(QApplication::translate("MainWindow", "Toggle Camera", 0, QApplication::UnicodeUTF8));
        orb_pb->setText(QApplication::translate("MainWindow", "ORB_IMG_Off", 0, QApplication::UnicodeUTF8));
        slam_calib_pb->setText(QApplication::translate("MainWindow", "SLAM_Calib", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindow", "X:", 0, QApplication::UnicodeUTF8));
        x_cn_num->clear();
        x_cn_num->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "200", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "180", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "160", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "140", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "120", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "100", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "90", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "80", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "70", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "60", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "50", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "40", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "30", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "20", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "10", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-10", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-20", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-30", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-40", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-50", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-60", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-70", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-80", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-90", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-100", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-120", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-140", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-160", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-180", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-200", 0, QApplication::UnicodeUTF8)
        );
        x_cn_num->setItemText(x_cn_num->currentIndex(),QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        height_cn_num->clear();
        height_cn_num->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "200", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "180", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "160", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "140", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "130", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "120", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "110", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "100", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "90", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "80", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "70", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "60", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "50", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "40", 0, QApplication::UnicodeUTF8)
        );
        height_cn_num->setItemText(height_cn_num->currentIndex(),QApplication::translate("MainWindow", "50", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindow", "Y:", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("MainWindow", "Z:", 0, QApplication::UnicodeUTF8));
        pos_setpoint_pb->setText(QApplication::translate("MainWindow", "Set_POS_Setpoin", 0, QApplication::UnicodeUTF8));
        y_cn_num->clear();
        y_cn_num->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "200", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "180", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "160", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "140", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "120", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "100", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "90", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "80", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "70", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "60", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "50", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "40", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "30", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "20", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "10", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-10", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-20", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-30", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-40", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-50", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-60", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-70", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-80", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-90", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-100", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-120", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-140", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-160", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-180", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "-200", 0, QApplication::UnicodeUTF8)
        );
        y_cn_num->setItemText(y_cn_num->currentIndex(),QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

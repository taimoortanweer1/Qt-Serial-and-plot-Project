/*
* mainwindow.h
*
*  Created on  : Dec 10, 2017
*  Author      : Vinay Divakar
*  website     : www.deeplyemebedded.org
*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

/* Includes */
#include <QMainWindow>
#include<QTimer>
#include <QSerialPortInfo>
#include<QDebug>
#include<QThread>
#include<QFont>

#include "ComSerial.h"
#include "vector"
#include "iostream"
#include "thread"
#include "fstream"
#include "unistd.h"
#include "math.h"
#include "time.h"
#include "string.h"

using namespace std;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void initGUI();

private slots:




    void on_bttnRPMApply_clicked();


    void on_bttnConnectSP_clicked();

    void on_bttnDisconnectSP_clicked();

    void on_bttnRefreshSP_clicked();

    void on_bttnInit_clicked();

    void on_bttnCheckStatus_clicked();

    void on_bttnAutoTestStart_clicked();

    void on_bttnManualTestStart_clicked();

    void on_bttnDurationApply_clicked();

    void on_bttnReferenceApply_clicked();

    void on_bttnAutoTestStop_clicked();

    void on_cbComPort_currentIndexChanged(int index);

    void on_cbConndeviceName_currentIndexChanged(int index);

    void on_bttnManualTestStop_clicked();

public slots:
    void realtimePlot();
    void updateUI(int dataID);
    void sendData();
    void receiveData();
    void saveDataInFile();


private:
    Ui::MainWindow *ui;
    QTimer timer_plot;
    QTimer timer_sender;
    QTimer timer_ui;
    std::thread rx_thread_power;
    std::thread rx_thread_sensor;
    std::thread rx_thread_trigger;
    ofstream file;


    ComSerial serial[TOTALDEVICES];
    int fileNo;
    int dataID;
    QString deviceName[3] = {"Power","Trigger","Sensor"};
    QString devicePort[3] = {"COM1","COM2","COM3"};

private: signals:
    void updateData(int dataID);

};

#endif // MAINWINDOW_H

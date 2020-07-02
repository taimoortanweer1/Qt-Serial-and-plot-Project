/*
* mainwindow.cpp
*
*  Created on  : Dec 10, 2017
*  Author      : Vinay Divakar
*  website     : www.deeplyemebedded.org
*/


#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //init file no with 0
    fileNo = 0;
    //initialization of GUI objects in Mainwindow
    initGUI();
    dataID = 100; //dummy value at startup
    //this timer is started whenever START button is pressed. After timeout sendData is called which resends updated data.
    connect(&timer_sender, SIGNAL(timeout()),this,SLOT(sendData()));
    connect(this, SIGNAL(updateData(int)),this,SLOT(updateUI(int)));
    on_bttnRefreshSP_clicked();
    //this thread keeps receiving data from serial if available from RECEIVED DATA thread Handler
    rx_thread_power = std::thread(&MainWindow::receiveData,this);

}

MainWindow::~MainWindow()
{
    delete ui;
    rx_thread_power.detach();
    rx_thread_power.join();


}


void MainWindow::sendData()
{


    //data is sent in this given format

    //    uint32_t duration;
    //    uint16_t rpm;
    //    uint16_t current;
    //    uint16_t voltage;
    //    uint16_t temp;
    //    time_t ref_t;
    //    int16_t reference;
    //    uint32_t sumatoria;


    //transmit data over serial port to Sensor and Trigger

    serial[Sensor].transmitData();
    serial[Trigger].transmitData();
}

void MainWindow::receiveData()
{
    bool receiveFlag = false;
    while(1)
    {

        //        for testing
        //        sendData();
        //                for(uint8_t c: serial.txBuffer)
        //                {
        //                    receiveFlag = serial.receiveData(c);
        //                }

        //        serial.txBuffer.clear();

        //whenever all data is received true is returned


        //sensor is connected
        if(serial[Sensor].getConnected())
        {
            receiveFlag = serial[Sensor].receiveData();

            //data reception is completed
            if(receiveFlag == true)
            {
                serial[Sensor].extractData(serial[Sensor].dataBuffer);
                dataID = Sensor;
                emit updateData(dataID);
                receiveFlag = false;

            }

        }


        //trigger is connected
        if(serial[Trigger].getConnected())
        {


            receiveFlag = serial[Trigger].receiveData();

            //data reception is completed
            if(receiveFlag == true)
            {
                serial[Trigger].extractData(serial[Trigger].dataBuffer);
                dataID = Trigger;
                emit updateData(dataID);
                receiveFlag = false;
            }
        }


        //power is connected
        if(serial[Power].getConnected())
        {
            receiveFlag = serial[Power].receiveData();

            //data reception is completed
            if(receiveFlag == true)
            {
                serial[Power].extractData(serial[Power].dataBuffer);

                dataID = Power;
                emit updateData(dataID);
                receiveFlag = false;
            }

        }

        usleep(100000);
    }

}






void MainWindow::on_bttnRPMApply_clicked()
{
    //update rpm from user and send to serial
    serial[Trigger].txSendStruct.rpm = static_cast<uint16_t>(ui->spinRPM->value());

}

void MainWindow::initGUI()
{


    //current plot
    /* Add graph and set the curve line color to green */
    ui->CustomPlotCurrent->addGraph();
    ui->CustomPlotCurrent->graph(0)->setPen(QPen(Qt::red));
    ui->CustomPlotCurrent->graph(0)->setAntialiasedFill(false);

    /* Configure x-Axis as time in secs */
    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%s");
    ui->CustomPlotCurrent->xAxis->setTicker(timeTicker);
    ui->CustomPlotCurrent->axisRect()->setupFullAxesBox();


    /* Configure x and y-Axis to display Labels */
    ui->CustomPlotCurrent->xAxis->setTickLabelFont(QFont(QFont().family(),8));
    ui->CustomPlotCurrent->yAxis->setTickLabelFont(QFont(QFont().family(),8));
    ui->CustomPlotCurrent->xAxis->setLabel("Time(s)");
    ui->CustomPlotCurrent->yAxis->setLabel("Current (A)");

    /* Make top and right axis visible, but without ticks and label */
    ui->CustomPlotCurrent->xAxis2->setVisible(true);
    ui->CustomPlotCurrent->yAxis->setVisible(true);
    ui->CustomPlotCurrent->xAxis2->setTicks(false);
    ui->CustomPlotCurrent->yAxis2->setTicks(false);
    ui->CustomPlotCurrent->xAxis2->setTickLabels(false);
    ui->CustomPlotCurrent->yAxis2->setTickLabels(false);

    // voltage plot
    ui->CustomPlotVoltage->addGraph();
    ui->CustomPlotVoltage->graph(0)->setPen(QPen(Qt::red));
    ui->CustomPlotVoltage->graph(0)->setAntialiasedFill(false);

    /* Configure x-Axis as time in secs */
    timeTicker->setTimeFormat("%s");
    ui->CustomPlotVoltage->xAxis->setTicker(timeTicker);
    ui->CustomPlotVoltage->axisRect()->setupFullAxesBox();


    /* Configure x and y-Axis to display Labels */
    ui->CustomPlotVoltage->xAxis->setTickLabelFont(QFont(QFont().family(),8));
    ui->CustomPlotVoltage->yAxis->setTickLabelFont(QFont(QFont().family(),8));
    ui->CustomPlotVoltage->xAxis->setLabel("Time(s)");
    ui->CustomPlotVoltage->yAxis->setLabel("Voltage(V)");

    /* Make top and right axis visible, but without ticks and label */
    ui->CustomPlotVoltage->xAxis2->setVisible(true);
    ui->CustomPlotVoltage->yAxis->setVisible(true);
    ui->CustomPlotVoltage->xAxis2->setTicks(false);
    ui->CustomPlotVoltage->yAxis2->setTicks(false);
    ui->CustomPlotVoltage->xAxis2->setTickLabels(false);
    ui->CustomPlotVoltage->yAxis2->setTickLabels(false);

    // temperature plot
    ui->CustomPlotTemperature->addGraph();
    ui->CustomPlotTemperature->graph(0)->setPen(QPen(Qt::red));
    ui->CustomPlotTemperature->graph(0)->setAntialiasedFill(false);

    /* Configure x-Axis as time in secs */
    timeTicker->setTimeFormat("%s");
    ui->CustomPlotTemperature->xAxis->setTicker(timeTicker);
    ui->CustomPlotTemperature->axisRect()->setupFullAxesBox();


    /* Configure x and y-Axis to display Labels */
    ui->CustomPlotTemperature->xAxis->setTickLabelFont(QFont(QFont().family(),8));
    ui->CustomPlotTemperature->yAxis->setTickLabelFont(QFont(QFont().family(),8));
    ui->CustomPlotTemperature->xAxis->setLabel("Time(s)");
    ui->CustomPlotTemperature->yAxis->setLabel("Temp (C)");

    /* Make top and right axis visible, but without ticks and label */
    ui->CustomPlotTemperature->xAxis2->setVisible(true);
    ui->CustomPlotTemperature->yAxis->setVisible(true);
    ui->CustomPlotTemperature->xAxis2->setTicks(false);
    ui->CustomPlotTemperature->yAxis2->setTicks(false);
    ui->CustomPlotTemperature->xAxis2->setTickLabels(false);
    ui->CustomPlotTemperature->yAxis2->setTickLabels(false);

    //RMS plot
    ui->CustomPlotRMS->addGraph();
    ui->CustomPlotRMS->graph(0)->setPen(QPen(Qt::red));
    ui->CustomPlotRMS->graph(0)->setAntialiasedFill(false);

    timeTicker->setTimeFormat("%s");
    ui->CustomPlotRMS->xAxis->setTicker(timeTicker);
    ui->CustomPlotRMS->axisRect()->setupFullAxesBox();
    ui->CustomPlotRMS->xAxis->setTickLabelFont(QFont(QFont().family(),8));
    ui->CustomPlotRMS->yAxis->setTickLabelFont(QFont(QFont().family(),8));
    ui->CustomPlotRMS->xAxis->setLabel("Time(s)");
    ui->CustomPlotRMS->yAxis->setLabel("RPM");
    ui->CustomPlotRMS->xAxis2->setVisible(true);
    ui->CustomPlotRMS->yAxis->setVisible(true);
    ui->CustomPlotRMS->xAxis2->setTicks(false);
    ui->CustomPlotRMS->yAxis2->setTicks(false);
    ui->CustomPlotRMS->xAxis2->setTickLabels(false);
    ui->CustomPlotRMS->yAxis2->setTickLabels(false);

    /* Set up and initialize the graph plotting timer */
    //connect(&timer_plot, SIGNAL(timeout()),this,SLOT(realtimePlot()));


}

void MainWindow::realtimePlot()
{
    //    static QTime time(QTime::currentTime());
    //    double key = time.elapsed()/1000.0;
    //    static double lastPointKey = 0;

    static QTime time(QTime::currentTime());
    double key = time.elapsed()/1000.0;
    static double lastPointKey = 0;

    if(key - lastPointKey > 0.002)
    {
        ui->CustomPlotCurrent->graph(0)->addData(key, serial[Sensor].rxStruct.current);
        ui->CustomPlotVoltage->graph(0)->addData(key, serial[Sensor].rxStruct.voltage);
        ui->CustomPlotTemperature->graph(0)->addData(key, serial[Sensor].rxStruct.temp);
        ui->CustomPlotRMS->graph(0)->addData(key, serial[Sensor].rxStruct.rpm);

        lastPointKey = key;
        saveDataInFile();
    }

    /* make key axis range scroll right with the data at a constant range of 8. */
    ui->CustomPlotCurrent->graph(0)->rescaleValueAxis();
    ui->CustomPlotCurrent->xAxis->setRange(key, 60, Qt::AlignRight);
    ui->CustomPlotCurrent->replot();

    ui->CustomPlotVoltage->graph(0)->rescaleValueAxis();
    ui->CustomPlotVoltage->xAxis->setRange(key, 60, Qt::AlignRight);
    ui->CustomPlotVoltage->replot();

    ui->CustomPlotTemperature->graph(0)->rescaleValueAxis();
    ui->CustomPlotTemperature->xAxis->setRange(key, 60, Qt::AlignRight);
    ui->CustomPlotTemperature->replot();

    ui->CustomPlotRMS->graph(0)->rescaleValueAxis();
    ui->CustomPlotRMS->xAxis->setRange(key, 60, Qt::AlignRight);
    ui->CustomPlotRMS->replot();


}

void MainWindow::saveDataInFile()
{

    //writing dta in file
    file << serial[Sensor].rxStruct.rpm;
    file << ",";
    file << serial[Sensor].rxStruct.current;
    file << ",";
    file << serial[Sensor].rxStruct.voltage;
    file << ",";
    file << serial[Sensor].rxStruct.temp;
    file << ",";
    file << serial[Sensor].rxStruct.ref_t;
    file << ",";
    file << serial[Sensor].rxStruct.reference;
    file << ",";
    file << serial[Sensor].rxStruct.duration;
    file << "\n";

}


void MainWindow::on_bttnConnectSP_clicked()
{
    QMessageBox msg ;

    //bool chk = serial.initSerial(ui->lineEdit->text());
    bool chk = serial[ui->cbComPort->currentIndex()].initSerial(devicePort[ui->cbComPort->currentIndex()]);

    if(chk)
    {
        serial[ui->cbComPort->currentIndex()].setDeviceName(deviceName[ui->cbComPort->currentIndex()]);
        msg.setText("Serial Port Connected");
        msg.exec();
    }
    else
    {
        msg.setText("Serial Port Connection Failed | Already Connected | Not Available");
        msg.exec();
    }
}

void MainWindow::on_bttnDisconnectSP_clicked()
{
    QMessageBox msg ;

    //bool chk = serial.initSerial(ui->lineEdit->text());
    serial[ui->cbComPort->currentIndex()].DisSerial();

    msg.setText("Serial Port Connected");
    msg.exec();

}

void MainWindow::on_bttnRefreshSP_clicked()
{
    ui->cbComPort->clear();

    //filling up available ports
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos)
        ui->cbComPort->addItem(info.portName());
}

void MainWindow::on_bttnInit_clicked()
{

    QString selectedDevice = ui->cbInitdeviceName->currentText();

    for(int i = 0; i < 3; i++)
    {
        QString devName = serial[i].getDeviceName();
        if(devName == selectedDevice)
        {
            serial[i].txSendStruct.code = STARTUP_CODE;
            serial[i].transmitData();
        }
    }
}

void MainWindow::on_bttnCheckStatus_clicked()
{

    QString selectedDevice = ui->cbStatusdeviceName->currentText();

    for(int i = 0; i < 3; i++)
    {
        QString devName = serial[i].getDeviceName();
        if(devName == selectedDevice)
        {
            serial[i].txSendStruct.code = READY_STATUS_CODE;
            serial[i].transmitData();
        }
    }
}

void MainWindow::on_bttnAutoTestStart_clicked()
{

    //naming format

    if(file.is_open())
    {
        file.flush();
        file.close();

    }
    char filename[5] = "data";

    fileNo++;
    std::string s = std::to_string(fileNo);
    char const *no = s.c_str();

    char ext[5] = ".csv";

    strcat(filename,no);

    strcat(filename,ext);



    file.open(filename);



    file << "rpm,current,voltage,temp,ref_t,Reference,duration";

    //plot is updated from 5ms
    timer_plot.start(5);

    //new data is sent after 1000ms
    timer_sender.start(1000);

    //send data from user to arduino via serial
    serial[Trigger].txSendStruct.duration = ui->spinDuration->value();
    serial[Trigger].txSendStruct.rpm = ui->spinRPM->value();
    serial[Trigger].txSendStruct.reference = ui->spinReference->value();

    serial[Trigger].txSendStruct.code = AUTO_CODE_TRIGGER;
    serial[Trigger].transmitData();

    serial[Sensor].txSendStruct.code = AUTO_CODE_SENSOR;
    serial[Sensor].transmitData();

    //data send fucntion
    sendData();

}

void MainWindow::on_bttnManualTestStart_clicked()
{
    serial[Trigger].txSendStruct.code = MANUAL_CODE_TRIGGER;
    serial[Trigger].transmitData();

    serial[Sensor].txSendStruct.code = MANUAL_CODE_SENSOR;
    serial[Sensor].transmitData();


    //plot is updated from 5ms
    timer_plot.start(5);

    //new data is sent after 1000ms
    timer_sender.start(1000);



}

void MainWindow::on_bttnDurationApply_clicked()
{
    serial[Trigger].txSendStruct.duration = static_cast<uint16_t>(ui->spinDuration->value());
}

void MainWindow::on_bttnReferenceApply_clicked()
{
    serial[Trigger].txSendStruct.reference = static_cast<uint16_t>(ui->spinReference->value());
}

void MainWindow::on_bttnAutoTestStop_clicked()
{
    //stop plot and data send by sending stop
    timer_plot.stop();
    timer_sender.stop();
    serial[Trigger].txSendStruct.code = STOP;
    serial[Sensor].txSendStruct.code = STOP;

    sendData();

    //close curretn file
    file.flush();
    file.close();
}

void MainWindow::on_cbComPort_currentIndexChanged(int index)
{
    ui->cbConndeviceName->setCurrentIndex(index);
}

void MainWindow::on_cbConndeviceName_currentIndexChanged(int index)
{
    ui->cbComPort->setCurrentIndex(index);
}



void MainWindow::on_bttnManualTestStop_clicked()
{
    //stop plot and data send by sending stop
    timer_plot.stop();
    timer_sender.stop();
    serial[Trigger].txSendStruct.code = STOP;
    serial[Sensor].txSendStruct.code = STOP;

    sendData();

    //close curretn file
    file.flush();
    file.close();
}


void MainWindow::updateUI(int dataID)
{

    if(dataID == Sensor)
    {
        //sensors values are updating
        //if(serial[Sensor].getGetValues() == AUTO_CODE_SENSOR)

            try {

                //                for (auto c:serial.dataBuffer) {
                //                    cout << c << " ";
                //                    cout << endl;
                //                }

                //parse dta from received data
                realtimePlot();

                //display in gui
                ui->lblRPM->setText(QString::number(serial[Sensor].rxStruct.rpm));
                ui->lblCurrent->setText(QString::number(serial[Sensor].rxStruct.current));
                ui->lblVoltage->setText(QString::number(serial[Sensor].rxStruct.voltage));
                ui->lblTemperature->setText(QString::number(serial[Sensor].rxStruct.temp));
                ui->lblRefT->setText(QString::number(serial[Sensor].rxStruct.ref_t));
                ui->lblReference->setText(QString::number(serial[Sensor].rxStruct.reference));
                ui->lblDuration->setText(QString::number(serial[Sensor].rxStruct.duration));

            }
            catch (std::exception ex) {
                std::cout << ex.what() <<endl;
            }


        //sensor ready status check
        if(serial[Sensor].getReady() == READY_YES)
        {
            ui->lblStatusSensor->setStyleSheet("color: rgb(255, 255, 255);background-color: rgb(0, 255, 0);");
        }
        else
        {
            ui->lblStatusSensor->setStyleSheet("color: rgb(255, 255, 255);background-color: rgb(255, 0, 0);");
        }

        //sensor initialization status check
        if(serial[Sensor].getInit_ret_code() == OK_CODE)
        {
            ui->lblInitSensor->setStyleSheet("color: rgb(255, 255, 255);background-color: rgb(0, 255, 0);");
        }
        else if(serial[Sensor].getInit_ret_code() == FAIL_CODE)
        {
            ui->lblInitSensor->setStyleSheet("color: rgb(255, 255, 255);background-color: rgb(255, 0, 0);");
        }
    }

    if(dataID == Power)
    {
        //Trigger ready status check
        if(serial[Power].getReady() == READY_YES)
        {
            ui->lblStatusPower->setStyleSheet("color: rgb(255, 255, 255);background-color: rgb(0, 255, 0);");
        }
        else
        {
            ui->lblStatusPower->setStyleSheet("color: rgb(255, 255, 255);background-color: rgb(255, 0, 0);");
        }

        //sensor initialization status check
        if(serial[Power].getInit_ret_code() == OK_CODE)
        {
            ui->lblInitPower->setStyleSheet("color: rgb(255, 255, 255);background-color: rgb(0, 255, 0);");
        }
        else if(serial[Power].getInit_ret_code() == FAIL_CODE)
        {
            ui->lblInitPower->setStyleSheet("color: rgb(255, 255, 255);background-color: rgb(255, 0, 0);");
        }

    }

    if(dataID == Trigger)
    {
        //Trigger ready status check
        if(serial[Trigger].getReady() == READY_YES)
        {
            ui->lblStatusTrigger->setStyleSheet("color: rgb(255, 255, 255);background-color: rgb(0, 255, 0);");
        }
        else
        {
            ui->lblStatusTrigger->setStyleSheet("color: rgb(255, 255, 255);background-color: rgb(255, 0, 0);");
        }

        //sensor initialization status check
        if(serial[Trigger].getInit_ret_code() == OK_CODE)
        {
            ui->lblInitTrigger->setStyleSheet("color: rgb(255, 255, 255);background-color: rgb(0, 255, 0);");
        }
        else if(serial[Trigger].getInit_ret_code() == FAIL_CODE)
        {
            ui->lblInitTrigger->setStyleSheet("color: rgb(255, 255, 255);background-color: rgb(255, 0, 0);");
        }

    }
}

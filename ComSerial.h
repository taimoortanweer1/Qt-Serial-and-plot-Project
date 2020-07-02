#ifndef COMSERIAL_H
#define COMSERIAL_H

#include <QSerialPort>
#include <QString>
#include "iostream"
#include "vector"

#define STX1 '\x0AA'
#define ETX1 '\x0FF'

//codes
//I, for start J  for change in the value of RPM, T  for time and , P for stop
#define START 'I'
#define RPM   'J'
#define TIME  'T'
#define STOP  'P'

#define STARTUP_CODE 'S'
#define READY_STATUS_CODE 'R'
#define READY_YES 'Y'
#define AUTO_CODE_SENSOR 'V'
#define AUTO_CODE_TRIGGER 'I'

#define MANUAL_CODE_SENSOR 'V'
#define MANUAL_CODE_TRIGGER 'M'

#define OK_CODE '1'
#define FAIL_CODE '2'

#define TOTALDEVICES 3
enum devices
{
  Power = 0,
  Trigger,
  Sensor
};

enum rxStates
{
    Header = 0,
    DataState,
    Tailer

};

using namespace std;


class ComSerial
{
public:
    ComSerial();
    ~ComSerial();
    bool initSerial(QString portname);
    void DisSerial();
    void transmitData();
    bool receiveData();
    bool receiveData(uint8_t rx);
    void extractData(vector<uint8_t>data);

private:
    QSerialPort *port;
    vector <uint8_t> rxbuffer;
    uint8_t recxState = Header;
    QString deviceName;
    bool connected;
    char ready;
    char init_ret_code;
    char getValues;

    const uint8_t STX = 0xAA;
    const uint8_t ETX = 0xFF;

public:
    vector <uint8_t> txBuffer;
    vector <uint8_t> dataBuffer;


    struct DataMessage
    {
    uint32_t duration;
    uint16_t rpm;
    uint16_t current;
    uint16_t voltage;
    uint16_t temp;
    time_t ref_t;
    int16_t reference;

    uint32_t sumatoria;
    char code;

    DataMessage()
    {
        duration = 0;
        rpm = 0;
        current = 0;
        voltage = 0;
        temp = 0;
        ref_t = 0;
        reference = 0;
        sumatoria = 0;
        code = 0;
    }

    };

    DataMessage txSendStruct;
    DataMessage rxStruct;
    QString getDeviceName() const;
    void setDeviceName(const QString &value);
    bool getConnected() const;
    void setConnected(bool value);

    char getReady() const;
    char getInit_ret_code() const;
    char getGetValues() const;
};

#endif // COMSERIAL_H

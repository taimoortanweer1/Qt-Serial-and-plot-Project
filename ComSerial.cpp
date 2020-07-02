#include "ComSerial.h"

#include "string.h"
#include "math.h"
#include <numeric>
ComSerial::ComSerial()
{
    port = new QSerialPort();
    connected = false;
    ready = '0';
    init_ret_code = '0';
    getValues = '0';
}

ComSerial::~ComSerial()
{
    delete port;
}

bool ComSerial::initSerial(QString portname)
{

    //ttyS0 is used : means comport 1 is used
    //    char buffer[40];
    //    char sp_name[10] = "/dev/";
    //    strcat(sp_name,"portname");
    //    sprintf(buffer,"sudo chown -R $USERNAME /dev/ttyS0");
    //    system(buffer);


    port->setPortName(portname);

    port->setBaudRate(QSerialPort::Baud9600);
    port->setDataBits(QSerialPort::Data8);
    port->setParity(QSerialPort::NoParity);
    port->setStopBits(QSerialPort::OneStop);   
    if (port->open(QIODevice::ReadWrite)) {
        connected = true;
        return true;
    }
    else {
        connected = false;
        return false;
    }
}

void ComSerial::DisSerial()
{
    port->close();
    connected = false;

}
void ComSerial::transmitData()
{

    //first of all current data is pushed in txBuffer and then this vector is transmitted over serial

    //    uint32_t duration;
    //    uint16_t rpm;
    //    uint16_t current;
    //    uint16_t voltage;
    //    uint16_t temp;
    //    time_t ref_t;
    //    int16_t reference;
    //    uint32_t sumatoria;


    //pushing all data bytes in a vector for sending
    txBuffer.push_back(STX);

    txBuffer.push_back(txSendStruct.duration);
    txBuffer.push_back(txSendStruct.duration >> 8);
    txBuffer.push_back(txSendStruct.duration >> 16);
    txBuffer.push_back(txSendStruct.duration >> 24);

    txBuffer.push_back(txSendStruct.rpm);
    txBuffer.push_back(txSendStruct.rpm >> 8);

    txBuffer.push_back(txSendStruct.current);
    txBuffer.push_back(txSendStruct.current >> 8);

    txBuffer.push_back(txSendStruct.voltage);
    txBuffer.push_back(txSendStruct.voltage >> 8);

    txBuffer.push_back(txSendStruct.temp);
    txBuffer.push_back(txSendStruct.temp >> 8);


    txBuffer.push_back(txSendStruct.ref_t);
    txBuffer.push_back(txSendStruct.ref_t >> 8);
    txBuffer.push_back(txSendStruct.ref_t >> 16);
    txBuffer.push_back(txSendStruct.ref_t >> 24);


    txBuffer.push_back(txSendStruct.reference);
    txBuffer.push_back(txSendStruct.reference >> 8);


    //for four byte sumatoria -- calculate sumotoria
    txSendStruct.sumatoria = txSendStruct.duration + txSendStruct.rpm + txSendStruct.current +
            txSendStruct.voltage + txSendStruct.temp  + txSendStruct.reference;


    txBuffer.push_back(txSendStruct.sumatoria);
    txBuffer.push_back(txSendStruct.sumatoria >> 8);
    txBuffer.push_back(txSendStruct.sumatoria >> 16);
    txBuffer.push_back(txSendStruct.sumatoria >> 24);

    txBuffer.push_back(txSendStruct.code);

    txBuffer.push_back(ETX);

    //    for (auto c:txBuffer)
    //        cout << (int)c << " ";
    //    cout << endl;

    //data write fucntion

    QByteArray arr;

    for (auto c:txBuffer)
        arr.append(c);

    port->write(arr);

    //clear buffer for next sending
    txBuffer.clear();

}

bool ComSerial::receiveData()
{
    //this function receives data one by one from serial and then check for current state of data.
    //at the end calculates checksum and compares it to check integrity of data.

    port->waitForReadyRead(100);

    QByteArray arr = port->readAll();
    rxbuffer.clear();
    int count = 0;

    uint8_t rxbyte;
    bool ret_val = false;

    while(count < arr.size())
    {
        rxbyte = arr.at(count);
        count++;

        switch (recxState)
        {
        case Header:

            if(rxbyte == STX)
            {
                recxState = DataState;
                rxbuffer.push_back(rxbyte);
            }
            else
            {
                recxState = Header;
                rxbuffer.clear();
            }
            break;

        case DataState:

            if(rxbyte == ETX)
            { //last byte is received

                rxbuffer.push_back(rxbyte);

                //calcuting checksum for received data
                uint32_t chksum = 0;

                for (int i=1 ;i<13; i++) {
                    chksum = chksum + rxbuffer[i];
                }
                chksum = chksum + rxbuffer[17] + rxbuffer[18];

                //checksum that is sent with received data
                uint calc_chksum = (rxbuffer[19]) + (rxbuffer[20] << 8) + (rxbuffer[21] << 16) + (rxbuffer[22] << 24);

                //means data is correct
                if((rxbuffer[0] == STX) && (calc_chksum == chksum))
                {
                    recxState = Header;
                    dataBuffer = rxbuffer;
                    rxbuffer.clear();
                    ret_val = true;
                    return ret_val;
                }
            }
            else
            {
                rxbuffer.push_back(rxbyte);
            }
            break;

        default:
            recxState = Header;
            rxbuffer.clear();

        }

    }
}
bool ComSerial::receiveData(uint8_t rx)
{
    uint8_t rxbyte = rx;
    bool ret_val = false;

    switch (recxState)
    {
    case Header:

        if(rxbyte == STX)
        {
            recxState = DataState;
            rxbuffer.push_back(rxbyte);
        }
        else
        {
            recxState = Header;
            rxbuffer.clear();
        }
        break;

    case DataState:

        if(rxbyte == ETX)
        {

            rxbuffer.push_back(rxbyte);

            //calcuting checksum for received data
            uint32_t chksum = 0;

            for (int i=1 ;i<13; i++) {
                chksum = chksum + rxbuffer[i];
            }
            chksum = chksum + rxbuffer[17] + rxbuffer[18];

            //checksum that is sent with received data
            uint calc_chksum = (rxbuffer[19]) + (rxbuffer[20] << 8) + (rxbuffer[21] << 16) + (rxbuffer[22] << 24);

            //means data is correct
            if((rxbuffer[0] == STX) && (calc_chksum == chksum))
            {
                recxState = Header;
                dataBuffer = rxbuffer;
                rxbuffer.clear();
                ret_val = true;
                return ret_val;
            }

        }
        else
        {
            rxbuffer.push_back(rxbyte);
        }
        break;

    default:
        recxState = Header;
        rxbuffer.clear();

    }
}

void ComSerial::extractData(vector<uint8_t>data)
{


//    uint32_t duration;
//    uint16_t rpm;
//    uint16_t current;
//    uint16_t voltage;
//    uint16_t temp;
//    time_t ref_t;
//    int16_t reference;

//    uint32_t sumatoria;
//    char code;

    //extract data from received bytes and save it in rxStruct  for use
    int byteNo = 1;
    rxStruct.duration = data[byteNo++] | data[byteNo++] << 8 | data[byteNo++] << 16 | data[byteNo++] << 24;
    rxStruct.rpm = data[byteNo++]| data[byteNo++]<< 8;
    rxStruct.current = data[byteNo++] | data[byteNo++] << 8;
    rxStruct.voltage = data[byteNo++] | data[byteNo++] << 8;
    rxStruct.temp = data[byteNo++]| data[byteNo++] << 8;
    rxStruct.ref_t = data[byteNo++]| data[byteNo++] << 8 | data[byteNo++] << 16 | data[byteNo++] << 24;
    rxStruct.reference = data[byteNo++]| data[byteNo++] << 8;
    byteNo = byteNo + 4;
    rxStruct.code = data[byteNo];

    if(rxStruct.code == OK_CODE || rxStruct.code == FAIL_CODE)
    {
        init_ret_code = rxStruct.code;
    }

    if(rxStruct.code == READY_YES)
    {
        ready = rxStruct.code;
    }

    if(rxStruct.code == AUTO_CODE_SENSOR)
    {
        getValues = AUTO_CODE_SENSOR;
    }

}

char ComSerial::getGetValues() const
{
    return getValues;
}

char ComSerial::getInit_ret_code() const
{
    return init_ret_code;
}

char ComSerial::getReady() const
{
    return ready;
}



bool ComSerial::getConnected() const
{
    return connected;
}

void ComSerial::setConnected(bool value)
{
    connected = value;
}


void ComSerial::setDeviceName(const QString &value)
{
    deviceName = value;
}

QString ComSerial::getDeviceName() const
{
    return deviceName;
}

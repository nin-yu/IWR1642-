#include "vitalsigns.h"

VitalSigns::VitalSigns()
{
    m_indexBuffer = 0;
}

// ****************************************判断串口是否打开****************************************** //
bool VitalSigns::InitialSerialPort(int srcT)
{
    // Find available COM ports on the computer
    QString dataPortNum = "COM4";
    QString userPortNum = "COM3";

    bool isFind, isFind1, isFind2;
    isFind = isFind1 = isFind2 = false;
    QString portNum;
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts())
    {
        portNum = serialPortInfo.portName();
        if (portNum==dataPortNum)
            isFind1 = true;
        if (portNum==userPortNum)
            isFind2 = true;

    }
    if (isFind1 && isFind2)
    {
        m_serialWrite = new QSerialPort();
        m_serialRead = new QSerialPort();  // 从这里开始定义接收端口
        bool dataPort_Connected = serialDataPortConfig(m_serialRead, 921600, dataPortNum);  // 配置串口
        bool userPort_Connected = serialUserPortConfig(m_serialWrite, 115200, userPortNum);
        if (dataPort_Connected && userPort_Connected)
            return 1;
        else
            return 0;
    }
    else
        return 0;
}

// *******************************************配置串口********************************************* //
bool VitalSigns::serialUserPortConfig(QSerialPort *serial, qint32 baudRate, QString userPortNum )
{
    bool FlagSerialPort_Connected;
    serial->setPortName(userPortNum);

    if(serial->open(QIODevice::ReadWrite))
      {
        FlagSerialPort_Connected = 1;
      }
    else
      {
        FlagSerialPort_Connected = 0;
        return FlagSerialPort_Connected;
      }
    serial->setBaudRate(baudRate);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    return FlagSerialPort_Connected;
}

bool VitalSigns::serialDataPortConfig(QSerialPort *serial, qint32 baudRate, QString dataPortNum )
{
    bool FlagSerialPort_Connected;
    serial->setPortName(dataPortNum);

    if(serial->open(QIODevice::ReadOnly) )
    {
        FlagSerialPort_Connected = 1;
    }
    else
    {
        FlagSerialPort_Connected = 0;
        return FlagSerialPort_Connected;
    }
    serial->setBaudRate(baudRate);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    return FlagSerialPort_Connected;
}

int VitalSigns::nextPower2(int num)
{
    int power = 1;
    while(power < num)
        power*=2;
    return power;
}

bool VitalSigns::OpenSerialPort(QString srcFile)
{
    std::cout<<"config file name : "<< srcFile.toStdString()<<std::endl;

    // 读取配置文件
    //filenameText.append("/profiles/profile_2d_VitalSigns_20fps_Front.cfg");
    //qDebug() << "Configuration File Path is %s/n"<<filenameText;

    //thresh_breath = 10;
    //thresh_heart = 0.1;
    //BREATHING_PLOT_MAX_YAXIS = 1.0;
    //HEART_PLOT_MAX_YAXIS = 0.5;

    // 按行读取并进行写入IWR1642中
    QFile infile(srcFile);
    if (infile.open(QIODevice::ReadWrite))
    {
        QTextStream inStream(&infile);
        while (!inStream.atEnd())
        {
            QString line = inStream.readLine();  // Read a line from the input Text File
            //qDebug() << line;
            m_serialWrite->write(line.toUtf8().constData());
            m_serialWrite->write("\n");
            m_serialWrite->waitForBytesWritten(10000);
            Sleep(100);
        }
        infile.close();
    }
    else
    {
        std::cout<<"ReadWrite config file fail!------"<<std::endl;
    }
    // 解析配置文件
    if (infile.open(QIODevice::ReadWrite))
    {
        QStringList listArgs;
        QString tempString;
        QTextStream inStream(&infile);
        while (!inStream.atEnd())
        {
            QString line = inStream.readLine();  // Read a line from the input Text File
            //qDebug() << line;
            if (line.contains("vitalSignsCfg", Qt::CaseInsensitive) )
            {
                  listArgs = line.split(QRegExp("\\s+"));
                  tempString = listArgs.at(1);
                  m_demoParams.rangeStartMeters = tempString.toFloat();
                 // qDebug() << m_demoParams.rangeStartMeters;

                  listArgs = line.split(QRegExp("\\s+"));
                  tempString = listArgs.at(2);
                  m_demoParams.rangeEndMeters = tempString.toFloat();
                 // qDebug() << m_demoParams.rangeEndMeters;

                  listArgs = line.split(QRegExp("\\s+"));
                  tempString = listArgs.at(5);
                  m_demoParams.AGC_thresh = tempString.toFloat();
                 // qDebug() << m_demoParams.AGC_thresh;
            }

            if (line.contains("profileCfg", Qt::CaseInsensitive) )    // 解析雷达发射的参数，频率，带宽，斜坡频率等
            {

                listArgs = line.split(QRegExp("\\s+"));
                tempString = listArgs.at(2);
                m_demoParams.stratFreq_GHz = tempString.toFloat();

                listArgs = line.split(QRegExp("\\s+"));
                tempString = listArgs.at(8);
                m_demoParams.freqSlope_MHZ_us = tempString.toFloat();

                listArgs = line.split(QRegExp("\\s+"));
                tempString = listArgs.at(10);
                m_demoParams.numSamplesChirp = tempString.toFloat();

                listArgs = line.split(QRegExp("\\s+"));
                tempString = listArgs.at(11);
                m_demoParams.samplingRateADC_ksps = tempString.toInt();
            }
        }
        infile.close();

        // Compute the Derived Configuration Parameters         // 根据文件中的配置参数，计算出更进一步的参数
        m_demoParams.chirpDuration_us = 1e3*m_demoParams.numSamplesChirp/m_demoParams.samplingRateADC_ksps;
        //qDebug() << "Chirp Duration in us is :" << m_demoParams.chirpDuration_us;

        m_demoParams.chirpBandwidth_kHz = (m_demoParams.freqSlope_MHZ_us)*(m_demoParams.chirpDuration_us);
        //qDebug() << "Chirp Bandwidth in kHz is : "<<m_demoParams.chirpBandwidth_kHz;

        float numTemp = (m_demoParams.chirpDuration_us)*(m_demoParams.samplingRateADC_ksps)*(3e8);
        float denTemp =  2*(m_demoParams.chirpBandwidth_kHz);
        m_demoParams.rangeMaximum_meters = numTemp/(denTemp*1e9);
       // qDebug() << "Maximum Range in Meters is :"<< m_demoParams.rangeMaximum_meters;

        m_demoParams.rangeFFTsize = nextPower2(m_demoParams.numSamplesChirp);
       // qDebug() << "Range-FFT size is :"<< m_demoParams.rangeFFTsize;

        m_demoParams.rangeBinSize_meters = m_demoParams.rangeMaximum_meters/m_demoParams.rangeFFTsize;
       // qDebug() << "Range-Bin size is :"<< m_demoParams.rangeBinSize_meters;

        m_demoParams.rangeBinStart_index = m_demoParams.rangeStartMeters/m_demoParams.rangeBinSize_meters;
        m_demoParams.rangeBinEnd_index   = m_demoParams.rangeEndMeters/m_demoParams.rangeBinSize_meters;
        //qDebug() << "Range-Bin Start Index is :"<< m_demoParams.rangeBinStart_index;
       // qDebug() << "Range-Bin End Index is :"  << m_demoParams.rangeBinEnd_index;

        m_demoParams.numRangeBinProcessed = m_demoParams.rangeBinEnd_index - m_demoParams.rangeBinStart_index + 1;

        // Calculate the Payload size

        #ifdef MMWAVE_PLATFORM_1642
            m_demoParams.totalPayloadSize_bytes = LENGTH_HEADER_BYTES;
            m_demoParams.totalPayloadSize_bytes += LENGTH_TLV_MESSAGE_HEADER_BYTES + (4*m_demoParams.numRangeBinProcessed);
            m_demoParams.totalPayloadSize_bytes += LENGTH_TLV_MESSAGE_HEADER_BYTES +  LENGTH_DEBUG_DATA_OUT_BYTES;
            std::cout << "Total Payload size from the UART is :"<< m_demoParams.totalPayloadSize_bytes;

            if ((m_demoParams.totalPayloadSize_bytes % MMWDEMO_OUTPUT_MSG_SEGMENT_LEN) != 0)
            {
                int paddingFactor = ceil( (float) m_demoParams.totalPayloadSize_bytes/ (float) MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
                std::cout << "Padding Factor is :"<<paddingFactor;

                m_demoParams.totalPayloadSize_bytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN*paddingFactor;
            }
        #else
            m_demoParams.totalPayloadSize_bytes = LENGTH_MAGIC_WORD_BYTES + 4 * (m_demoParams.numRangeBinProcessed +  LENGTH_DEBUG_DATA_OUT_FLOAT);
        #endif
            std::cout << "Total Payload size from the UART is :"<< m_demoParams.totalPayloadSize_bytes;
            m_demoParams.totalPayloadSize_nibbles = 2 * m_demoParams.totalPayloadSize_bytes;
       }
    return 1;
}

float  VitalSigns::parseValueFloat(QByteArray data, int valuePos, int valueSize)
{
    bool ok;
    QByteArray parseData;
    parseData = data.mid(valuePos,valueSize);
    QString strParseData = parseData;
    quint32 temp_int = strParseData.toUInt(&ok,16);
    temp_int =  qToBigEndian(temp_int);        // Convert to Big-Endian
    float parseValueOut;
    parseValueOut = *reinterpret_cast<float*>(&temp_int); // cast to Float
    return parseValueOut;
}

qint16  VitalSigns::parseValueInt16(QByteArray data, int valuePos, int valueSize)
{
    bool ok;
    QByteArray parseData;
    parseData = data.mid(valuePos, valueSize);
    QString strParseData = parseData;
    qint16 parseValueOut = strParseData.toInt(&ok,16);
    parseValueOut =  qToBigEndian(parseValueOut);        // Convert to Big-Endian
    return parseValueOut;
}

float VitalSigns::GetSerialData()
{
    QByteArray dataSerial;
    //qDebug()<<"byteAvailable:"<<serialRead->bytesAvailable();
    if (m_serialRead->waitForReadyRead(1000))    // 如果有readyread信号就直接读取数据，否则等待1000ms后发生timeout再读取数据。缺少这句的话C#调用就会出问题，QT调用就没事
    {
        dataSerial = m_serialRead->readAll().toHex();    // 读取串口数据
    }

    int dataSize = dataSerial.size();
    //qDebug() << "dataSize: "<< dataSize;
    m_dataBuffer = m_dataBuffer.append(dataSerial);
    m_indexBuffer = m_indexBuffer + dataSize;

    QByteArray data;
    bool MagicOk;

    while (m_dataBuffer.size() >= m_demoParams.totalPayloadSize_nibbles)
    {

        //QElapsedTimer timer;       // For computing the time required for the operations
        //timer.start();

        QByteArray searchStr("0201040306050807");  // Search String Array 数据包帧头标识
        int dataStartIndex = m_dataBuffer.indexOf(searchStr);

        if (dataStartIndex == -1)
        {
            MagicOk = 0;

            break;
        }
        else
        {
            data       = m_dataBuffer.mid(dataStartIndex, m_demoParams.totalPayloadSize_nibbles /*TOTAL_PAYLOAD_SIZE_NIBBLES*/);
            m_dataBuffer = m_dataBuffer.remove(dataStartIndex, m_demoParams.totalPayloadSize_nibbles /*TOTAL_PAYLOAD_SIZE_NIBBLES*/);

            m_dataBuffer = m_dataBuffer.remove(0, dataStartIndex);
            std::cout<<"m_dataBuffer size after removal is "<<m_dataBuffer.size()<<std::endl;

            // Check if all the data has been recieved succesfully
            if (data.size() >= m_demoParams.totalPayloadSize_nibbles)
            {
                MagicOk = 1;
            }
            else
            {
                MagicOk = 0;
                std::cout<< data.size();
            }
         }

        if (MagicOk == 1)
        {
           float phaseWfm_Out  = parseValueFloat(data, INDEX_IN_DATA_PHASE, 8); // Phase Waveforms
           //phaseWfm_Out是胸腔位移的相位幅值，需要乘上一个常量，常量的计算公式：\Delta \phi_{b}=\frac{4 \pi}{\lambda} \Delta R 。\Delta \phi_{b}是相位变化，\Delta R为心脏或者胸腔引起的微小位移变化
           phaseWfm_Out = phaseWfm_Out * 0.3100421; // 单位：mm

           return phaseWfm_Out;
        }
    }

}


void VitalSigns::GetSerialData(float &phaseWfm_Out, float &positionmax)
{
    QByteArray dataSerial;
    //qDebug()<<"byteAvailable:"<<serialRead->bytesAvailable();
    if (m_serialRead->waitForReadyRead(1000))    // 如果有readyread信号就直接读取数据，否则等待1000ms后发生timeout再读取数据。缺少这句的话C#调用就会出问题，QT调用就没事
    {
        dataSerial = m_serialRead->readAll().toHex();    // 读取串口数据
    }

    int dataSize = dataSerial.size();
    std::cout<< "dataSize: "<< dataSize<<std::endl;
    m_dataBuffer = m_dataBuffer.append(dataSerial);
    m_indexBuffer = m_indexBuffer + dataSize;

    QByteArray data;

    bool MagicOk;

    while (m_dataBuffer.size() >= m_demoParams.totalPayloadSize_nibbles)
    {
        //QElapsedTimer timer;       // For computing the time required for the operations
        //timer.start();

        QByteArray searchStr("0201040306050807");  // Search String Array 数据包帧头标识
        int dataStartIndex = m_dataBuffer.indexOf(searchStr);

        if (dataStartIndex == -1)
        {
            MagicOk = 0;
            break;
        }
        else
        {
            data = m_dataBuffer.mid(dataStartIndex, m_demoParams.totalPayloadSize_nibbles /*TOTAL_PAYLOAD_SIZE_NIBBLES*/);
            m_dataBuffer = m_dataBuffer.remove(dataStartIndex, m_demoParams.totalPayloadSize_nibbles /*TOTAL_PAYLOAD_SIZE_NIBBLES*/);

            m_dataBuffer = m_dataBuffer.remove(0, dataStartIndex);
            //qDebug()<<"dataBuffer size after removal is "<<m_dataBuffer.size();

            // Check if all the data has been recieved succesfully
            if (data.size() >= m_demoParams.totalPayloadSize_nibbles)
            {
                MagicOk = 1;
            }
            else
            {
                MagicOk = 0;
                //qDebug()<< data.size();
            }
        }

    if (MagicOk == 1)
    {
        // ************************************输出胸腔位移*********************************** //
        phaseWfm_Out = parseValueFloat(data, INDEX_IN_DATA_PHASE, 8); // Phase Waveforms
        // phaseWfm_Out是胸腔位移，需要乘上一个常量，常量的计算公式：\Delta \phi_{b}=\frac{4 \pi}{\lambda} \Delta R 。\Delta \phi_{b}是相位变化，\Delta R为心脏或者胸腔引起的微小位移变化
        phaseWfm_Out = phaseWfm_Out * 0.3100421; // 单位：mm
        //qDebug() << "displacement: " << *phaseWfm_Out;

        // ************************************输出距离*********************************** //
        unsigned int numRangeBinProcessed = m_demoParams.rangeBinEnd_index - m_demoParams.rangeBinStart_index + 1;
        QVector<double> RangeProfile(2*numRangeBinProcessed);
        QVector<double> xRangePlot(numRangeBinProcessed), yRangePlot(numRangeBinProcessed);
        unsigned int indexRange = INDEX_IN_DATA_RANGE_PROFILE_START;/*168;*/
        unsigned int positionmaxIndex;


        for (unsigned int index = 0; index < 2*numRangeBinProcessed; index ++)
        {
            qint16  tempRange_int  = parseValueInt16(data, indexRange, 4);  // Range Bin Index
            RangeProfile[index] = tempRange_int;
            indexRange = indexRange + 4;
        }

        for (unsigned int indexRangeBin = 0; indexRangeBin < numRangeBinProcessed; indexRangeBin ++)
        {
            yRangePlot[indexRangeBin] = sqrt(RangeProfile[2*indexRangeBin]*RangeProfile[2*indexRangeBin]+ RangeProfile[2*indexRangeBin + 1]*RangeProfile[2*indexRangeBin + 1]);
            xRangePlot[indexRangeBin] = m_demoParams.rangeStartMeters + m_demoParams.rangeBinSize_meters*indexRangeBin;
        }
        double maxRCS = *std::max_element(yRangePlot.constBegin(), yRangePlot.constEnd());

        for (int i = 0; i < yRangePlot.size(); ++i) // 查找对应最大值的下标索引值
        {
            if (yRangePlot.at(i) == maxRCS)
                positionmaxIndex = i;
        }
        positionmax = xRangePlot[positionmaxIndex] * 1000;  // 单位：mm
        //maxRCS_updated = alphaRCS*(maxRCS) + (1-alphaRCS)*maxRCS_updated;
        //qDebug() << "Range: " << *positionmax;
    }
  }
}

void VitalSigns::StopSerialPort()
{
    QString stp_str = "sensorStop\n";

    m_serialWrite->write(stp_str.toUtf8().constData());
    m_serialWrite->waitForBytesWritten(10000);
}

void VitalSigns::GetVitalSignsData()
{
    float da1;
    float da2;
    GetSerialData(da1,da2);
    //std::cout<<"data1:"<<da1<<"; data2: "<<da2<<std::endl;
}

#ifndef VITALSIGNS_H
#define VITALSIGNS_H

#include <QObject>
#include <iostream>
#include <windows.h>
#include <QTextStream>
#include <QFile>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QtEndian>
#include <QVector>
#include <QTimer>


#define MMWAVE_PLATFORM_1642                        // TI_mmwave Sensor Platfrom
#define LENGTH_MAGIC_WORD_BYTES                8    // Length of Magic Word appended to the UART packet from the EVM

#ifdef MMWAVE_PLATFORM_1642
    #define LENGTH_HEADER_BYTES               40   // Header + Magic Word
    #define LENGTH_TLV_MESSAGE_HEADER_BYTES    8
    #define LENGTH_DEBUG_DATA_OUT_BYTES       88   // VitalSignsDemo_OutputStats size
    #define MMWDEMO_OUTPUT_MSG_SEGMENT_LEN    32   // The data sent out through the UART has Extra Padding to make it a
                                                   // multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN
    #define LENGTH_OFFSET_BYTES               LENGTH_HEADER_BYTES  - LENGTH_MAGIC_WORD_BYTES + LENGTH_TLV_MESSAGE_HEADER_BYTES
    #define LENGTH_OFFSET_NIBBLES             2*LENGTH_OFFSET_BYTES

    #define  INDEX_GLOBAL_COUNT                  -5
    #define  INDEX_RANGE_BIN_PHASE               1
    #define  INDEX_RANGE_BIN_VALUE               2
    #define  INDEX_PHASE                         5
    #define  INDEX_BREATHING_WAVEFORM            6
    #define  INDEX_HEART_WAVEFORM                7
    #define  INDEX_HEART_RATE_EST_FFT            8
    #define  INDEX_HEART_RATE_EST_FFT_4Hz        9
    #define  INDEX_HEART_RATE_EST_FFT_xCorr      10
    #define  INDEX_BREATHING_RATE_FFT            11
    #define  INDEX_BREATHING_RATE_PEAK           12
    #define  INDEX_HEART_RATE_EST_PEAK           13
    #define  INDEX_CONFIDENCE_METRIC_BREATH      14
    #define  INDEX_CONFIDENCE_METRIC_HEART       15
    #define  INDEX_CONFIDENCE_METRIC_HEART_4Hz   16
    #define  INDEX_ENERGYWFM_BREATH              17
    #define  INDEX_ENERGYWFM_HEART               18
    #define  INDEX_CONFIDENCE_METRIC_HEART_xCorr 19
    #define  INDEX_RESERVED_1                    20
    #define  INDEX_RESERVED_2                    21
    #define  INDEX_RESERVED_3                    22
    #define  INDEX_RESERVED_4                    23
    #define  INDEX_RANGE_PROFILE_START           25  //(LENGTH_DEBUG_DATA_OUT_BYTES+LENGTH_TLV_MESSAGE_HEADER_BYTES)/4  + 1
#else
#define LENGTH_DEBUG_DATA_OUT_FLOAT       19   // These should match the values in the C-code
#define  LENGTH_OFFSET_NIBBLES            0
#define  INDEX_GLOBAL_COUNT               1
#define  INDEX_RANGE_BIN_PHASE            2
#define  INDEX_PAYLOAD_SIZE               2
#define  INDEX_RANGE_BIN_PHASE            2
#define  INDEX_RANGE_BIN_VALUE            3
#define  INDEX_CONFIDENCE_METRIC_HEART_4Hz              6
#define  INDEX_PHASE                      7
#define  INDEX_BREATHING_WAVEFORM         8
#define  INDEX_HEART_WAVEFORM             9
#define  INDEX_HEART_RATE_EST_FFT_4Hz     10
#define  INDEX_HEART_RATE_EST_FFT_xCorr   11
#define  INDEX_BREATHING_RATE_FFT         12
#define  INDEX_HEART_RATE_EST_FFT         13
#define  INDEX_BREATHING_RATE_PEAK        14
#define  INDEX_HEART_RATE_EST_PEAK        15
#define  INDEX_CONFIDENCE_METRIC_BREATH   16
#define  INDEX_CONFIDENCE_METRIC_HEART    17
#define  INDEX_ENERGYWFM_BREATH           18
#define  INDEX_ENERGYWFM_HEART            19
#define  INDEX_RANGE_PROFILE_START        20
#define  INDEX_RESERVED_1                    20
#define  INDEX_RESERVED_2                    21
#define  INDEX_RESERVED_3                    22
#define  INDEX_RESERVED_4                    23

#endif


#define  INDEX_IN_GLOBAL_FRAME_COUNT                LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_GLOBAL_COUNT*8
#define  INDEX_IN_RANGE_BIN_INDEX                   LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_RANGE_BIN_PHASE*8 + 4
#define  INDEX_IN_DATA_CONFIDENCE_METRIC_HEART_4Hz  LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_CONFIDENCE_METRIC_HEART_4Hz*8
#define  INDEX_IN_DATA_PHASE                        LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_PHASE*8
#define  INDEX_IN_DATA_BREATHING_WAVEFORM           LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_BREATHING_WAVEFORM*8
#define  INDEX_IN_DATA_HEART_WAVEFORM               LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_HEART_WAVEFORM*8
#define  INDEX_IN_DATA_BREATHING_RATE_FFT           LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_BREATHING_RATE_FFT*8
#define  INDEX_IN_DATA_HEART_RATE_EST_FFT           LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_HEART_RATE_EST_FFT*8
#define  INDEX_IN_DATA_HEART_RATE_EST_FFT_4Hz       LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_HEART_RATE_EST_FFT_4Hz*8
#define  INDEX_IN_DATA_HEART_RATE_EST_FFT_xCorr     LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_HEART_RATE_EST_FFT_xCorr*8
#define  INDEX_IN_DATA_BREATHING_RATE_PEAK          LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_BREATHING_RATE_PEAK*8
#define  INDEX_IN_DATA_HEART_RATE_EST_PEAK          LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_HEART_RATE_EST_PEAK*8
#define  INDEX_IN_DATA_CONFIDENCE_METRIC_BREATH     LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_CONFIDENCE_METRIC_BREATH*8
#define  INDEX_IN_DATA_CONFIDENCE_METRIC_HEART      LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_CONFIDENCE_METRIC_HEART*8
#define  INDEX_IN_DATA_ENERGYWFM_BREATH             LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_ENERGYWFM_BREATH*8
#define  INDEX_IN_DATA_ENERGYWFM_HEART              LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_ENERGYWFM_HEART*8
#define  INDEX_IN_RESERVED_1                        LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES  + INDEX_RESERVED_1*8
#define  INDEX_IN_RESERVED_2                        LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES  + INDEX_RESERVED_2*8
#define  INDEX_IN_RESERVED_3                        LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES  + INDEX_RESERVED_3*8
#define  INDEX_IN_DATA_RANGE_PROFILE_START          LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_RANGE_PROFILE_START*8


#define THRESH_BACK                       (30)
#define NUM_PTS_DISTANCE_TIME_PLOT        (256)
#define HEART_RATE_EST_MEDIAN_FLT_SIZE    (200)
#define thresh_HeartCM                    (0.25)
#define thresh_diffEst                    (20)
#define alpha_heartRateCM                 (0.2)
#define alphaRCS                          (0.2)
#define BACK_THRESH_BPM                   (4)
#define BACK_THRESH_CM                    (0.20)
#define BACK_THRESH_4Hz_CM                (0.15)

struct CfgParams
{
    float rangeStartMeters;
    float rangeEndMeters;
    float samplingRateADC_ksps;
    int   numSamplesChirp;
    float freqSlope_MHZ_us;
    float stratFreq_GHz;
    float chirpDuration_us;
    float chirpBandwidth_kHz;
    float rangeMaximum_meters;
    int   rangeFFTsize;
    float rangeBinSize_meters;
    int rangeBinStart_index;
    int rangeBinEnd_index;
    int numRangeBinProcessed;
    int totalPayloadSize_bytes;
    int totalPayloadSize_nibbles;
    float AGC_thresh;
};


class VitalSigns
{
public:
    VitalSigns();

    int m_indexBuffer;
    QByteArray m_dataBuffer;
    CfgParams m_demoParams;
    QSerialPort *m_serialWrite;
    QSerialPort *m_serialRead;
    QTimer *m_PollTimer;

    bool InitialSerialPort(int srcT=200);
    bool OpenSerialPort(QString srcFile);
    float GetSerialData();
    void GetSerialData(float &phaseWfm_Out, float &positionmax);
    void StopSerialPort();

private:
    int nextPower2(int num);
    float parseValueFloat(QByteArray data, int valuePos, int valueSize);
    qint16  parseValueInt16(QByteArray data, int valuePos, int valueSize);
    bool serialUserPortConfig(QSerialPort *serial, qint32 baudRate, QString userPortNum );
    bool serialDataPortConfig(QSerialPort *serial, qint32 baudRate, QString dataPortNum );

private slots:
    void GetVitalSignsData();
};

#endif // VITALSIGNS_H

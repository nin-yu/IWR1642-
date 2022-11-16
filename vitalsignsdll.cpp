#include "vitalsignsdll.h"
#include <vitalsigns.h>
#include <QTimer>

VitalSignsDll::VitalSignsDll()
{
}

int TestAdd_1(int srcA,int srcB)
{
    return  srcA+srcB;
}

//c# =>TestAdd_mws()
extern "C" __declspec(dllexport) int __stdcall TestAdd_mws(int srcA,int srcB)
{
    return  TestAdd_1(srcA,srcB);
}

VitalSigns *m_vitalsigns;
extern "C" __declspec(dllexport) bool __stdcall InitialSerialPort()
{
    m_vitalsigns = new VitalSigns();
    return  m_vitalsigns->InitialSerialPort();
}

extern "C" __declspec(dllexport) bool __stdcall OpenSerialPort(char *path)
{
    QString pathStr = QString(QLatin1String(path));
    //QString fileName = "D:\\LinaTech\\AT\\code\\20220803\\MVCGECSharp\\MVCGECSharp64\\bin\\x64\\Debug\\profile_2d_VitalSigns_20fps_Front.cfg";
    return m_vitalsigns->OpenSerialPort(pathStr);
}

extern "C" __declspec(dllexport) void __stdcall GetSerialData(float *dstData)
{
    dstData[0] = 0;
    dstData[1] = 0;
    float da1;
    float da2;
    m_vitalsigns->GetSerialData(da1,da2);
    dstData[0] = da1;
    dstData[1] = da2;
}

extern "C" __declspec(dllexport) void __stdcall StopSerialPort()
{
    m_vitalsigns->StopSerialPort();
}

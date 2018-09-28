
#ifndef _BASE_BRIDGE_H_
#define _BASE_BRIDGE_H_

#include "MyTypeDefine.h"
#include <pthread.h>
#include <string.h>

#include<semaphore.h>
/*****************************************************************************
 MicroDefine And Struct
 *****************************************************************************/
 typedef struct OtherSensorData
{
    S64 s32Timestamp;
    bool matchFlag;
    U32 u32Angle;
    S32 s32LDistance;
    S32 s32RDistance;
    S32 s32LSpeed;
    S32 s32RSpeed;
    S32 s32IMU;
}OtherSensorData_S;

typedef struct LDSMeasurementNode
{
    S64 s32Timestamp; ///ms
    U16 u16Sync;
    U16 u16Data_count;
    U16 u16Start_angle;
    U16 u16Distance[190];  //sometimes the quantities of data are more than 144, so I set it to 80.
    U8 u8Intensity[190];
    U16 u16Checksum;
}LDSMeasurementNode_S;

/*         xjtao*/
#define LDSFRAME_NODE_NUMBER 10
#define FILTER_N 8
typedef struct LDSMeasurementFrame
{
    U32 u32NodeFlag;   ////bit0 ~bit9
    LDSMeasurementNode_S astLDSNodeSet[LDSFRAME_NODE_NUMBER];
    OtherSensorData_S astOtherSensorSet[LDSFRAME_NODE_NUMBER];
}LDSMeasurementFrame_S;

typedef struct LaserFrameData
{
    S64 s32Timestamp;
    S32 s32FrameIndex;
    float Distance[1900];
    U8 Intensity[1900];
}LaserFrameData_S;



#define LDSDATAUNPACK_BUFLEN (8*1024)
#define MCUDATAUNPACK_BUFLEN (512)
typedef struct LDSDataUnPackHandler
{
    U8 databuf[LDSDATAUNPACK_BUFLEN];
    S32 buflen;
}LDSDataUnPackHandler_S;
typedef struct MCUDataUnPackHandler
{
    U8 databuf[MCUDATAUNPACK_BUFLEN];
    S32 buflen;
}MCUDataUnPackHandler_S;

typedef struct LDSRecvInfo
{
    string strDevName;
    S32 s32DevIndex;
    S64 s32LastRecvTick;
    S32 s32FrameNum;
    S32 s32LastFrameNum;
}LDSRecvInfo_S;
/* MCU related define*/
#define MAX_BUF (64)

#define DEG2RAD(x) x*M_PI/180.0
extern double gMaxAngle_;
extern double gMinAngle_;
extern int gWindow_;
extern int gNeighbors_;
extern bool gEnableLDSCompensate;

class BaseBridgeAPI
{
public:
    BaseBridgeAPI();
    ~BaseBridgeAPI();
 /*****************************************************************************
 Function
 *****************************************************************************/
    S32 MCUInit(string strDevName);
    S32 BaseBridgeInit(string strMCUName, string strLDSName);

    S32 GetSensorData(S32 s32ExpectTime, OtherSensorData_S *pstSensorInfo);
    S32 GetSensorData(OtherSensorData_S *pstSensorInfo);

    S32 GYIODistanceReset(U32 u32NeedIndex);
    S32 NavyMCUMoveCtrl(S16,S16,S16,S16);

    void NavyMCUMegneticCal();
    S32 NavyMCUMegneticGetData(S16 *s16Deg, S8 *s8Flag);

    inline void SetIntensityThresh(U8 thresh)
    { u8IntensityThresh_ = thresh;}
private:

    static void* velPub(void*);
/*****************************************************************************
 Function
 *****************************************************************************/

    static void *WorkerRun(void *arg);
    S32 MainLoop();

    S32 MCUDataProcess(U8 *pu8RecvData, U32 u32RecvLen);

    S32 MCUDataFrameAnalyse(U8* pdata, S32 datalen);
    S32 ClientMsgProcess(U8 *pu8MessageBuf, U32 u32MessageLen);


    S32 OnMCUSerialComRead();
    S32 OnSerialComWrite(S32 Fd, U8 *data, U16 len);
    S32 OnMCUMessageReceive();
    S32 MCUSendMoveCtrl_1(S16,S16,S16,S16);
    S32 SendMoveStop(S32,S32);
/*****************************************************************************

 *****************************************************************************/
    pthread_t threadHandler;
    S32 s32LDSFd;
    S32 s32MCUFd;

    LDSRecvInfo_S stLDSRecvInfo; ////for lds device state check

    /** for gyio & distance */
    pthread_mutex_t GyioDisMutex;
    pthread_mutex_t LDSCommMutex_;
    bool bLDSComWaitingRet_;
    bool bGetGyioDisFlag;
    U32 u32ResetGYIODistanceReqIndex;
#define SENSORINTERFACE_GYRO_VECTORBUFSIZE 10
    vector<OtherSensorData_S *> vecGYRODistance;

    /** for lds uart recv */
    pthread_mutex_t LDSMutex;
    LDSDataUnPackHandler_S stLDSHandler;
    MCUDataUnPackHandler_S stMCUHandler;

    LDSMeasurementNode_S stLDSUnpackNode;
    LDSMeasurementFrame_S stLDSUnpackFrame;
#define SENSORINTERFACE_LDS_VECTORFRAMESIZE 5
    vector<LDSMeasurementFrame_S *> vecLDSRawFrame;

    U32 mLastGyroGetData;
    S32 mLastGyroSetData;

    pthread_t m_threadID ;
    bool m_isTerminal;
    pthread_mutex_t m_locker;
//4:ok , 1: lowVoltage , 2: unknow error, 3:comm port not open
    U8 LDSState_;
    /* mcu comm related*/
    U8 RxBuf[MAX_BUF];
    U8 mRxData[MAX_BUF];
    U8 mHeadFlag;

    U32 mGyroData;
    S32 mLSpeed;
    S32 mRSpeed;
    U32 idNum  ;

    /*   mag related*/
       sem_t  MagneticSem;
       S16 u16MagneticDeg;
       U8  u16MagneticFlag;

       U8 u8IntensityThresh_;
       bool bIntensityMode_;




    pthread_mutex_t MCUCMDMutex_;
    pthread_mutex_t MCUComMutex_;

 };

#endif

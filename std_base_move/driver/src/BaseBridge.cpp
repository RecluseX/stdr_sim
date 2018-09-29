#include "BaseBridge.h"


#include <poll.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <set>
#include <iostream>
#include "BaseTimeApi.h"

static S8 crc8_table[256];     /* 8-bit table */
static S32 made_table = 0;

#define GP  0x107   /* x^8 + x^2 + x + 1 */
#define DI  0x07
static FILE *stfp = NULL;
pthread_t gCmd_thread_ = -1;
S32 gVelFd_ = -1;

/*****************************************************************************
 MicroDefine And Struct
 *****************************************************************************/
#define DEG2RAD(x) x*M_PI/180.0
#define RAD2DEG(x) x*180.0/M_PI

//#define Debug_ 1
U32 g_u32SensorMapTraceID = 0;




S32 deltaTheta(U32& a, S32& b)
{
    S32 result = 0 ;
    S32 c = a - b ;
    //fprintf(stderr, "c:%d",c);

    if(std::abs(c)>18000)
        {
            return c + (c/(std::abs(c))) * (-36000);
        }
    return c;
}
  S32 deltaOdom(S32* currentOdom, S32* lastOdom, float* deltaE)
  {
      float LDist = (currentOdom[1] - lastOdom[1])/1000.0;
      float RDist = (currentOdom[0] - lastOdom[0])/1000.0;
      float dTheta = DEG2RAD(-1*(currentOdom[2] - lastOdom[2])/100.0);

      float dS = (LDist + RDist)/2.0;
      deltaE[0] = dS*cos(dTheta);
      deltaE[1] = dS*sin(dTheta);
      return 0;
  }

  
  float getAngleWithViewPoint(float& dist1,float& dist2,float& deltaT)
  	{
  		return atan2(dist2*sin(deltaT),dist1 - dist2*cos(deltaT));
  	}
  
 
/*****************************************************************************
 Global
 *****************************************************************************/

/*
 * Should be called before any other crc function.
 */
void init_crc8()
{
    S32 i,j;
    U8 crc;
    if (!made_table)
    {
        for (i = 0; i < 256; i++)
        {
            crc = i;
            for (j = 0; j < 8; j++)
            crc = (crc << 1) ^ ((crc & 0x80) ? DI : 0);
            crc8_table[i] = crc & 0xFF;
            /* printf("table[%d] = %d (0x%X)\n", i, crc, crc); */
        }
        made_table = 1;
    }
}

/*
 * For a byte array whose accumulated crc value is stored in *crc, computes
 * resultant crc obtained by appending m to the byte array
 */
void crc8(U8 *crc, U8 m)
{
    if (!made_table)
    {
        init_crc8();
    }
    *crc = crc8_table[(*crc) ^ m];
    *crc &= 0xFF;
}


void getStrCrc8(U8 *crc, U8 *str, U32 length)
{
    U32 i;
    if((NULL == crc) || (NULL == str))
    {
        return;
    }

    for(i = 0; i < length; i++)
    {
        crc8(crc , *(str + i));
    }
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 CheckCRC(U8 *data, U32 len)
{
    U8 crc = 0;
    getStrCrc8(&crc, data, len);

    if (crc != data[len])
    {
        return -1;
    }

    return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
BaseBridgeAPI::BaseBridgeAPI():mGyroData(0),
    mLSpeed(0),mRSpeed(0)
{
    s32LDSFd = -1;
    s32MCUFd = -1;

    ////reset data for gyro
    bGetGyioDisFlag = false;//xjtao
    u32ResetGYIODistanceReqIndex = -1;


    memset(&stMCUHandler, 0, sizeof(MCUDataUnPackHandler_S));

     mLastGyroSetData = 0;
     mLastGyroGetData = 0 ;

     m_threadID = -1 ;
     m_isTerminal = false;

    pthread_mutex_init(&GyioDisMutex, NULL);
     pthread_mutex_init(&m_locker,NULL);// = PTHREAD_MUTEX_INITIALIZER;
     pthread_mutex_init(&MCUCMDMutex_, NULL);
     pthread_mutex_init(&MCUComMutex_, NULL);



     idNum = 0 ;
     bIntensityMode_ = false;

      sem_init(&MagneticSem,0,0);

}

BaseBridgeAPI::~BaseBridgeAPI()
{
    pthread_mutex_destroy(&GyioDisMutex);
    while(vecGYRODistance.size() > 0)
    {
        vector<OtherSensorData_S *>::iterator iter1;
        iter1 = vecGYRODistance.begin();
        delete (OtherSensorData_S *)(*iter1);
        vecGYRODistance.erase(iter1);
    }
    vecGYRODistance.clear();

    

    pthread_mutex_lock(&m_locker);
    m_isTerminal = true;
    pthread_mutex_unlock(&m_locker);


    if(m_threadID != -1)
    pthread_join(m_threadID,NULL);
    if(gVelFd_ != -1)
 	pthread_join(gVelFd_,NULL);

    pthread_mutex_destroy(&m_locker);
    pthread_mutex_destroy(&MCUCMDMutex_);
    pthread_mutex_destroy(&MCUComMutex_);
    sem_destroy(&MagneticSem);
    if(stfp != NULL)
    fclose(stfp);

   }



/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseBridgeAPI::MCUInit(string strDevName)
{
    s32MCUFd = open((S8 *)strDevName.c_str(), O_RDWR | O_NOCTTY);
    if (-1 == s32MCUFd)
    {
        printf("MCUInit MCU open %s Error\n", (S8 *)strDevName.c_str());
        return 1;
    }
    struct termios new_cfg,old_cfg;
    int speed;
    /* save present comm config*/
    if(tcgetattr(s32MCUFd,&old_cfg) != 0)
    {
        perror("tcgetattr");
        return -1;
    }
    new_cfg=old_cfg;
    cfmakeraw(&new_cfg);/* config the comm attribute to raw  mode*/
    new_cfg.c_cflag &= ~CSIZE;

    /*set baud*/
    speed = B115200;
    cfsetispeed(&new_cfg,speed);
    cfsetospeed(&new_cfg,speed);

    /*set date bit*/
    new_cfg.c_cflag |=CS8;
    new_cfg.c_cflag &= ~PARENB;
    new_cfg.c_iflag &= ~INPCK;

    /*set stop bit*/
    new_cfg.c_cflag &= ~CSTOPB;
    /*set wait time and minimum bit to receive*/
    new_cfg.c_cc[VTIME] = 0 ;
    new_cfg.c_cc[VMIN] = 0;
    tcflush(s32MCUFd,TCIFLUSH);

    /* active new config*/
    if(tcsetattr(s32MCUFd,TCSANOW,&new_cfg) !=0 ){
        perror("tcsetattr");
        return -1;
    }
    return 0;
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseBridgeAPI::BaseBridgeInit(string strMCUName, string strLDSName)
{
    S32 s32Ret;

    stLDSRecvInfo.strDevName = strLDSName;


    /** Init Communication **/
    s32Ret = MCUInit(strMCUName);
    if(s32Ret)
    {
        printf("*****************************   MCUInit Error   ********************************\n");
        return -1;
    }

    /// Create thread for MCUCommunication Worker.
    m_threadID = pthread_create((pthread_t*)&threadHandler, NULL, WorkerRun, this);
    //gVelFd_  = pthread_create((pthread_t*)&gCmd_thread_,NULL, velPub,this);
    if(0 != m_threadID)
    {
        printf("SensorInterfaceInit  pthread_create Error\n");
        return 1;
    }

    return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
void *BaseBridgeAPI::WorkerRun(void *arg)
{
    BaseBridgeAPI *pclHandler;

    /// set thread parameter.
   // pthread_detach(pthread_self());

    printf("Thread <BaseBridgeAPI::WorkerRun> is Running ...\n\n");

    pclHandler = reinterpret_cast<BaseBridgeAPI *>(arg);

    pclHandler->MainLoop();

    return NULL;
}

/**************************************************************
Description:
Input:
Output:
Return
    ***************************************************************/
S32 BaseBridgeAPI::MainLoop()
{
#define MCU_MAX_BUFFER_LEN 50

    int ret;
    int timeout;

    U8 mcu_recvbuf[MCU_MAX_BUFFER_LEN] = {0};
    int readLen;
    struct pollfd fds[2];



    if(s32MCUFd != -1 )
    {
        fds[1].fd = s32MCUFd;
        fds[1].events = POLLIN | POLLERR ;           //  POLLIN锟斤拷示锟斤拷锟斤拷锟捷可讹拷, POLLERR锟斤拷示锟斤拷锟斤拷
        fds[1].revents = 0;
    }
    else
    {
        fds[1].fd = -1;
    }
    timeout = 10;

    while( 1 )
    {
    	if(m_isTerminal == true)
    		  break;

        if(s32MCUFd < 0)
        {
            fds[1].fd = -1;
            /** reconnect to application **/
            /*************/
        }
        ret = poll( fds, 2, timeout);
        if ( ret < 0 )
        {
            printf("poll failure!\n");
            break;
        }
        if(0 == ret)
        {
            continue;
        }


        if( (fds[1].revents & POLLERR) ||(fds[1].revents & POLLRDHUP) )                     //  锟斤拷锟捷筹拷锟斤拷
        {
            printf("MCU Disconnect\n");
            close(s32MCUFd);
            s32MCUFd = -1;
        }
        else if( fds[1].revents & POLLIN )                      //  锟酵伙拷锟斤拷锟阶斤拷锟斤拷锟斤拷锟斤拷锟捷匡拷写
        {
            //printf("mcu poll in event\n");
            int readLen;
            memset(mcu_recvbuf,  0 ,  MCU_MAX_BUFFER_LEN);
            readLen = read(s32MCUFd, mcu_recvbuf,  MCU_MAX_BUFFER_LEN);
            if( readLen <= 0 )
            {
                 ////close and reset
                close( s32MCUFd);
                s32MCUFd = -1;
            }
            else
            {
                /** Get Message From Remote **/
                int ret = MCUDataProcess(mcu_recvbuf, readLen);
                printf("mcudataProcess rst:%d, readLen : %d, \n",ret,readLen);

            }
        }

    }

    return 0;
}


/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseBridgeAPI::MCUDataProcess(U8 *pu8RecvData, U32 u32RecvLen)
{
  MCUDataUnPackHandler_S* pstHandler = &stMCUHandler;

  if(pstHandler->buflen + u32RecvLen >= MCUDATAUNPACK_BUFLEN) // 512
  {
    printf("data over flow in mcu comm\n");
    pstHandler->buflen = 0 ;
    return 1;
  }
#if 1
{
    if(NULL == stfp)
    {
        stfp = fopen("/home/yang/savecmudata.txt", "wb");
    }
    if(NULL != stfp)
    {
        int count = fwrite(pu8RecvData, 1, u32RecvLen, stfp);
    }
}
#endif

  memcpy(pstHandler->databuf + pstHandler->buflen,pu8RecvData,u32RecvLen);
  pstHandler->buflen += u32RecvLen;
  if(pstHandler->buflen < 11)
      return 2;

   S32 pos1,pos2;
   U8* ptmp;
   bool bStartFlag = false;

   pos1=0;pos2=0;
   ptmp = pstHandler->databuf;

   int i = 0;int j = 0;
   while(i < (pstHandler->buflen -1))
   {
        if(ptmp[i] == 0xaa && ptmp[i+1] == 0xcc)
        {
            pos1 = i;
            bStartFlag  = true;
            if(i + 10 > pstHandler->buflen)
                break;
            for(j = i+5 ; j < (pstHandler->buflen) ; ++j)
            {
                if(ptmp[j] == '\r'&& ptmp[j-1] == '\n')
                {
                    pos2 = j;
                    break;
                }
            }
            break;
        }
        i++;

   }
   if(bStartFlag == false)
       return 3;
//printf("bstartFlag == true pos:%d,%d\n",pos1,pos2);
   if(pos2 == 0)
    {
       if(pos1 != 0)
        {
            printf("find start but not end\n");
            memmove(pstHandler->databuf,pstHandler->databuf+pos1,pstHandler->buflen-pos1);
            pstHandler->buflen -= pos1;
        }
       return 4;
   }

   if(pos2 - pos1 > 30)
   {
	printf("delta pos is too large\n");
        pstHandler->buflen = 0 ;
        return 5;
   }
   ptmp = pstHandler->databuf + pos1;
   int frameLen = pos2 -pos1 +1 ;
   int ret = MCUDataFrameAnalyse(ptmp,frameLen);
   if(ret != 0)
   {
      // printf("MCUDataFrameAnalyses  Error:%d\n",ret);
       memmove(pstHandler->databuf,pstHandler->databuf+pos2+1,pstHandler->buflen - pos2 - 1);
       pstHandler->buflen -= (pos2 +1);

       return 6;
   }

   memmove(pstHandler->databuf,pstHandler->databuf+pos2+1,pstHandler->buflen - pos2 - 1);
   pstHandler->buflen -= (pos2 +1);
   S8 command, type, response;
   S32 Ldistance ,Rdistance;
   bool bflag = false;
   U32 u32WENum = 0 ;
   command = 0;
   type = 0 ;
   command = mRxData[6];
   type = mRxData[7];
   pthread_mutex_lock(&MCUCMDMutex_);
   if(command == 'W' && type == 'E')
   {
        Ldistance = mRxData[8] | (mRxData[9] << 8) |
            (mRxData[10] <<16) | (mRxData[11] << 24);
        Rdistance = mRxData[12] | (mRxData[13] << 8)|
            (mRxData[14] <<16) | (mRxData[15] << 24);
     //   dataReadyFlag_ = true;
     //printf("Ldistance:%d, rdistance:%d\n",Ldistance,Rdistance);
        OtherSensorData_S* odom = new OtherSensorData_S;
        odom->s32LDistance = Ldistance;
        odom->s32RDistance = Rdistance;
        odom->u32Angle = mGyroData;
        odom->s32Timestamp = BaseGetTimeTick();
	//std::cout<<"s32:"<<cecovacs::common::BaseGetTimeTick()<<std::endl;
        odom->s32LSpeed = mLSpeed;
        odom->s32RSpeed = mRSpeed;
        pthread_mutex_lock(&GyioDisMutex);
        vector<OtherSensorData_S*>::iterator iter1;
        if(vecGYRODistance.size() > SENSORINTERFACE_GYRO_VECTORBUFSIZE)
        {
            iter1 = vecGYRODistance.begin();
            delete (OtherSensorData_S*)(*iter1);
            vecGYRODistance.erase(iter1);
        }

        vecGYRODistance.push_back(odom);
        pthread_mutex_unlock(&GyioDisMutex);
   }
   else if(command == 'G' && type == 'B')
   {
        mGyroData = mRxData[8] | (mRxData[9] << 8);
      //  printf("gyrodata :%d\n",mGyroData);
   }
   else if(command =='E' && type=='A')
   {
       u16MagneticDeg  = mRxData[8] + (mRxData[9]<<8);
       u16MagneticFlag = mRxData[10];
       sem_post(&MagneticSem);
   }
    else if(command == 'W' && type== 'R')
    {
       short u16RSpeed =  mRxData[8] | (mRxData[9] <<8 );
       mRSpeed = u16RSpeed;
       short u16LSpeed  =  mRxData[10] | (mRxData[11] << 8);
       mLSpeed = u16LSpeed;
    }
   pthread_mutex_unlock(&MCUCMDMutex_);
//  printf("stMCUHandler->buflen:%d\n",pstHandler->buflen);
  // pstHandler->buflen = 0;
   return 0;

}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseBridgeAPI::OnSerialComWrite(S32 Fd, U8 *data, U16 len)
{
    U32 i;
    int ret;
    for (i = 0; i < len; i++)
    {
        ret = write(Fd, &data[i], 1);
        if(ret != 1)
            {
                printf("OnMCUSerialComWriteErr, ret = %d,Fd:%d\n", ret,Fd);
            }
    }


    return 0;
}
S32 BaseBridgeAPI::MCUDataFrameAnalyse(U8* pdata, S32 datalen)
{
    S32 len;
    len = pdata[2] | (pdata[3] << 8);
    if((8 > len) || (len > MAX_BUF))
    {
	for(int i = 0 ; i < datalen; ++i)
{
	printf("%0x ",pdata[i]);

}
printf("\n");
        printf("mcu data frame analyses error ,len is :%d\n",len);
        return 2;
    }
 //   memset(mRxData , 0 , sizeof(mRxData));
   if(len != datalen)
{
       	for(int i = 0 ; i < datalen; ++i)
{
	printf("%0x ",pdata[i]);
}
printf("\n");
}
    memcpy(mRxData, pdata, len);
    if(CheckCRC(mRxData +4, len - 7 ))
    {
        printf("len is :%d,datalen is:%d\n",len,datalen);
return 3;
    }
    return 0;
}



S32 BaseBridgeAPI::GetSensorData( OtherSensorData_S *pstSensorInfo)
{

#define SENSOR_MATCH_THRESHOLD_MS 200
    S32 s32Tmp, i;
    OtherSensorData_S *pstGYROMatch = NULL;
    S32 s32Min = SENSOR_MATCH_THRESHOLD_MS;

    if(0 == vecGYRODistance.size())
    {
        return 1;
    }

    /** get gyio ... sensor data from vector **/
    pthread_mutex_lock(&GyioDisMutex);
    pstGYROMatch = vecGYRODistance.back();

//    if(NULL != pstGYROMatch)
    {
        pstSensorInfo->s32Timestamp = pstGYROMatch->s32Timestamp;
        pstSensorInfo->u32Angle = pstGYROMatch->u32Angle;
        pstSensorInfo->s32LDistance= pstGYROMatch->s32LDistance;
        pstSensorInfo->s32RDistance= pstGYROMatch->s32RDistance;
        pstSensorInfo->s32IMU = 0;
        pstSensorInfo->s32LSpeed = pstGYROMatch->s32LSpeed;
        pstSensorInfo->s32RSpeed = pstGYROMatch->s32RSpeed;

    }
    while(vecGYRODistance.size() > 0)
    {
        vector<OtherSensorData_S *>::iterator iter1;
        iter1 = vecGYRODistance.begin();
        delete (OtherSensorData_S*)(*iter1);
        vecGYRODistance.erase(iter1);
    }
    vecGYRODistance.clear();

    pthread_mutex_unlock(&GyioDisMutex);

    return 0;

}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseBridgeAPI::GYIODistanceReset(U32 u32NeedIndex)
{
    U8 crc = 0;
    S32 len, timeOut = 0;
    U8 SetCommandBuf[11] = {0xaa, 0xcc, 11, 0, 0, 1, 'W', 'F', 0, 0x0A, 0x0D};

    u32NeedIndex = u32NeedIndex;

    len = (SetCommandBuf[2] | SetCommandBuf[3] << 8);
    getStrCrc8(&crc, SetCommandBuf + 4, len - 7);
    SetCommandBuf[8] = crc;

    pthread_mutex_lock(&MCUComMutex_);
    OnSerialComWrite(s32MCUFd, SetCommandBuf, len);
    pthread_mutex_unlock(&MCUComMutex_);


    return 0;
}
/* send navy move ctrl*/
S32 BaseBridgeAPI::NavyMCUMoveCtrl(S16 s16LSpeed,S16 s16RSpeed,
        S16 s16LAcce, S16 s16RAcce)
{
    if(s16LSpeed || s16RSpeed)
    {
        MCUSendMoveCtrl_1(s16LSpeed,s16RSpeed,s16LAcce,s16RAcce);
    }
    else{
        SendMoveStop(s16LSpeed,s16LAcce);
    }
    //usleep(10);
    return 0;
}
/* mcu send move ctrl _1*/
S32 BaseBridgeAPI::MCUSendMoveCtrl_1(S16 s16LSpeed,S16 s16RSpeed,
        S16 s16LAcce, S16 s16RAcce)
{
    U8 crc = 0 ;
    S32 len;
    U8 SetSpeed[20] = {0xaa,0xcc,20,0,0,1,'W','A',1,0,0,0,0,0,0,0,0,
    1,0x0A,0x0D};
    printf("MCUSendMoveCtrl,%d,%d,%d,%d\n",s16LSpeed,s16RSpeed,s16LAcce,s16RAcce);
    SetSpeed[5] = idNum++;
    SetSpeed[8] = 1;
    SetSpeed[9] = s16LSpeed& 0xFF;
    SetSpeed[10] = (s16LSpeed >> 8) & 0xFF;
    SetSpeed[11] = s16RSpeed & 0xFF;
    SetSpeed[12] = (s16RSpeed >> 8) & 0xFF;
    SetSpeed[13] = s16LAcce & 0xFF;
    SetSpeed[14] = (s16LAcce >> 8) & 0xFF;
    SetSpeed[15] = s16RAcce & 0xFF;
    SetSpeed[16] = (s16RAcce >> 8) & 0xFF;
    len = SetSpeed[2] | SetSpeed[3] << 8;
    getStrCrc8(&crc , SetSpeed +4 , len-7);
    SetSpeed[17] = crc;
    pthread_mutex_lock(&MCUComMutex_);
    OnSerialComWrite(s32MCUFd,SetSpeed,len);
    pthread_mutex_unlock(&MCUComMutex_);
return 0 ;
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseBridgeAPI::SendMoveStop(S32 s32Speed, S32 s32Accleration)
{
    U8 crc = 0;
    S32 len = 0;
    U8 SetSpeed[22] = {0xaa, 0xcc, 14, 0, 0, 1, 'W', 'A', 0, 0, 0, 1, 0x0A, 0x0D};

    S16 s16Speed, s16Acce;

    s16Speed = s32Speed;
    s16Acce = s32Accleration;

  //  printf("SendMoveStop, [%d] \n", s32Accleration);

    SetSpeed[5] = idNum++;
    if(s32Accleration > 900)
    {
        SetSpeed[8] = 3;   ////break stop
    }
    else
    {
        SetSpeed[8] = 2;   ////slow stop
    }
    SetSpeed[9] = s16Acce & 0xFF;
    SetSpeed[10] = (s16Acce >> 8) & 0xFF;
    len = (SetSpeed[2] | SetSpeed[3] << 8);
    getStrCrc8(&crc, SetSpeed + 4, len - 7);
    SetSpeed[11] = crc;
    pthread_mutex_lock(&MCUComMutex_);
    OnSerialComWrite(s32MCUFd,SetSpeed, len);
    pthread_mutex_unlock(&MCUComMutex_);
    return 0;
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
void BaseBridgeAPI::NavyMCUMegneticCal()
{
    U8 crc = 0;
    S32 len;
    U8 SetCal[11] = {0xaa, 0xcc, 11, 0, 0, 1, 'E', 'B',1, 0x0A, 0x0D};

    printf("NavyMCUMegneticCal\n");

    len = (SetCal[2] | SetCal[3] << 8);
    getStrCrc8(&crc, SetCal + 4, len - 7);
    SetCal[8] = crc;
    pthread_mutex_lock(&MCUComMutex_);
    OnSerialComWrite(s32MCUFd, SetCal, len);
    pthread_mutex_unlock(&MCUComMutex_);

    return ;
}
S32 BaseBridgeAPI::NavyMCUMegneticGetData(S16 *s16Deg, S8 *s8Flag)
{
    U8 crc = 0;
    S32 len, ret;
    U8 SetMeg[11] = {0xaa, 0xcc, 11, 0, 0, 1, 'E', 'A' ,1, 0x0A, 0x0D};
    struct timespec ts;

    printf("NavyMCUMegneticGetData\n");

    len = (SetMeg[2] | SetMeg[3] << 8);
    getStrCrc8(&crc, SetMeg + 4, len - 7);
    SetMeg[8] = crc;
    pthread_mutex_lock(&MCUComMutex_);
    OnSerialComWrite(s32MCUFd,SetMeg, len);
    pthread_mutex_unlock(&MCUComMutex_);


   if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
        printf("clock_gettime error\n");
        return -1;
   }

   ts.tv_sec +=1;

    ret = sem_timedwait(&MagneticSem,&ts);
    if (ret == -1)
    {
        *s8Flag = 1;
        *s16Deg = 0;
            return -1;
    }
        *s8Flag = u16MagneticFlag;
        *s16Deg = u16MagneticDeg;

    return 0;
}


void* BaseBridgeAPI::velPub(void* arg)
{
	BaseBridgeAPI* pHandle = reinterpret_cast<BaseBridgeAPI *>(arg);
	S64 timeStart = BaseGetTimeTick();
 	S32 dur = 0;
     	while(dur < 100000)
{

	pHandle->NavyMCUMoveCtrl(0,0,600,600);
	dur = BaseGetTimeTick() - timeStart;
	usleep(100000);

}
timeStart = BaseGetTimeTick();
dur = 0;
 	while(dur < 100000)
{

	pHandle->NavyMCUMoveCtrl(500,100,600,600);
	dur = BaseGetTimeTick() - timeStart;
	usleep(100000);

}
return NULL;
}



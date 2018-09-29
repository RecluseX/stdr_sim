#ifndef _MY_TYPE_DEFINE_H_
#define _MY_TYPE_DEFINE_H_
#include <stdio.h>
#include <string>
#include <vector>
#include <stdlib.h>
typedef int   S32;
typedef long long S64;
typedef unsigned char U8;
typedef unsigned int U32;
typedef unsigned short U16;
typedef short S16;
typedef char S8;
using std::string;
using std::vector;

typedef struct ODOMDATA{
    int ADist;
    int BDist;
    int CDist;
    int LDist;
    int RDist;
    int Gyro;
    long long timeStamp;
} Odom_Type;

#endif


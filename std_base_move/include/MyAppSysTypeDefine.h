/******************************************************************************
 * Copyright (C)
 * File name:       MYAppSysTypeDefine.h
 * Version:
 * Description:    base type define in MYAppSys Team
 * Other:
 * History:
 ******************************************************************************/

#ifndef _MYAPPSYS_TYPE_DEFINE_H_
#define _MYAPPSYS_TYPE_DEFINE_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#ifdef __cplusplus
extern "C" {
#endif


/** platform define */
#ifdef WIN32
#define MY_WIN32
#endif



#define CONST   const
#define STATIC  static
#define INLINE  STATIC __inline
#define EXTERN  extern



typedef char               S8;              /*8-bit*/
typedef short              S16;            /*16-bit*/
typedef int                S32;            /*32-bit*/

typedef unsigned char         U8;              /*8-bit*/
typedef unsigned short        U16;            /*16-bit*/
typedef unsigned int          U32;            /*32-bit*/

typedef float                 FLOAT;

typedef struct
{
    U8 *ptr;
    S32 length;
    S32 fill;
} MYAppSysBD;

#define MX_SUCCESS          (1)
#define MX_FAIL                 (0)


#define INT8_MAX        (+127)
#define INT16_MAX       (+32767)
#define INT32_MAX       (+2147483647L)

#define INT8_MIN        (-INT8_MAX - 1)
#define INT16_MIN       (-INT16_MAX - 1)
#define INT32_MIN       (-INT32_MAX - 1)

#define UINT8_MAX       (0xffU)
#define UINT16_MAX      (0xffffU)
#define UINT32_MAX      (0xffffffffUL)


/* ֵָָPTR_GRID (ƽ̨) */
#define PTR_GRID        4


#define GRID_PTR(p)     ((void*)(((Address)(p)+(PTR_GRID-1))/PTR_GRID*PTR_GRID))
#define GRID_SIZE(n)    ((Size)(((Size)(n)+(PTR_GRID-1))/PTR_GRID*PTR_GRID))


#define IS_PTR_VALID(p) (NULL!=(p))


#define POINTER_CONVERT(t,p)    ((t)(p))


#define ARRAY_SIZE(ar)  (sizeof(ar)/sizeof(*(ar)))



#define MAX(a,b)        (((a)>(b))?(a):(b))
#define MIN(a,b)        (((a)<(b))?(a):(b))



#define MEMBER_OFFSET(s,m)   ((Size)&(((s *)0)->m))
#define SIZEOF_MEMBER(s,m)   (sizeof(((s *)0)->m))



#define MYAppSysMalloc(A)                    (malloc(A))
#define MYAppSysFree(A)                       (free(A))
#define MYAppSysMemZero(p,n)            ((void)memset((p),0,(Size)(n)))
#define MYAppSysMemCopy(d,s,n)         ((void)memcpy((d),(s),(Size)(n)))
#define MYAppSysMemMove(d,s,n)	   ((void)memmove((d),(s),(Size)(n)))
#define MYAppSysMemComp(d,s,n)	   ((Cmp)memcmp((d),(s),(Size)(n)))



#define GET_BITFIELD(field, mask, shift)    (((field) & (mask)) >> (shift))


#define SET_BITFIELD(field, val, mask, shift) \
{ \
(field) &= ~(mask); \
(field) |= (((val) << (shift)) & (mask)); \
}

#ifdef __cplusplus
}
#endif

#endif



#ifndef __AIKIT_BIZ_TYPE_H__
#define __AIKIT_BIZ_TYPE_H__

#include <stdio.h>
#include "aikit_type.h"

/**
 * AIKIT API type
 */
#if defined(_MSC_VER)            /* Microsoft Visual C++ */
#   if !defined(AIKITAPI)
#       if defined(AIKIT_EXPORT)
#           define AIKITAPI __declspec(dllexport)
#       else
#           define AIKITAPI __declspec(dllimport)
#       endif
#   endif
#elif defined(__BORLANDC__)      /* Borland C++ */
#   if !defined(AIKITAPI)
#       define AIKITAPI __stdcall
#   endif
#elif defined(__WATCOMC__)       /* Watcom C++ */
#   if !defined(AIKITAPI)
#       define AIKITAPI __stdcall
#   endif
#else                            /* Any other including Unix */
#   if !defined(AIKITAPI)
#       if defined(AIKIT_EXPORT)
#           define AIKITAPI __attribute__ ((visibility("default")))
#       else
#           define AIKITAPI
#       endif
#   endif
#endif

#if defined(_MSC_VER)
#   if !defined(attribute_deprecated)
#       define attribute_deprecated __declspec(deprecated)
#   endif
#else
#   if !defined(attribute_deprecated)
#       define attribute_deprecated __attribute__((deprecated))
#   endif
#endif

#ifndef int32_t
typedef int int32_t;
#endif
#ifndef uint32_t
typedef unsigned int uint32_t;
#endif

typedef AIKIT_BaseParam       AIKIT_BizParam;
typedef AIKIT_BaseParamPtr    AIKIT_BizParamPtr;
typedef AIKIT_BaseData        AIKIT_BizData;
typedef AIKIT_BaseDataPtr     AIKIT_BizDataPtr;

typedef AIKIT_BizData         AIKIT_InputData;

typedef struct _AIKIT_BaseDataList {
    AIKIT_BaseData *node;     // 链表节点
    int32_t count;          // 链表节点个数
    int32_t totalLen;       // 链表节点所占内存空间：count*(sizeof(AIKIT_BaseData)+strlen(node->key)+node->len)
} AIKIT_BaseDataList, *AIKIT_BaseDataListPtr;      // 配置对复用该结构定义

typedef struct _AIKIT_BaseParamList {
    AIKIT_BaseParam *node;    // 链表节点
    int32_t count;          // 链表节点个数
    int32_t totalLen;       // 链表节点所占内存空间：count*(sizeof(AIKIT_BaseParam)+strlen(node->key)+node->len)
} AIKIT_BaseParamList, *AIKIT_BaseParamListPtr;      // 配置对复用该结构定义

typedef AIKIT_BaseDataList    AIKIT_OutputData;
typedef AIKIT_BaseParamList   AIKIT_OutputEvent;

typedef struct {
    void*       usrContext;
    const char* abilityID;
    size_t      handleID;
} AIKIT_HANDLE;


typedef void (*AIKIT_OnOutput)(AIKIT_HANDLE* handle, const AIKIT_OutputData* output);
typedef void (*AIKIT_OnEvent)(AIKIT_HANDLE* handle, AIKIT_EVENT eventType, const AIKIT_OutputEvent* eventValue);
typedef void (*AIKIT_OnError)(AIKIT_HANDLE* handle, int32_t err, const char* desc);

typedef struct {
    AIKIT_OnOutput outputCB;  //輸出回调
    AIKIT_OnEvent  eventCB;   //事件回调
    AIKIT_OnError  errorCB;   //错误回调
} AIKIT_Callbacks;

typedef struct {
    int         authType;     // 授权方式，0=设备级授权，1=应用级授权
    const char* appID;        // 应用id
    const char* apiKey;       // 应用key
    const char* apiSecret;    // 应用secret
    const char* workDir;      // sdk工作目录，需可读可写权限
    const char* resDir;       // 只读资源存放目录，需可读权限
    const char* licenseFile;  // 离线激活方式的授权文件存放路径，为空时需联网进行首次在线激活
    const char* batchID;      // 授权批次
    const char* UDID;         // 用户自定义设备标识            
    const char* cfgFile;      // 配置文件路径，包括文件名
} AIKIT_InitParam;

#endif // __AIKIT_BIZ_TYPE_H__

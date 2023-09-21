#ifndef __AIKIT_TYPE_H__
#define __AIKIT_TYPE_H__

#include <stddef.h>
#include <stdint.h>
#include "aikit_common.h"

#if defined(_MSC_VER)            /* Microsoft Visual C++ */
  #pragma pack(push, 8)
#elif defined(__BORLANDC__)      /* Borland C++ */
  #pragma option -a8
#elif defined(__WATCOMC__)       /* Watcom C++ */
  #pragma pack(push, 8)
#else                            /* Any other including Unix */
#endif

typedef enum _AIKIT_VarType {
    AIKIT_VarTypeString   =   0,      // 字符串型
    AIKIT_VarTypeInt      =   1,      // 整型
    AIKIT_VarTypeDouble   =   2,      // 实型
    AIKIT_VarTypeBool     =   3,      // 布尔类型
    AIKIT_VarTypeParamPtr =   4,      // 子参数类型
    AIKIT_VarTypeUnknown  =   -1      //
} AIKIT_VarType;

typedef enum _AIKIT_DataStatus {
    AIKIT_DataBegin    =  0,      // 首数据
    AIKIT_DataContinue =  1,      // 中间数据
    AIKIT_DataEnd      =  2,      // 尾数据
    AIKIT_DataOnce     =  3,      // 非会话单次输入输出
} AIKIT_DataStatus;

typedef enum _AIKIT_DataType {
    AIKIT_DataText    =   0,      // 文本数据
    AIKIT_DataAudio   =   1,      // 音频数据
    AIKIT_DataImage   =   2,      // 图像数据
    AIKIT_DataVideo   =   3,      // 视频数据
//    AIKIT_DataPer     =   4,      // 个性化数据
} AIKIT_DataType;

typedef enum {
    AIKIT_Event_UnKnown  = 0,
    AIKIT_Event_Start    = 1,  // 引擎计算开始事件
    AIKIT_Event_End      = 2,  // 引擎计算结束事件
    AIKIT_Event_Timeout  = 3,  // 引擎计算超时事件
    AIKIT_Event_Progress = 4,  // 引擎计算进度事件

    // 在线能力连接状态事件
    AIKIT_Event_Null = 10,          // 空连接状态
    AIKIT_Event_Init,               // 初始状态
    AIKIT_Event_Connecting,         // 正在连接
    AIKIT_Event_ConnTimeout,        // 连接超时
    AIKIT_Event_Failed,             // 连接失败
    AIKIT_Event_Connected,          // 已经连接
    AIKIT_Event_Error,              // 收发出错，意味着断开连接
    AIKIT_Event_Disconnected,       // 断开连接，一般指心跳超时，连接无错误
    AIKIT_Event_Closing,            // 正在关闭连接
    AIKIT_Event_Closed,             // 已经关闭连接
    AIKIT_Event_Responding,         // 网络连接正在响应
    AIKIT_Event_ResponseTimeout,    // 网络连接响应超时
    
    // VAD事件
    AIKIT_Event_VadBegin = 30, // VAD开始
    AIKIT_Event_VadEnd       // VAD结束
} AIKIT_EVENT;

typedef struct _AIKIT_BaseParam {
    struct _AIKIT_BaseParam *next;   // 链表指针
    const char *key;        // 数据标识
    void *value;            // 数据实体
    void* reserved;         // 预留字段
    int32_t len;            // 数据长度
    int32_t type;        // 变量类型，取值参见AIKIT_VarType
} AIKIT_BaseParam, *AIKIT_BaseParamPtr;      // 配置对复用该结构定义

typedef struct _AIKIT_BaseData {
    struct _AIKIT_BaseData *next;    // 链表指针
    AIKIT_BaseParam *desc;   // 数据描述，包含每个数据(audio/video/text/image)的所有特征参数数据(sample_rate,channels,data等)
    const char *key;        // 数据标识
    void *value;            // 数据实体
    void* reserved;         // 预留字段
    int32_t len;            // 数据长度
    int32_t type;           // 数据类型，取值参见AIKIT_DataType
    int32_t status;         // 数据状态，取值参见AIKIT_DataStatus
    int32_t from;           // 数据来源，取值参见AIKIT_DATA_PTR_TYPE
} AIKIT_BaseData, *AIKIT_BaseDataPtr;

typedef struct _AIKIT_CustomData {
    struct _AIKIT_CustomData*    next;       // 链表指针
    const char*         key;        // 数据标识
    void*               value;      // 数据内容
    void*               reserved;   // 预留字段
    int32_t             index;      // 数据索引,用户可自定义设置
    int32_t             len;        // 数据长度，type非DATA_PTR_FILE时本字段有效
    int32_t             from;       // 数据内容的类型，取值参见枚举AIKIT_DATA_PTR_TYPE
} AIKIT_CustomData, *AIKIT_CustomDataPtr;

/* Reset the structure packing alignments for different compilers. */
#if defined(_MSC_VER)            /* Microsoft Visual C++ */
#pragma pack(pop)
#elif defined(__BORLANDC__)      /* Borland C++ */
#pragma option -a.
#elif defined(__WATCOMC__)       /* Watcom C++ */
#pragma pack(pop)
#endif

#endif

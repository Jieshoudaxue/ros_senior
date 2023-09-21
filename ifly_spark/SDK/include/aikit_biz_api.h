//
// Created by xkzhang9 on 2020/10/16.
//

#ifndef AIKIT_BIZ_API_H
#define AIKIT_BIZ_API_H

#include "aikit_biz_type.h"
#include "aikit_biz_builder.h"

namespace AIKIT {

/**
 * SDK初始化函数用以初始化整个SDK
 * 初始化相关配置参数通过AIKIT_Congigurator配置
 * @return 结果错误码，0=成功
 */
AIKITAPI int32_t AIKIT_Init();


/**[deprecated]
 * SDK初始化函数用以初始化整个SDK
 * @param param  SDK配置参数
 * @return 结果错误码，0=成功
 */
AIKITAPI int32_t AIKIT_Init(AIKIT_InitParam* param);

/** 
 * SDK逆初始化函数用以释放SDK所占资源
 * @return 结果错误码，0=成功
 */
AIKITAPI int32_t AIKIT_UnInit();

/**
 * 注册回调函数用以返回执行结果
 * @param onOutput  能力实际输出回调
 * @param onEvent   能力执行事件回调
 * @param onError   能力执行错误回调
 * @return 结果错误码，0=成功
 */
AIKITAPI int32_t AIKIT_RegisterCallback(AIKIT_Callbacks cbs);

/**
 * 注册回调函数用以返回执行结果
 * @param ability   [in]  能力唯一标识
 * @param onOutput  能力实际输出回调
 * @param onEvent   能力执行事件回调
 * @param onError   能力执行错误回调
 * @return 结果错误码，0=成功
 */
AIKITAPI int32_t AIKIT_RegisterAbilityCallback(const char* ability, AIKIT_Callbacks cbs);

/**
 * 初始化能力引擎
 * @param ability   [in]  能力唯一标识
 * @param param     [in] 初始化参数
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t AIKIT_EngineInit(const char* ability, AIKIT_BizParam* param);

/**
 * 能力引擎逆初始化,释放能力及对应引擎占用所有资源
 * @param ability   [in]  能力唯一标识
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t AIKIT_EngineUnInit(const char* ability);

/**
 * 个性化数据预处理
 * @param   ability     [in]  能力唯一标识
 * @param   srcData     [in]  原始数据输入
 * @param   data        [out] 结果数据输出
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t AIKIT_PreProcess(const char* ability, AIKIT_CustomData* srcData, AIKIT_CustomData** data);

/**
 * 离线个性化数据加载
 * @param ability    能力唯一标识
 * @param data      个性化数据
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t AIKIT_LoadData(const char* ability, AIKIT_CustomData* data);

/**
 * 在线个性化数据上传
 * @param ability   能力唯一标识
 * @param params    个性化参数
 * @param data      个性化数据
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t AIKIT_LoadDataAsync(const char* ability, AIKIT_BizParam* params, AIKIT_InputData* data, void* usrContext, AIKIT_HANDLE** outHandle);

/**
 * 个性化数据查询
 * @param ability    能力唯一标识
 * @param data      个性化数据
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t AIKIT_QueryData(const char* ability, AIKIT_CustomData* data);

/**
 * @brief 可见即可说（AIUI定制接口）
 * @param  ability  能力唯一标识
 * @param params    个性化参数
 * @param data      个性化数据
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t AIKIT_LoadDataSpeakableAsync(const char* ability, AIKIT_BizParam* params, AIKIT_InputData* data, void* usrContext, AIKIT_HANDLE** outHandle);

/**
 * 个性化数据卸载
 * @param  ability      能力唯一标识
 * @param  key          个性化数据唯一标识
 * @param  index        个性化数据索引
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t AIKIT_UnLoadData(const char* ability, const char* key, int index);

/**
 * 指定要使用的个性化数据集合，未调用，则默认使用所有AIKIT_LoadData加载的数据
 * 可调用多次以使用不同key集合
 * @param  abilityId  能力唯一标识
 * @param  key        个性化数据唯一标识数组
 * @param  index      个性化数据索引数组
 * @param  count      个性化数据索引数组成员个数
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t  AIKIT_SpecifyDataSet(const char* ability, const char* key, int index[], int count);

/**
 * 启动one-shot模式能力同步模式调用
 * @param   ability    能力唯一标识
 * @param   param      能力参数
 * @param   inputData  能力数据输入
 * @param   outputData 能力数据输出
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t AIKIT_OneShot(const char* ability, AIKIT_BizParam* params, AIKIT_InputData* inputData, AIKIT_OutputData** outputData);

/**
 * 启动one-shot模式能力异步模式调用
 * @param ability    能力唯一标识d
 * @param param      能力参数
 * @param data       能力数据输入
 * @param usrContext 上下文指针
 * @param outHandle  生成的引擎会话句柄
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t AIKIT_OneShotAsync(const char* ability, AIKIT_BizParam* params, AIKIT_InputData* data, void* usrContext, AIKIT_HANDLE** outHandle);

/**
 * 启动会话模式能力调用实例，若引擎未初始化，则内部首先完成引擎初始化
 * @param ability   能力唯一标识
 * @param len       ability长度
 * @param param     初始化参数
 * @param usrContext上下文指针
 * @param outHandle 生成的引擎会话句柄
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t AIKIT_Start(const char* ability, AIKIT_BizParam* param, void* usrContext, AIKIT_HANDLE** outHandle);

/**
 * 会话模式输入数据
 * @param handle    会话实例句柄
 * @param input     输入数据
 * @return 结果错误码，0=成功
 */
AIKITAPI int32_t AIKIT_Write(AIKIT_HANDLE* handle, AIKIT_InputData* input);

/**
 * 会话模式同步读取数据
 * @param handle    会话实例句柄
 * @param output     输入数据
 * @return 结果错误码，0=成功
 * @note  output内存由SDK自行维护,无需主动释放
 */
AIKITAPI int32_t AIKIT_Read(AIKIT_HANDLE* handle, AIKIT_OutputData** output);

/**
 * 结束会话实例
 * @param handle    会话实例句柄
 * @return 结果错误码，0=成功
 */
AIKITAPI int32_t AIKIT_End(AIKIT_HANDLE* handle);


/**
 * 释放能力占用资源,注意不会释放引擎实例
 * 若要释放能力及能力所有资源，需调用AIKIT_EngineUnInit()
 * @param   ability    能力唯一标识
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t AIKIT_FreeAbility(const char* ability);

/**
 * 设置日志级别
 * @param  level    日志级别
 * @return 错误码 0=成功，其他表示失败
*/
AIKITAPI int32_t AIKIT_SetLogLevel(int32_t level);

/**
 * 设置日志输出模式
 * @param  mode    日志输出模式
 * @return 错误码 0=成功，其他表示失败
*/
AIKITAPI int32_t AIKIT_SetLogMode(int32_t mode);

/**
 * 输出模式为文件时，设置日志文件名称
 * @param  path    日志名称
 * @return 错误码 0=成功，其他表示失败
*/
AIKITAPI int32_t AIKIT_SetLogPath(const char* path);

/** 
 * 获取设备ID
 * @param deviceID    设备指纹输出字符串
 * @return 结果错误码，0=成功
 */
AIKITAPI int32_t AIKIT_GetDeviceID(const char** deviceID);

/** 
 * 设置授权更新间隔，单位为秒，默认为300秒
 * AIKIT_Init前设置
 * @param interval    间隔秒数
 * @return 结果错误码，0=成功
 */
AIKITAPI int32_t AIKIT_SetAuthCheckInterval(uint32_t interval);

/** 
 * 强制更新授权
 * 注意：注意需在AIKIT_Init调用成功后，方可调用
 * @param timeout    超时时间 单位为秒 
 * @return 结果错误码，0=成功
 */
AIKITAPI int32_t AIKIT_UpdateAuth(uint32_t timeout);

/** 
 * 获取能力授权剩余秒数
 * 能力输入参数为空时，默认返回最接近授权过期的能力剩余秒数
 * 注意：注意需在AIKIT_Init调用成功后，方可调用
 * @param leftTime   返回的能力授权剩余秒数，0 表示永久授权， 负值表示已过期的秒数
 * @param ability    能力id标识
 * @return 返回调用结果，0 = 成功， 其他值表示调用失败
 */
AIKITAPI int AIKIT_GetAuthLeftTime(int64_t& leftTime,int64_t& authEndTime, const char* ability = nullptr);

/** 
 * 获取SDK版本号
 * @return SDK版本号
 */
AIKITAPI const char* AIKIT_GetVersion();

/**
 * @brief 获取能力对应的引擎版本
 * 
 * @param ability 能力唯一标识
 * @return const* 引擎版本号
 */
AIKITAPI const char* AIKIT_GetEngineVersion(const char* ability);

/**
 * 本地日志是否开启
 * @param open
 * @return
 */
AIKITAPI int32_t AIKIT_SetILogOpen(bool open);

/**
 * 本地日志最大存储个数（【1，300】）
 * @param count
 * @return
 */
AIKITAPI int32_t AIKIT_SetILogMaxCount(uint32_t count);

/**
 * 设置单日志文件大小（(0，10M】）
 * @param bytes
 * @return
 */
AIKITAPI int32_t AIKIT_SetILogMaxSize(long long bytes);

/**
 * 设置SDK相关配置
 * @param key    参数名字
 * @param value  参数值
 * @return 结果错误码，0=成功
 */
AIKITAPI int32_t AIKIT_SetConfig(const char* key, const void* value);

/**
 * 设置SDK内存模式
 * @param ability 能力id
 * @param mode    模式，取值参见 AIKIT_MEMORY_MODE
 * @return AIKITAPI 
 */
AIKITAPI int32_t AIKIT_SetMemoryMode(const char* ability,int32_t mode);


/**自2.1.6版本开始已废弃接口，当前仅保留空实现接口声明
 * 自2.1.6版本开始AIKIT_OneShot响应数据由SDK内部自己维护，无需用户维护
 * 同步接口响应数据缓存释放接口
 * @param  outputData    由同步接口AIKIT_OneShot获取的响应结果数据
 */
AIKITAPI int32_t AIKIT_Free(AIKIT_OutputData** outputData);

/**自2.1.6版本开始已废弃接口，由AIKIT_FreeAbility替代
 * 释放能力占用资源
 * @param   ability    能力唯一标识
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t AIKIT_GC(const char* ability);

/**
 * 设置日志级别，模式，以及保存路径,旧版日志接口，不推荐使用
 * @param  level    日志级别
 * @param  mode     日志输出模式
 * @param  level    输出模式为文件时的文件名称
 * @return 错误码 0=成功，其他表示失败
 */
AIKITAPI int32_t AIKIT_SetLogInfo(int32_t level, int32_t mode, const char* path);

/**
 * 获取能力授权剩余秒数
 * 能力输入参数为空时，默认返回最接近授权过期的能力剩余秒数
 * 注意：注意需在AEE_Init调用成功后，方可调用
 * @param leftTime   返回的能力授权剩余秒数，0 表示永久授权， 负值表示已过期的秒数
 * @param ability    能力id标识
 * @return 返回调用结果，0 = 成功， 其他值表示调用失败
 */
AIKITAPI int AIKIT_GetAuthLeftTime(int64_t& leftTime, const char* ability = nullptr);

}  // namespace AIKIT

#endif  //AIKIT_BIZ_API_H

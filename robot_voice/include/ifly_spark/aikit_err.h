/*
 * @Description: 
 * @version: 
 * @Author: rfge
 * @Date: 2020-11-01 17:56:53
 * @LastEditors: rfge
 * @LastEditTime: 2020-12-14 14:26:44
 */
//
// Created by xkzhang9 on 2020/6/10.
//

#ifndef AIKIT_ERR_H
#define AIKIT_ERR_H

typedef enum {
    AIKIT_ERR_AUTH     = 18000,                // 授权部分
    AIKIT_ERR_RES      = AIKIT_ERR_AUTH + 100,  // 资源部分
    AIKIT_ERR_ENG      = AIKIT_ERR_AUTH + 200,  // 引擎部分
    AIKIT_ERR_SDK      = AIKIT_ERR_AUTH + 300,  // SDK部分
    AIKIT_ERR_SYS      = AIKIT_ERR_AUTH + 400,  // 系统部分
    AIKIT_ERR_PARAM    = AIKIT_ERR_AUTH + 500,  // 参数部分
    AIKIT_ERR_PROTOCOL = AIKIT_ERR_AUTH + 600,  // 协议部分
    AIKIT_ERR_CLOUD    = AIKIT_ERR_AUTH + 700,  // 云端错误
    AIKIT_ERR_LOCAL_NET= AIKIT_ERR_AUTH + 800,  //本地网络网络
    AIKIT_ERR_VMS      = AIKIT_ERR_AUTH + 900,  // 虚拟人错误
    AIKIT_ERR_SPARK    = AIKIT_ERR_AUTH + 950,  // spark大模型错误
    AIKIT_ERR_OTHER = 0xFF00  // 其他
} AIKIT_ERR_SECTION;

typedef enum {
    AIKIT_ERR_SUCCESS        = 0,  // 操作成功
    AIKIT_ERR_GENERAL_FAILED = 1,  // 一般错误

    AIKIT_ERR_AUTH_LICENSE_NOT_FOUND    = AIKIT_ERR_AUTH + 0,  // 18000 本地license文件不存在
    AIKIT_ERR_AUTH_LICENSE_FILE_INVALID = AIKIT_ERR_AUTH + 1,  // 18001 授权文件内容非法
    AIKIT_ERR_AUTH_LICENSE_PARSE_FAILED = AIKIT_ERR_AUTH + 2,  // 18002 授权文件解析失败
    AIKIT_ERR_AUTH_PAYLOAD_DEFECT       = AIKIT_ERR_AUTH + 3,  // 18003 payload内容缺失
    AIKIT_ERR_AUTH_SIGN_DEFECT          = AIKIT_ERR_AUTH + 4,  // 18004 signature内容缺失
    AIKIT_ERR_AUTH_EXPIRED              = AIKIT_ERR_AUTH + 5,  // 18005 授权已过期
    AIKIT_ERR_AUTH_TIME_ERROR           = AIKIT_ERR_AUTH + 6,  // 18006 授权时间错误，比正常时间慢30分钟以上
    AIKIT_ERR_AUTH_APP_NOT_MATCH        = AIKIT_ERR_AUTH + 7,  // 18007 授权应用不匹配（apiKey、apiSecret）
    AIKIT_ERR_AUTH_LICENSE_EXPIRED      = AIKIT_ERR_AUTH + 8,  // 18008 授权文件激活过期
    AIKIT_ERR_AUTH_NULL_APP_PTR         = AIKIT_ERR_AUTH + 9,  // 18009 授权app信息指针为空
    AIKIT_ERR_AUTH_PLATFORM_NOT_MATCH   = AIKIT_ERR_AUTH + 10, // 18010 离线授权激活文件指定平台与设备平台不匹配
    AIKIT_ERR_AUTH_ARCH_NOT_MATCH       = AIKIT_ERR_AUTH + 11, // 18011 离线授权激活文件指定架构与设备cpu架构不匹配
    AIKIT_ERR_AUTH_WRONG_LICENSE_NUM    = AIKIT_ERR_AUTH + 12, // 18012 离线授权激活文件中包含License个数异常
    AIKIT_ERR_AUTH_DEVICE_NOT_FOUND     = AIKIT_ERR_AUTH + 13, // 18013 离线授权激活文件中未找到当前设备
    AIKIT_ERR_AUTH_LEVEL_NOT_VALID      = AIKIT_ERR_AUTH + 14, // 18014 离线授权激活文件中设备指纹安全等级非法
    AIKIT_ERR_AUTH_HARDWARE_FAILED      = AIKIT_ERR_AUTH + 15, // 18015 硬件授权验证失败
    AIKIT_ERR_AUTH_OFFLINE_PROT_INVALID = AIKIT_ERR_AUTH + 16, // 18016 离线授权激活文件内容非法
    AIKIT_ERR_AUTH_HEADER_INVALID       = AIKIT_ERR_AUTH + 17, // 18017 离线授权激活文件中协议头非法
    AIKIT_ERR_AUTH_PART_COUNT_INVALID   = AIKIT_ERR_AUTH + 18, // 18018 离线授权激活文件中指纹组成项个数为0
    AIKIT_ERR_AUTH_RESOURCE_EXPIREd     = AIKIT_ERR_AUTH + 19, // 18019 资源已过期

    AIKIT_ERR_RES_VERIFY_FAILED = AIKIT_ERR_RES + 0,  // 18100 资源鉴权失败
    AIKIT_ERR_RES_INVALID_HEADER= AIKIT_ERR_RES + 1,  // 18101 资源格式解析失败
    AIKIT_ERR_RES_NOT_MATCH     = AIKIT_ERR_RES + 2,  // 18102 资源(与引擎)不匹配
    AIKIT_ERR_RES_NULL_PTR      = AIKIT_ERR_RES + 3,  // 18103 资源参数不存在（指针为NULL）
    AIKIT_ERR_RES_OPEN_FAILED   = AIKIT_ERR_RES + 4,  // 18104 资源路径打开失败
    AIKIT_ERR_RES_LOAD_FAILED   = AIKIT_ERR_RES + 5,  // 18105 资源加载失败，workDir内未找到对应资源
    AIKIT_ERR_RES_UNLOAD_FAILED = AIKIT_ERR_RES + 6,  // 18106 资源卸载失败, 卸载的资源未加载过

    AIKIT_ERR_ENG_VERIFY_FAILED   = AIKIT_ERR_ENG + 0,  // 18200 引擎鉴权失败
    AIKIT_ERR_ENG_LOAD_FAILED     = AIKIT_ERR_ENG + 1,  // 18201 引擎动态加载失败
    AIKIT_ERR_ENG_NOT_INITED      = AIKIT_ERR_ENG + 2,  // 18202 引擎未初始化
    AIKIT_ERR_ENG_API_NOT_SUPPORT = AIKIT_ERR_ENG + 3,  // 18203 引擎不支持该接口调用
    AIKIT_ERR_ENG_NULL_CREATE_PTR = AIKIT_ERR_ENG + 4,  // 18204 引擎craete函数指针为空

    AIKIT_ERR_SDK_INVALID                  = AIKIT_ERR_SDK + 0,   // 18300 sdk不可用
    AIKIT_ERR_SDK_NOT_INITED               = AIKIT_ERR_SDK + 1,   // 18301 sdk没有初始化
    AIKIT_ERR_SDK_INIT_FAILED              = AIKIT_ERR_SDK + 2,   // 18302 sdk初始化失败
    AIKIT_ERR_SDK_ALREADY_INIT             = AIKIT_ERR_SDK + 3,   // 18303 sdk已经初始化
    AIKIT_ERR_SDK_INVALID_PARAM            = AIKIT_ERR_SDK + 4,   // 18304 sdk不合法参数
    AIKIT_ERR_SDK_NULL_SESSION_HANDLE      = AIKIT_ERR_SDK + 5,   // 18305 sdk会话handle为空
    AIKIT_ERR_SDK_SESSION_NOT_FOUND        = AIKIT_ERR_SDK + 6,   // 18306 sdk会话未找到
    AIKIT_ERR_SDK_SESSION_ALREADY_END      = AIKIT_ERR_SDK + 7,   // 18307 sdk会话重复终止
    AIKIT_ERR_SDK_TIMEOUT                  = AIKIT_ERR_SDK + 8,   // 18308 超时错误
    AIKIT_ERR_SDK_INITING                  = AIKIT_ERR_SDK + 9,   // 18309 sdk正在初始化中
    AIKIT_ERR_SDK_SESSEION_ALREAY_START    = AIKIT_ERR_SDK + 10,  // 18310 sdk会话重复开启
    AIKIT_ERR_SDK_CONCURRENT_OVERFLOW      = AIKIT_ERR_SDK + 11,  // 18311 sdk同一能力并发路数超出最大限制

    AIKIT_ERR_SYS_WORK_DIR_ILLEGAL         = AIKIT_ERR_SYS + 0,  // 18400 工作目录无写权限
    AIKIT_ERR_SYS_DEVICE_UNKNOWN           = AIKIT_ERR_SYS + 1,  // 18401 设备指纹获取失败，设备未知
    AIKIT_ERR_SYS_FILE_OPEN_FAILED         = AIKIT_ERR_SYS + 2,  // 18402 文件打开失败
    AIKIT_ERR_SYS_MEM_ALLOC_FAILED         = AIKIT_ERR_SYS + 3,  // 18403 内存分配失败
    AIKIT_ERR_SYS_DEVICE_COMPARE_FAILED    = AIKIT_ERR_SYS + 4,  // 18404 设备指纹比较失败

    AIKIT_ERR_PARAM_NOT_FOUND                = AIKIT_ERR_PARAM + 0,   // 18500 未找到该参数key
    AIKIT_ERR_PARAM_OVERFLOW                 = AIKIT_ERR_PARAM + 1,   // 18501 参数范围溢出，不满足约束条件
    AIKIT_ERR_PARAM_NULL_INIT_PARAM_PTR      = AIKIT_ERR_PARAM + 2,   // 18502 sdk初始化参数为空
    AIKIT_ERR_PARAM_NULL_APPID_PTR           = AIKIT_ERR_PARAM + 3,   // 18503 sdk初始化参数中appid为空
    AIKIT_ERR_PARAM_NULL_APIKEY_PTR          = AIKIT_ERR_PARAM + 4,   // 18504 sdk初始化参数中apiKey为空
    AIKIT_ERR_PARAM_NULL_APISECRET_PTR       = AIKIT_ERR_PARAM + 5,   // 18505 sdk初始化参数中apiSecret为空
    AIKIT_ERR_PARAM_NULL_ABILITY_PTR         = AIKIT_ERR_PARAM + 6,   // 18506 ability参数为空
    AIKIT_ERR_PARAM_NULL_INPUT_PTR           = AIKIT_ERR_PARAM + 7,   // 18507 input参数为空
    AIKIT_ERR_PARAM_DATA_KEY_NOT_EXIST       = AIKIT_ERR_PARAM + 8,   // 18508 输入数据参数Key不存在
    AIKIT_ERR_PARAM_REQUIRED_MISSED          = AIKIT_ERR_PARAM + 9,   // 18509 必填参数缺失
    AIKIT_ERR_PARAM_NULL_OUTPUT_PTR          = AIKIT_ERR_PARAM + 10,  // 18510 output参数缺失
    
    AIKIT_ERR_CODEC_NOT_SUPPORT              = AIKIT_ERR_PARAM + 20,  // 18520 不支持的编解码类型
    AIKIT_ERR_CODEC_NULL_PTR                 = AIKIT_ERR_PARAM + 21,  // 18521 编解码handle指针为空
    AIKIT_ERR_CODEC_MODULE_MISSED            = AIKIT_ERR_PARAM + 22,  // 18522 编解码模块条件编译未打开
    AIKIT_ERR_CODEC_ENCODE_FAIL              = AIKIT_ERR_PARAM + 23,  // 18523 编码错误
    AIKIT_ERR_CODEC_DECODE_FAIL              = AIKIT_ERR_PARAM + 24,  // 18524 解码错误

    AIKIT_ERR_VAD_RESPONSE_TIMEOUT           = AIKIT_ERR_PARAM + 30,  // VAD静音超时

    AIKIT_ERR_PROTOCOL_TIMESTAMP_MISSING        = AIKIT_ERR_PROTOCOL + 0, // 18600 协议中时间戳字段缺失
    AIKIT_ERR_PROTOCOL_ABILITY_NOT_FOUND        = AIKIT_ERR_PROTOCOL + 1, // 18601 协议中未找到该能力ID
    AIKIT_ERR_PROTOCOL_RESOURCE_NOT_FOUND       = AIKIT_ERR_PROTOCOL + 2, // 18602 协议中未找到该资源ID
    AIKIT_ERR_PROTOCOL_ENGINE_NOT_FOUND         = AIKIT_ERR_PROTOCOL + 3, // 18603 协议中未找到该引擎ID
    AIKIT_ERR_PROTOCOL_ZERO_ENGINE_NUM          = AIKIT_ERR_PROTOCOL + 4, // 18604 协议中引擎个数为0
    AIKIT_ERR_PROTOCOL_NOT_LOADED               = AIKIT_ERR_PROTOCOL + 5, // 18605 协议未被初始化解析
    AIKIT_ERR_PROTOCOL_INTERFACE_TYPE_NOT_MATCH = AIKIT_ERR_PROTOCOL + 6, // 18606 协议能力接口类型不匹配
    AIKIT_ERR_PROTOCOL_TEMP_VERIFY_FAILED       = AIKIT_ERR_PROTOCOL + 7, // 18607 预置协议解析失败

    AIKIT_ERR_CLOUD_GENERAL_FAILED          = AIKIT_ERR_CLOUD + 0,  // 18700 通用网络错误
    AIKIT_ERR_CLOUD_CONNECT_FAILED          = AIKIT_ERR_CLOUD + 1,  // 18701 网路不通
    AIKIT_ERR_CLOUD_403                     = AIKIT_ERR_CLOUD + 2,  // 18702 网关检查不过
    AIKIT_ERR_CLOUD_WRONG_RSP_FORMAT        = AIKIT_ERR_CLOUD + 3,  // 18703 云端响应格式不对
    AIKIT_ERR_CLOUD_APP_NOT_FOUND           = AIKIT_ERR_CLOUD + 4,  // 18704 应用未注册
    AIKIT_ERR_CLOUD_APP_CHECK_FAILED        = AIKIT_ERR_CLOUD + 5,  // 18705 应用apiKey&&apiSecret校验失败
    AIKIT_ERR_CLOUD_WRONG_ARCHITECT         = AIKIT_ERR_CLOUD + 6,  // 18706 引擎不支持的平台架构
    AIKIT_ERR_CLOUD_AUTH_EXPIRED            = AIKIT_ERR_CLOUD + 7,  // 18707 授权已过期
    AIKIT_ERR_CLOUD_AUTH_FULL               = AIKIT_ERR_CLOUD + 8,  // 18708 授权数量已满
    AIKIT_ERR_CLOUD_ABILITY_NOT_FOUND       = AIKIT_ERR_CLOUD + 9,  // 18709 未找到该app绑定的能力
    AIKIT_ERR_CLOUD_RESOURCE_NOT_FOUND      = AIKIT_ERR_CLOUD + 10, // 18710 未找到该app绑定的能力资源
    AIKIT_ERR_CLOUD_JSON_PARSE_FAILED       = AIKIT_ERR_CLOUD + 11, // 18711 JSON操作失败
    AIKIT_ERR_CLOUD_404                     = AIKIT_ERR_CLOUD + 12, // 18712 http 404错误
    AIKIT_ERR_CLOUD_LEVEL_NOT_MATCH         = AIKIT_ERR_CLOUD + 13, // 18713 设备指纹安全等级不匹配
    AIKIT_ERR_CLOUD_401                     = AIKIT_ERR_CLOUD + 14, // 18714 用户没有访问权限，需要进行身份认证
    AIKIT_ERR_CLOUD_SDK_NOT_FOUND           = AIKIT_ERR_CLOUD + 15, // 18715 未找到该SDK ID
    AIKIT_ERR_CLOUD_ABILITYS_NOT_FOUND      = AIKIT_ERR_CLOUD + 16, // 18716 未找到该组合能力集合
    AIKIT_ERR_CLOUD_ABILITY_NOT_ENOUGH      = AIKIT_ERR_CLOUD + 17, // 18717 SDK组合能力授权不足
    AIKIT_ERR_CLOUD_APP_SIG_INVALID         = AIKIT_ERR_CLOUD + 18, // 18718 无效授权应用签名
    AIKIT_ERR_CLOUD_APP_SIG_NOT_UNIQUE      = AIKIT_ERR_CLOUD + 19, // 18719 应用签名不唯一
    AIKIT_ERR_CLOUD_SCHEMA_INVALID          = AIKIT_ERR_CLOUD + 20, // 18720 能力schema不可用
    AIKIT_ERR_CLOUD_TEMPLATE_NOT_FOUND      = AIKIT_ERR_CLOUD + 21, // 18721 竞争授权: 未找到能力集模板
    AIKIT_ERR_CLOUD_ABILITY_NOT_IN_TEMPLATE = AIKIT_ERR_CLOUD + 22, // 18722 竞争授权: 能力不在模板能力集模板中

    AIKIT_ERR_LOCAL_NET_CONNECT_FAILED   = AIKIT_ERR_LOCAL_NET + 1, // 18801 连接建立出错
    AIKIT_ERR_LOCAL_NET_RES_WAIT_TIMEOUT = AIKIT_ERR_LOCAL_NET + 2, // 18802 结果等待超时
    AIKIT_ERR_LOCAL_NET_CONNECT_ERROR    = AIKIT_ERR_LOCAL_NET + 3, // 18803 连接状态异常

    AIKIT_ERR_VMS_START_ERROR     = AIKIT_ERR_VMS + 1, // 18901虚拟人启动错误
    AIKIT_ERR_VMS_STOP_ERROR      = AIKIT_ERR_VMS + 2, // 18902虚拟人结束错误
    AIKIT_ERR_VMS_DRIVE_ERROR     = AIKIT_ERR_VMS + 3, // 18903虚拟人驱动错误
    AIKIT_ERR_VMS_HEARTBEAT_ERROR = AIKIT_ERR_VMS + 4, // 18904虚拟人心跳报错
    AIKIT_ERR_VMS_TIMEOUT         = AIKIT_ERR_VMS + 5, // 18905虚拟人服务超时
    AIKIT_ERR_VMS_OTHER_ERROR     = AIKIT_ERR_VMS + 6, // 18906虚拟人通用报错

    AIKIT_ERR_SPARK_REQUEST_UNORDERED = AIKIT_ERR_SPARK + 1, // 18951 同一流式大模型会话，禁止并发交互请求
    AIKIT_ERR_SPARK_TEXT_INVALID = AIKIT_ERR_SPARK + 2       // 18952 输入文本格式或内容非法

} AIKIT_ERR;

typedef enum { AIKIT_ERR_TYPE_AUTH = 0, AIKIT_ERR_TYPE_HTTP = 1 } AIKIT_ERR_TYPE;

#endif  //AIKIT_ERR_H

#ifndef AIKIT_BIZ_CONFIG_H
#define AIKIT_BIZ_CONFIG_H
#include <iostream>
#include "aikit_biz_type.h"
namespace AIKIT {

class ConfigBuilder;
class AppBuilder;
class AuthBuilder;
class LogBuilder;
class CodecBuilder;

class AIKITAPI AIKIT_Configurator {
public:
    static ConfigBuilder& builder();
};

class AIKITAPI ConfigBuilder {
public:
    /**
     * @brief 用户app信息配置
     */
    AppBuilder& app();

    /**
     * @brief 授权相关配置
     */
    AuthBuilder& auth();

    /**
     * @brief 日志相关配置
     */
    LogBuilder& log();

    /**
     * @brief 音视频编解码相关配置
     */
    CodecBuilder&   codec();
};

class AIKITAPI AppBuilder : public ConfigBuilder {
public:
    /**
     * @brief 配置应用id
     */
    AppBuilder& appID(const char* appID);

    /**
     * @brief 配置应用key
     */
    AppBuilder& apiKey(const char* apiKey);

    /**
     * @brief 配置应用secret
     */
    AppBuilder& apiSecret(const char* apiSecret);

    /**
     * @brief 配置sdk工作目录，需可读可写权限
     */
    AppBuilder& workDir(const char* workDir);

     /**
     * @brief 配置只读资源存放目录, 需可读权限
     */
    AppBuilder& resDir(const char* resDir);

    /**
     * @brief 配置文件路径，包括文件名
     */
    AppBuilder& cfgFile(const char* cfgFile);
};

class AIKITAPI AuthBuilder : public ConfigBuilder {
public:
    /**
     * @brief 配置授权方式 0=设备级授权，1=应用级授权
     */
    AuthBuilder& authType(int authType);

    /**
     * @brief 配置离线激活方式的授权文件路径，为空时需联网进行首次在线激活
     */
    AuthBuilder& licenseFile(const char* licenseFile);

    /**
     * @brief 配置授权渠道Id
     */
    AuthBuilder& channelID(const char* channelID);

    /**
     * @brief 配置用户自定义设备标识
     */
    AuthBuilder& UDID(const char* UDID);

    /**
     * @brief 配置授权离线能力，如需配置多个能力，可用";"分隔开, 如"xxx;xxx"
     */
    AuthBuilder& ability(const char* ability);

    /**
     * @brief 配置单个在线能力id及对应请求地址，如需配置多个能力，可多次调用
     * url格式参考：https://cn-huadong-1.xf-yun.com/v1/private/xxx:443
     */
    AuthBuilder& abilityURL(const char* ability, const char* url);

};


/**
 * @brief 配置SDK日志信息
 */
class AIKITAPI LogBuilder : public ConfigBuilder {
public:
    /**
     * @brief 配置日志等级
     */
    LogBuilder& logLevel(int32_t level);

     /**
     * @brief 配置日志输出模式
     */
    LogBuilder& logMode(int32_t mode);

    /**
     * @brief 配置日志文件保存文件
     */
    LogBuilder& logPath(const char* path);
   
};

/**
 * @brief 配置SDK音视频编解码方式
 */
class AIKITAPI CodecBuilder : public ConfigBuilder {
public:
    /**
     * @brief 配置全局音频编码选项
     */
    CodecBuilder& audioEncoding(const char* encodingType);

    /**
     * @brief 配置能力级音频编码选项
     */
    CodecBuilder& audioEncoding(const char* abilityID, const char* encodingType);
    /**
     * @brief 配置全局音频解码选项
     */
    CodecBuilder& audioDecoding(const char* encodingType);

    /**
     * @brief 配置能力级音频解码选项
     */
    CodecBuilder& audioDecoding(const char* abilityID, const char* encodingType);
};

} // end of namespace AIKIT
#endif
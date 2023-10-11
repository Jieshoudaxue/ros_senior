#ifndef AIKIT_SPARK_H
#define AIKIT_SPARK_H
#include "aikit_biz_api.h"

namespace AIKIT {


class AIKITAPI ChatParam {
public:
    static ChatParam* builder();
   
    virtual ~ChatParam();

    /**
     * @brief 配置授权的用户id，用于关联用户交互的上下文
     */
    virtual ChatParam* uid(const char* uid) = 0;

    /**
     * @brief 配置chat领域信息
     */
    virtual ChatParam* domain(const char* domain) = 0;

    /**
     * @brief 配置内容审核策略
     */
    virtual ChatParam* auditing(const char* auditing) = 0;

    /**
     * @brief 配置关联会话chat id标识，需要保障⽤户下唯⼀
     */
    virtual ChatParam* chatID(const char* chatID) = 0;

    /**
     * @brief 配置核采样阈值，默认值0.5，向上调整可以增加结果的随机程度
     */
    virtual ChatParam* temperature(const float& temperature) = 0;

    /**
     * @brief 配置从k个候选中随机选择⼀个（⾮等概率），默认值4，取值范围1-99
     */
    virtual ChatParam* topK(const int& topK) = 0;

    /**
     * @brief 配置回答的tokens的最大长度，默认值2048
     */
    virtual ChatParam* maxToken(const int& maxToken) = 0;

    
    /**
     * @brief 配置chat服务域名地址
     */
    virtual ChatParam* url(const char* url) = 0;

     /**
     * @brief 配置chat扩展功能参数
     */
    virtual ChatParam* param(const char* key, const char* value) = 0;
    virtual ChatParam* param(const char* key, int value)                          = 0;
    virtual ChatParam* param(const char* key, double value)                       = 0;
    virtual ChatParam* param(const char* key, bool value)                         = 0;

};

using AIChat_Handle = AIKIT_HANDLE;

typedef void (*onChatOutput)(AIChat_Handle* handle, const char* role, const char* content, const int& index);
typedef void (*onChatToken)(AIChat_Handle* handle, const int& completionTokens, const int& promptTokens, const int& totalTokens);
typedef void (*onChatError)(AIChat_Handle* handle, const int& err, const char* errDesc);
typedef struct {
    onChatOutput outputCB;     //输出回调
    onChatToken  tokenCB;      //token计算信息回调
    onChatError  errorCB;      //错误回调
} AIKIT_ChatCBS;

AIKITAPI int32_t AIKIT_ChatCallback(const AIKIT_ChatCBS& cbs);
//异步chat
AIKITAPI int32_t AIKIT_AsyncChat(const ChatParam* params, const char* inputText, void* usrContext);



AIKITAPI int32_t AIKIT_Start(const ChatParam* params, void* usrContext, AIChat_Handle** outHandle);
AIKITAPI int32_t AIKIT_Write(AIChat_Handle* handle, const char* inputText);

//
//AIKITAPI int32_t AIKIT_End(AIChat_Handle* handle);


} // end of namespace AIKIT
#endif
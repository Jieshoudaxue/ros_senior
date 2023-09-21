# AIKit-Spark Linux 接入文档

# 1. 功能特性描述

- 高效接入：SDK统一封装鉴权模块，接口简单最快三步完成SDK集成接入
- 稳定可靠： 统一连接池保障连接时效性，httpDNS保障请求入口高可用性
- 配套完善：支持多路并发用户回调上下文绑定，交互历史管理及排障日志回传收集
- 多平台兼容：覆盖Windows，Linux，Android，iOS以及其他交叉编译平台

# 2. SDK集成指南

## 2.1 SDK包目录结构说明

```c++
── SDK                          // SDK 头文件及动态库
│   ├── include                 // SDK所需头文件列表
│   │   ├── aikit_biz_api.h
│   │   ├── aikit_biz_builder.h
│   │   ├── aikit_biz_obsolete_builder.h
│   │   ├── aikit_biz_type.h
│   │   ├── aikit_common.h
│   │   ├── aikit_spark_api.h   // Spark核心头文件
│   │   └── aikit_type.h
│   └── libs                    // SDK动态库目录
│       └── x64
│           └── libaikit.so
── demo                          // demo目录
│   ├── build.sh                 // demo编译脚本
│   ├── run.sh                   // demo运行脚本
│   ├── README.md                // demo运行说明
│   ├── sparkDemo.cpp            // demo源码
│   └── sparkDemo.h
── docs                          // SDK相关文档
```



## 2.2 demo运行步骤

1. 进入sdk包内,填写sparkDemo.cpp中appID、apiKey、apiSecret字段，demo目录运行build.sh 完成demo编译，生成sparkDemo可执行文件

2. 运行run.sh， 即可进行demo交互体验

3. demo编译环境建议如下：

   | 编译环境信息   | 版本                |
   | -------------- | ------------------- |
   | **编译系统**   | Ubuntu 18.04 x86_64 |
   | **编译器版本** | gcc 7.5.0           |
   | **链接器版本** | ld 2.30             |

   

## 2.3 项目集成步骤

1. 导入SDK目录内include头文件和libs下对应libaikit.so动态库
2. 集成SDK相关接口，接口调用流程及详细说明如下：

##  3. 详细API介绍

**Demo说明**（C++语言）：

```c++
static void onChatToken(AIChat_Handle* handle, const int& completionTokens, const int& promptTokens, const int& totalTokens) {
    cout<<" promptTokens:"<<promptTokens<<" totalTokens:"<<totalTokens<<endl;
}

static void onChatOutput(AIChat_Handle* handle, const char* role, const char* content, const int& index) {
    cout<<"role: "<<role<<" :content: "<<content<<endl;  
}

static void onChatError(AIChat_Handle* handle,const int& err, const char* errDesc) {
    cout<<"chatOnError: err:"<<err<<" ,desc:"<<errDesc<<endl;
}

int main() {
    // SDK全局初始化参数配置
    AIKIT_InitParam initParam{};
    initParam.appID = "yourAppid";
    initParam.apiKey = "yourKey";
    initParam.apiSecret = "yourSecret";
    // SDK全局初始化
    int ret = AIKIT_Init(&initParam);
    if(ret != 0) {
        printf("AIKIT_Init failed:%d\n",ret);
        return 0;
    }
    // 异步回调函数注册
    AIKIT_ChatCallback({ onChatOutput, onChatToken, onChatError });
	// 配置请求参数
   	ChatParam* params = ChatParam::builder();
    params->uid("xxxid")
          ->domain("generalv2")
          ->maxToken(2048)
          ->url("ws://spark-api.xf-yun.com/v2.1/chat");
    // 设置输入文本
	string inputText="你好";
    // 发起交互请求
    ret = AIKIT_AsyncChat(params, inputText.c_str() , nullptr);
    if(ret != 0){
        printf("AIKIT_AsyncChat failed:%d\n",ret);
        return ret;
    }
    // 等待异步回调
    // ........
    
    // 应用退出，逆初始化SDK
    AIKIT_UnInit();
    return 0;
}
```

Demo中常见设置参照下方接口文档进行开发。

### 3.1 SDK初始化接口

```c++
AIKITAPI int32_t AIKIT_Init(AIKIT_InitParam* param);
```

**说明**：返回 0表示成功，其他表示错误。
该函数用以初始化整个SDK，初始化相关配置参数如下：

#### 3.1.1 SDK初始化参数配置


```c++
typedef struct {
    const char* appID;        // 应用id
    const char* apiKey;       // 应用key
    const char* apiSecret;    // 应用secret
} AIKIT_InitParam;
```

**说明**：这个接口用于设置SDK初始化相关的参数，相关参数配置参考下表：

|   字段    | 含义                                     | 类型   | 说明           | 是否必传 |
| :-------: | :--------------------------------------- | :----- | -------------- | -------- |
|   appID   | 应用的app_id，需要在飞云交互平台申请     | string | "maxLength":16 | 是       |
|  apiKey   | 应用的app_key，需要在飞云交互平台申请    | string |                | 是       |
| apiSecret | 应用的app_secret，需要在飞云交互平台申请 | string |                | 是       |

#### 3.1.2 回调函数注册

```c++
AIKITAPI int32_t AIKIT_ChatCallback(const AIKIT_ChatCBS& cbs);
```

**说明**：注册回调函数用于接收处理异步结果。其中，AIKIT_ChatCBS是一个包含回调函数的结构体，定义如下：

```c++
typedef struct {
    onChatOutput outputCB;     // 输出回调
    onChatToken  tokenCB;      // token计算信息回调
    onChatError  errorCB;      // 错误回调
} AIKIT_ChatCBS;
```

其中三个回调函数定义如下：

#### 1. onChatOutput，用于异步请求回调结果输出

```c++
typedef void (*onChatOutput)(AIChat_Handle* handle, const char* role, const char* content, const int& index);
```

参数：

| 字段    | 含义                                           | 类型           |
| ------- | ---------------------------------------------- | -------------- |
| handle  | 用户句柄                                       | AIChat_Handle* |
| role    | user：表示用户的提问， assistant：表示AI的回复 | const char*    |
| content | 文本内容，该角色的对话内容                     | const char*    |
| index   | 结果序号，在多候选中使用                       | int            |

#### 2. onChatToken，用于统计Token相关的数据

```c++
typedef void (*onChatToken)(AIChat_Handle* handle, const int& completionTokens, const int& promptTokens, const int& totalTokens);
```

参数：

| 字段             | 含义                   | 类型           |
| ---------------- | ---------------------- | -------------- |
| handle           | 用户句柄               | AIChat_Handle* |
| completionTokens | 历史总回答tokens大小   | int            |
| promptTokens     | 历史问题总tokens大小   | int            |
| totalTokens      | 问题和回答的tokens大小 | int            |

#### 3. onChatError，用于返回错误信息

```c++
typedef void (*onChatError)(AIChat_Handle* handle, const int& err, const char* errDesc);
```

详细错误码描述见文档最后错误码列表。

参数：

| 字段    | 含义     | 类型           |
| ------- | -------- | -------------- |
| handle  | 用户句柄 | AIChat_Handle* |
| err     | 错误码   | int            |
| errDesc | 错误信息 | const char* |

#### 4. 用户句柄handle说明

**说明**：handle是一个结构体，定义如下：

```c++
typedef struct AICHAT_HANDLE {
    void*       usrContext;  // 用户上下文指针
} AIChat_Handle;
```

参数如下：

| 字段       | 含义           | 类型  |
| ---------- | -------------- | ----- |
| usrContext | 用户上下文指针 | void* |

### 3.2 交互请求接口<异步>

```c++
AIKITAPI int32_t AIKIT_AsyncChat(const ChatParam* params, const char* inputText, void* usrContext);
```

**说明**：这个接口用于用户异步发送请求，ret为结果错误码，0=成功，错误码见最后错误码列表。相关参数配置参考下表：

| 字段            | 含义       | 是否必填 | 备注   |
| ------------- | -------- | ---- | ---- |
| params        | 请求参数配置信息 | 是    | 见 3.2.1请求参数配置 |
| inputText     | 请求文本内容   | 是    | 具体支持文本格式，见3.2.2请求文本内容格式 |
| usrContext    | 用户上下文指针 | 否    | 此指针会在回调函数中带出，用于应用进行对应上下文操作 |

#### 3.2.1 请求参数配置

```c++
ChatParam* ChatParam::builder();
```

**说明**：这个接口用于配置请求参数，相关参数配置参考下表：

| 字段        | 含义                                   | 类型   | 限制                                                         | 是否必传 |
| :---------- | -------------------------------------- | ------ | ------------------------------------------------------------ | -------- |
| domain      | 需要使用的领域，当前仅支持通用大模型。 | char*  | domian值与星火服务版本对应。<br/>默认请求星火v1.x 版本，domian默认值为 "general"<br/>星火v2.0 版本(url: ws://spark-api.xf-yun.com/v2.1/chat)，domian 值为"generalv2" | 是       |
| uid         | 授权的用户id，用于关联用户交互的上下文 | char*  | "maxLength":32                                               | 否       |
| maxToken    | 回答的tokens的最大长度                 | int    | 取值范围1-2048，默认2048                                     | 否       |
| temperature | 配置核采样阈值，改变结果的随机程度     | float  | 最小是0, 最大是1，默认0.5;<br />数字越大，随机性越小，改变此值会影响效果。 | 否       |
| auditing    | 内容审核的场景策略                     | char*  | strict表示严格审核策略；moderate表示中等审核策略；loose表示宽松审核策略；default表示默认的审核程度；（调整策略需线下联系产品经理授权） | 否       |
| topK        | 配置从k个候选中随机选择⼀个（⾮等概率) | int    | 取值范围1-6，默认值4                                         | 否       |
| url         | 星火服务器地址                         | string | 默认为 v1.x版本。<br>星火V2.0 版本服务地址： ws://spark-api.xf-yun.com/v2.1/chat 。domain值对应为 "generalv2" | 否       |

请求参数配置示例：

```c++
ChatParam* params = ChatParam::builder();
params->uid("xxxid")
      ->domain("generalv2")
      ->maxToken(1024)
      ->url("ws://spark-api.xf-yun.com/v2.1/chat");
```

#### 3.2.2 请求文本内容格式

**说明**：不同对话交互场景，输入文本格式不同，具体不同如下：

#### 1. 单轮交互

输入文本是纯字符串，代码示例：

```c++
ChatParam* params = ChatParam::builder();
params->uid("xxxid")
      ->domain("generalv2")
      ->maxToken(2048)
      ->url("ws://spark-api.xf-yun.com/v2.1/chat");
// 设置输入文本
string inputText="你好";
// 发起交互请求
ret = AIKIT_AsyncChat(params, inputText.c_str() , nullptr);
```

#### 2. 多轮持续交互，包含交互上下文

##### 1、用户自行管理交互上下文，输入文本是json数组格式包含完整交互历史，具体SDK调用流程如下：

1.1、配置请求相关参数

```c++
ChatParam* params = ChatParam::builder();
params->uid("xxxid")
      ->domain("generalv2")
      ->maxToken(2048)
      ->url("ws://spark-api.xf-yun.com/v2.1/chat");
```

1.2、第一轮交互：

```c++
// 设置输入文本
string inputText="你好";
// 发起交互请求
ret = AIKIT_AsyncChat(params, inputText.c_str() , nullptr);
```

或

```c++
// 设置输入文本
string inputText = R"(
[
 {"role": "user", "content": "你好"},        // ⽤户第⼀个问题 role是user,表示是⽤户的提问
]
)";
// 发起交互请求
ret = AIKIT_AsyncChat(params, inputText.c_str() , nullptr);
```

1.3、第二轮交互

```c++
// 每次输入的请求文本和对应请求的响应文本都需要按照交互顺序完整的记录下来，最终形成的文本数据格式如下：
string inputText = R"(
[
 {"role": "user", "content": "你好"},        // ⽤户第⼀个问题 role是user,表示是⽤户的提问
 {"role": "assistant", "content": "你好！"}, // AI的第⼀个回复 role是assistant，表示是AI的回复
 {"role": "user", "content": "你是谁？"},    // ⽤户第⼆个问题
]
)";
ret = AIKIT_AsyncChat(params, inputText.c_str() , nullptr);
```


### 3.3 逆初始化接口

**说明**：应用退出时可调用此函数回收SDK占用相关资源。

```c++
AIKITAPI int32_t AIKIT_UnInit() ;
```

## 内容审核说明

当返回10013或者10014错误码时，代码内容审核判断当前问题或回复的信息涉及敏感信息。返回错误的同时，message中携带当前的敏感提示语，如果：  

* 用户的请求文本敏感，则给出对应提示  

* 生成的内容敏感，需要删除已经显示的内容信息，替换成对应提示  

如果需要调整内容审核的严格程度、敏感词等信息，请联系我们。

## 6、错误码列表

服务错误码：

| 错误码 | 错误信息                                                     |
| ------ | ------------------------------------------------------------ |
| 0      | 成功                                                         |
| 10000  | 升级为ws出现错误                                             |
| 10001  | 通过ws读取用户的消息 出错                                    |
| 10002  | 通过ws向用户发送消息 出错                                    |
| 10003  | 用户的消息格式有错误                                         |
| 10004  | 用户数据的schema错误                                         |
| 10005  | 用户参数值有错误                                             |
| 10006  | 用户并发错误：当前用户已连接，同一用户不能多处同时连接。     |
| 10007  | 用户流量受限：服务正在处理用户当前的问题，需等待处理完成后再发送新的请求。<br />（必须要等大模型完全回复之后，才能发送下一个问题） |
| 10008  | 服务容量不足，联系服务商                                     |
| 10009  | 和引擎建立连接失败                                           |
| 10010  | 接收引擎数据的错误                                           |
| 10011  | 向引擎发送数据的错误                                         |
| 10012  | 引擎内部错误                                                 |
| 10013  | 用户问题涉及敏感信息，审核不通过，拒绝处理此次请求。         |
| 10014  | 回复结果涉及到敏感信息，审核不通过，后续结果无法展示给用户。 |
| 10015  | appid在黑名单中                                              |
| 10016  | appid授权类的错误。比如：未开通此功能，未开通对应版本，token不足，并发超过授权 等等。 （联系我们开通授权或提高限制） |
| 10017  | 清除历史失败                                                 |
| 10018  | 用户在5分钟内持续发送ping消息，但并没有实际请求数据，会返回该错误码并断开ws连接。短链接使用无需关注 |
| 10019  | 该错误码表示返回结果疑似敏感，建议拒绝用户继续交互           |
| 10110  | 服务忙，请稍后再试。                                         |
| 10163  | 请求引擎的参数异常 引擎的schema 检查不通过                   |
| 10222  | 引擎网络异常                                                 |
| 10223  | LB找不到引擎节点                                             |
| 10907  | token数量超过上限。对话历史+问题的字数太多，需要精简输入。   |
| 11200  | 授权错误：该appId没有相关功能的授权 或者 业务量超过限制（联系我们开通授权或提高限制） |
| 11201  | 授权错误：日流控超限。超过当日最大访问量的限制。（联系我们提高限制） |
| 11202  | 授权错误：秒级流控超限。秒级并发超过授权路数限制。（联系我们提高限制） |
| 11203  | 授权错误：并发流控超限。并发路数超过授权路数限制。（联系我们提高限制） |

SDK错误码：

| 错误码 | 错误信息                        |
| ------ | ------------------------------- |
| 18300  | sdk不可用                       |
| 18301  | sdk没有初始化                   |
| 18302  | sdk初始化失败                   |
| 18303  | sdk已经初始化                   |
| 18304  | sdk不合法参数                   |
| 18305  | sdk会话handle为空               |
| 18306  | sdk会话未找到                   |
| 18307  | sdk会话重复终止                 |
| 18308  | 超时错误                        |
| 18309  | sdk正在初始化中                 |
| 18310  | sdk会话重复开启                 |
| 18400  | 工作目录无写权限                |
| 18402  | 文件打开失败                    |
| 18403  | 内存分配失败                    |
| 18500  | 未找到该参数key                 |
| 18501  | 参数范围溢出，不满足约束条件    |
| 18502  | sdk初始化参数为空               |
| 18503  | sdk初始化参数中appid为空        |
| 18504  | sdk初始化参数中apiKey为空       |
| 18505  | sdk初始化参数中apiSecret为空    |
| 18507  | input参数为空                   |
| 18509  | 必填参数缺失                    |
| 18700  | 通用网络错误                    |
| 18701  | 网路不通                        |
| 18702  | 网关检查不过                    |
| 18712  | http 404错误                    |
| 18801  | 连接建立出错                    |
| 18802  | 结果等待超时                    |
| 18311  | sdk同一能力并发路数超出最大限制 |

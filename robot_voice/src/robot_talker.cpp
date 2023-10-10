#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <atomic>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <robot_voice/StringToVoice.h>


#include "ifly_spark/aikit_spark_api.h"

class IflySpark {
public:
  IflySpark() {
    ROS_INFO("IflySpark Constructor");
  }
  ~IflySpark() {
    ROS_INFO("IflySpark Destructor");
  }

  int SendRequest(std::string& ask_str) {
    // clear answer_str_
    IflySpark::answer_str_ = "";
    IflySpark::is_over_ = false;
    //请求参数配置
    AIKIT::ChatParam* config = AIKIT::ChatParam::builder();
    config->uid("xxxid")
          ->domain("generalv2")
          ->auditing("default")
          ->url("ws://spark-api.xf-yun.com/v2.1/chat");

    int ret = AIKIT::AIKIT_AsyncChat(config, ask_str.c_str(), &usr_);
    if(ret != 0) {
        ROS_ERROR("AIKIT_AsyncChat failed:%d\n",ret);
        return ret;
    }
    return ret;
  }

  static void ChatOnToken(AIKIT::AIChat_Handle* handle, const int& completionTokens, const int& promptTokens, const int& totalTokens) {
    if(handle!=nullptr){
        std::cout<<"chatID:"<<((UsrCtx*)handle->usrContext)->chatID<<", ";
    }
    std::cout<<"completionTokens:"<<completionTokens<<" promptTokens:"<<promptTokens<<" totalTokens:"<<totalTokens<<std::endl;
    IflySpark::is_over_ = true;
  }

  static void ChatOnOutput(AIKIT::AIChat_Handle* handle, const char* role, const char* content, const int& index) {
    if(handle!=nullptr){
        std::cout<<"chatID:"<<((UsrCtx*)handle->usrContext)->chatID<<", ";
    }
    
    std::string slice_txt = content;
    IflySpark::answer_str_ += slice_txt;
  }

  static void ChatOnError(AIKIT::AIChat_Handle* handle, const int& err, const char* errDesc) {
    if(handle!=nullptr){
        std::cout<<"chatID:"<<((UsrCtx*)handle->usrContext)->chatID<<", ";
    }
    printf("chatOnError: err:%d,desc:%s\n",err,errDesc);
  }

  void InitSDK() {
    AIKIT_InitParam initParam{};
    initParam.appID = "bb839ccf";
    initParam.apiKey = "ab95d52b4ea9f2a5dc15e77ae3f778fa";
    initParam.apiSecret = "MTAxZDllZDY1OTVjYmIwYzg2NDBlZWQ0";
    AIKIT::AIKIT_SetLogInfo(100,0,nullptr);
    int ret = AIKIT::AIKIT_Init(&initParam);
    if(ret != 0) {
        ROS_ERROR("AIKIT_Init failed:%d",ret);
        return ;
    }
    
    //异步回调注册
    AIKIT::AIKIT_ChatCallback({ ChatOnOutput, ChatOnToken, ChatOnError });
  }

  void UnInit() {
    //逆初始化SDK
    AIKIT::AIKIT_UnInit();
  }

  static std::string get_answer_str() {
    while (IflySpark::is_over_ == false || IflySpark::answer_str_ == "") {
      sleep(1); 
    }

    return answer_str_;
  }

private:
  typedef struct UsrCtx {
    std::string chatID;
  } UsrCtx_t;

  // 设置chatID,用于用户动态控制会话轮次
  UsrCtx_t usr_ = {"FistRound"};

  static std::string answer_str_;
  static bool is_over_;
};

std::string IflySpark::answer_str_ = "";
bool IflySpark::is_over_ = false;

class RobotTalker {
public:
  RobotTalker() {
    ROS_INFO("RobotTalker Constructor");
  }

  ~RobotTalker() {
    robot_talker_.UnInit();
    ROS_INFO("RobotTalker Destructor");
  }

  int Init(ros::NodeHandle& nh) {
    robot_talker_.InitSDK();
    client_ = nh.serviceClient<robot_voice::StringToVoice>("str2voice");
    return 0;
  }

  void ToDownstream(const std::string& answer_txt) {
    robot_voice::StringToVoice::Request req;
    robot_voice::StringToVoice::Response resp;
    req.data = answer_txt;

    bool ok = client_.call(req, resp);
    if (ok) {
      printf("send str2voice service success: %s\n", req.data.c_str());
    } else {
      printf("failed to send str2voice service\n");
    }
  }

  bool ChatterCallbback(robot_voice::StringToVoice::Request &req, robot_voice::StringToVoice::Response &resp) {
    printf("i received: %s\n", req.data.c_str());

    std::string voice_txt = req.data;
    std::string answer_txt = "";

    robot_talker_.SendRequest(voice_txt);
    answer_txt = IflySpark::get_answer_str();
    if (answer_txt != "") {
      ToDownstream(answer_txt);
      printf("answer_txt = %s\n", answer_txt.c_str());
    }

    resp.success = true;
    return resp.success;
  }

  void Start(ros::NodeHandle& nh) {
    chatter_server_ = nh.advertiseService("human_chatter", &RobotTalker::ChatterCallbback, this);
  }

private:
  ros::ServiceServer chatter_server_;
  ros::ServiceClient client_;
  IflySpark robot_talker_;
};


int main(int argc, char* argv[]) {
  int ret = 0;
  ros::init(argc, argv, "robot_talker");
  ros::NodeHandle nh;

  RobotTalker rc;
  rc.Init(nh);

  ROS_INFO("this is a robot talker base ifly spark\n");
  rc.Start(nh);

  ros::spin();

  return 0;
}

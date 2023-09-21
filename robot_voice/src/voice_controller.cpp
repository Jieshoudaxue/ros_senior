#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "ifly/qisr.h"
#include "ifly/msp_cmn.h"
#include "ifly/formats.h"
#include "ifly/msp_errors.h"
#include "ifly/speech_recognizer.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class VoiceDetector {
public:
  VoiceDetector() {
    ROS_INFO("voice detector Constructor");
  }
  ~VoiceDetector() {
    ROS_INFO("voice detector Destructor ");
  }

  int Init() {
    int ret = MSP_SUCCESS;
    ret = MSPLogin(NULL, NULL, login_params_.c_str());
    if (MSP_SUCCESS != ret)	{
      ROS_ERROR("MSPLogin failed , Error code %d", ret);
      MSPLogout(); // Logout...
      return -1;
    }    

    ROS_INFO("MSP Login for update, waiting for seconds...");

    return 0;
  }

  static void JoinTxt(const char *result, char is_last) {
    if (result) {
      std::string slice_txt = result;

      VoiceDetector::voice_txt_ += slice_txt;
    }
    if (is_last) {
      printf("voice txt : %s\n", VoiceDetector::voice_txt_.c_str());
    }
  }

  static void InitSpeech() {
    VoiceDetector::voice_txt_ = "";

    printf("Start Listening...\n");
  }

  static void EndSpeech(int reason) {
    if (reason == END_REASON_VAD_DETECT) {
      printf("\nSpeaking done \n");
    } else {
      printf("\nRecognizer error %d\n", reason);
    }
  }

  int StartSpeech() {
    int ret;
    int i = 0;

    struct speech_rec iat;

    struct speech_rec_notifier recnotifier = {
      JoinTxt,
      InitSpeech,
      EndSpeech
    };

    ret = sr_init(&iat, session_begin_params_.c_str(), SR_MIC, &recnotifier);
    if (ret) {
      ROS_ERROR("speech recognizer init failed");
      return -1;
    }

    ret = sr_start_listening(&iat);
    if (ret) {
      printf("start listen failed %d\n", ret);
    }

    /* demo 15 seconds recording */
    sleep(15);

    ret = sr_stop_listening(&iat);
    if (ret) {
      printf("stop listening failed %d\n", ret);
    }

    sr_uninit(&iat);

    return 0;
  }

  static std::string get_voice_txt_() {
    return voice_txt_;
  }

private:
	const std::string login_params_ = "appid = bb839ccf, work_dir = .";
	const std::string session_begin_params_ =
		"sub = iat, domain = iat, language = zh_cn, "
		"accent = mandarin, sample_rate = 16000, "
		"result_type = plain, result_encoding = utf8";

  const uint32_t	BufferSize = 4096;
  uint64_t g_buffersize = BufferSize;
  static std::string voice_txt_;
};

class VoiceResponse {
public:
  VoiceResponse() {}
  VoiceResponse(ros::NodeHandle& nh) {
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ROS_INFO("voice Response Constructor");
  }

  ~VoiceResponse() {
    ROS_INFO("voice Response Destructor ");
  }

private:
  ros::Publisher cmd_pub_;

};

std::string VoiceDetector::voice_txt_ = "";

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "voice_controller");

  std::string tip_string = "向前, 向后, 向左, 向右, 转圈, 结束";
  printf("this is a voice controller app for robot, you can say: %s\n", tip_string.c_str());

  int ret = 0;
  VoiceDetector vd;
  ret = vd.Init();
  if (ret < 0) {
    return -1;
  }

  ret = vd.StartSpeech();
  if (ret < 0) {
    return -1;
  }

  std::string result = VoiceDetector::get_voice_txt_();
  printf("result: %s\n", result.c_str());




  return 0;
}
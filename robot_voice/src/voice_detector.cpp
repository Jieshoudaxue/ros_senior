#include <ros/ros.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "ifly_voice/qisr.h"
#include "ifly_voice/qtts.h"
#include "ifly_voice/msp_cmn.h"
#include "ifly_voice/formats.h"
#include "ifly_voice/msp_errors.h"
#include "ifly_voice/speech_recognizer.h"
#include <std_msgs/String.h>
#include <robot_voice/StringToVoice.h>

class Helper {
public:
  static void SignalHandler(int signal) {
      ROS_INFO("\nCaught signal %d. Exiting gracefully...\n", signal);
      exit(0);
  }
};

class VoiceDetector {
public:
  VoiceDetector() {
    ROS_INFO("VoiceDetector Constructor");
  }
  ~VoiceDetector() {
    ROS_INFO("VoiceDetector Destructor");
  }

  int Init() {
    int ret = MSP_SUCCESS;
    ret = MSPLogin(NULL, NULL, login_params_.c_str());
    if (MSP_SUCCESS != ret)	{
      ROS_ERROR("MSPLogin failed , Error code %d", ret);
      MSPLogout(); // Logout...
      return -1;
    }    

    ROS_INFO("VoiceDetector MSP Login for update, waiting for seconds...");

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

    printf("clear cache, start Listening...\n");
  }

  static void EndSpeech(int reason) {
    if (reason == END_REASON_VAD_DETECT) {
      printf("\nSpeaking done \n");
    } else {
      printf("\nRecognizer error %d\n", reason);
    }
  }

  int SpeechOnce() {
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
    sleep(10);

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

std::string VoiceDetector::voice_txt_ = "";


int main(int argc, char* argv[]) {
  int ret = 0;
  ros::init(argc, argv, "voice_detector");
  ros::NodeHandle nh;
  ros::ServiceClient client_ = nh.serviceClient<robot_voice::StringToVoice>("human_chatter");

  if (signal(SIGINT, Helper::SignalHandler) == SIG_ERR) {
    return -1;
  }

  VoiceDetector vd;
  ret = vd.Init();
  if (ret < 0) {
    return -1;
  }
  
  while (1) {
    ret = vd.SpeechOnce();
    if (ret < 0) {
      return -1;
    }

    std::string voice_txt = VoiceDetector::get_voice_txt_();
    if (voice_txt == "") {
      printf("voice_txt is empty, do not send chatter\n");
      continue;
    } else if (voice_txt.find("结束") != std::string::npos) {
      break;
    }

    robot_voice::StringToVoice::Request req;
    robot_voice::StringToVoice::Response resp;
    req.data = voice_txt;

    bool ok = client_.call(req, resp);
    if (ok) {
      printf("send human_chatter service success: %s\n", req.data.c_str());
    } else {
      printf("failed to send human_chatter service\n");
    }
  }

  ros::spin();

  return 0;
}

    
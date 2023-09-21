#include <ros/ros.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "ifly/qisr.h"
#include "ifly/qtts.h"
#include "ifly/msp_cmn.h"
#include "ifly/formats.h"
#include "ifly/msp_errors.h"
#include "ifly/speech_recognizer.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


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

std::string VoiceDetector::voice_txt_ = "";

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

  int ProcessTxt(std::string& txt) {
    int          ret          = -1;
    FILE*        fp           = NULL;
    const char*  sessionID    = NULL;
    unsigned int audio_len    = 0;
    int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;
    WavePcmHdr_t wav_hdr      = {
      { 'R', 'I', 'F', 'F' },
      0,
      {'W', 'A', 'V', 'E'},
      {'f', 'm', 't', ' '},
      16,
      1,
      1,
      16000,
      32000,
      2,
      16,
      {'d', 'a', 't', 'a'},
      0  
    };

    const char* src_text = txt.c_str();
    const char* des_path = filename_.c_str();
    const char* params = session_begin_params_.c_str();

    if (NULL == src_text || NULL == des_path) {
      ROS_ERROR("params is error!");
      return ret;
    }

    fp = fopen(des_path, "wb");
    if (NULL == fp) {
      ROS_ERROR("open %s error", des_path);
      return ret;
    }

    /* 开始合成 */
    sessionID = QTTSSessionBegin(params, &ret);
    if (MSP_SUCCESS != ret) {
      ROS_ERROR("QTTSSessionBegin failed, error code: %d", ret);
      fclose(fp);
      return ret;
    }

    ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
    if (MSP_SUCCESS != ret) {
      ROS_ERROR("QTTSTextPut failed, error code: %d",ret);
      QTTSSessionEnd(sessionID, "TextPutError");
      fclose(fp);
      return ret;
    }
    
    printf("正在合成 ...\n");
    fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
    while (1)  {
      /* 获取合成音频 */
      const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
		  if (MSP_SUCCESS != ret) {
        break;
      }

      if (NULL != data) {
        fwrite(data, audio_len, 1, fp);
        wav_hdr.data_size += audio_len; //计算data_size大小
      }

      if (MSP_TTS_FLAG_DATA_END == synth_status) {
        break;
      }

      printf(">");
      usleep(150*1000); //防止频繁占用CPU
    }
    printf("\n");

    if (MSP_SUCCESS != ret) {
      ROS_ERROR("QTTSAudioGet failed, error code: %d",ret);
      QTTSSessionEnd(sessionID, "AudioGetError");
      fclose(fp);
      return ret;
    }

    /* 修正wav文件头数据的大小 */
    wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
    
    /* 将修正过的数据写回文件头部,音频文件为wav格式 */
    fseek(fp, 4, 0);
    fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
    fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
    fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
    fclose(fp);
    fp = NULL;
    /* 合成完毕 */
    ret = QTTSSessionEnd(sessionID, "Normal");
    if (MSP_SUCCESS != ret) {
      ROS_ERROR("QTTSSessionEnd failed, error code: %d", ret);
      return ret;
    }

    fp = popen(play_cmd_.c_str(),"r");
    if (fp == NULL) {
      ROS_ERROR("play /tmp/tts_sample.wav failed");
      return -1;
    }
    pclose(fp);

    return 0;
  }

  int AnswerVoice(std::string& voice_txt) {
    int ret = -1;
    std::string answer_txt = "";

    if (voice_txt.find("前") != std::string::npos) {
      answer_txt = "小车请向前跑";

      ret = ProcessTxt(answer_txt);
      if (MSP_SUCCESS != ret) {
        ROS_ERROR("AnswerVoice failed, error code: %d", ret);
        return -1;
      }

      SendTopic(0.3, 0);
    } else if (voice_txt.find("后") != std::string::npos) {
      answer_txt = "小车请向后倒";

      ret = ProcessTxt(answer_txt);
      if (MSP_SUCCESS != ret) {
        ROS_ERROR("AnswerVoice failed, error code: %d", ret);
        return -1;
      }

      SendTopic(-0.3, 0);
    } else if (voice_txt.find("左") != std::string::npos) {
      answer_txt = "小车请向左转";

      ret = ProcessTxt(answer_txt);
      if (MSP_SUCCESS != ret) {
        ROS_ERROR("AnswerVoice failed, error code: %d", ret);
        return -1;
      }

      SendTopic(0, 0.3);
    } else if (voice_txt.find("右") != std::string::npos) {
      answer_txt = "小车请向右转";

      ret = ProcessTxt(answer_txt);
      if (MSP_SUCCESS != ret) {
        ROS_ERROR("AnswerVoice failed, error code: %d", ret);
        return -1;
      }

      SendTopic(0, -0.3);
    } else if (voice_txt.find("转") != std::string::npos) {
      answer_txt = "小车请打转";

      ret = ProcessTxt(answer_txt);
      if (MSP_SUCCESS != ret) {
        ROS_ERROR("AnswerVoice failed, error code: %d", ret);
        return -1;
      }

      SendTopic(0.3, 0.3);
    }


    return 0;
  }

  void SendTopic(float linear_x, float angular_z) {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    cmd_pub_.publish(msg);
  }

private:
  ros::Publisher cmd_pub_;

	const std::string session_begin_params_ = 
    "voice_name = xiaoyan, text_encoding = utf8, "
    "sample_rate = 16000, speed = 50, volume = 50, "
    "pitch = 50, rdn = 2";
	const std::string filename_ = "/tmp/tts_sample.wav"; //合成的语音文件名称
  const std::string play_cmd_ = "play /tmp/tts_sample.wav";

  /* wav音频头部格式 */
  typedef struct WavePcmHdr {
    char            riff[4];                // = "RIFF"
    int		size_8;                 // = FileSize - 8
    char            wave[4];                // = "WAVE"
    char            fmt[4];                 // = "fmt "
    int		fmt_size;		// = 下一个结构体的大小 : 16

    short int       format_tag;             // = PCM : 1
    short int       channels;               // = 通道数 : 1
    int		samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
    int		avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
    short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
    short int       bits_per_sample;        // = 量化比特数: 8 | 16

    char            data[4];                // = "data";
    int		data_size;              // = 纯数据长度 : FileSize - 44 
  } WavePcmHdr_t;

};


int main(int argc, char* argv[]) {
  int ret = 0;
  ros::init(argc, argv, "voice_controller");
  ros::NodeHandle nh;

  if (signal(SIGINT, Helper::SignalHandler) == SIG_ERR) {
    return -1;
  }

  printf("this is a voice controller app for robot, you can say: 向前, 向后, 向左, 向右, 转圈, 结束\n");
  VoiceResponse vr(nh);
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
      continue;
    } else if (voice_txt.find("结束") != std::string::npos) {
      break;
    }

    vr.AnswerVoice(voice_txt);
  }

  ros::spin();

  return 0;
}
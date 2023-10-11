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
#include <robot_voice/StringToVoice.h>


class Helper {
public:
  static void SignalHandler(int signal) {
      ROS_INFO("\nCaught signal %d. Exiting gracefully...\n", signal);
      exit(0);
  }
};


class VoiceCreator {
public:
  VoiceCreator() {
    ROS_INFO("VoiceCreator Constructor");
  }

  ~VoiceCreator() {
    ROS_INFO("VoiceCreator Destructor ");
  }

  int Init() {
    int ret = MSP_SUCCESS;
    ret = MSPLogin(NULL, NULL, login_params_.c_str());
    if (MSP_SUCCESS != ret)	{
      ROS_ERROR("MSPLogin failed , Error code %d", ret);
      MSPLogout(); // Logout...
      return -1;
    }    

    ROS_INFO("VoiceCreator MSP Login for update, waiting for seconds...");

    return 0;
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
    sleep(1);
    pclose(fp);

    return 0;
  }

  bool Speeking(robot_voice::StringToVoice::Request &req, robot_voice::StringToVoice::Response &resp) {
    int ret = -1;
    ret = ProcessTxt(req.data);
    if (MSP_SUCCESS != ret) {
      ROS_ERROR("AnswerVoice failed, error code: %d", ret);
      resp.success = false;
      return false;
    } else {
      resp.success = true;
    }

    return resp.success;
  }

  void Start(ros::NodeHandle& nh) {
    server_ = nh.advertiseService("str2voice", &VoiceCreator::Speeking, this);
  }

private:
  ros::ServiceServer server_;

	const std::string login_params_ = "appid = bb839ccf, work_dir = .";

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

int main(int argc, char ** argv) {
  int ret = 0;
  ros::init(argc, argv, "voice_creator");
  ros::NodeHandle nh;

  if (signal(SIGINT, Helper::SignalHandler) == SIG_ERR) {
    return -1;
  }

  VoiceCreator vc;
  ret = vc.Init();
  if (ret < 0) {
    return -1;
  }

  vc.Start(nh);

  ros::spin();
  return 0;
}

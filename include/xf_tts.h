#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <sound_play/SoundRequestAction.h>
#include <sound_play/SoundRequestGoal.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <xunfei_sdk/qtts.h>
#include <xunfei_sdk/msp_cmn.h>
#include <xunfei_sdk/msp_errors.h>

namespace xf_tts{
    /* wav音频头部格式 */
    typedef struct _wave_pcm_hdr
    {
        char            riff[4];                // = "RIFF"
        int             size_8;                 // = FileSize - 8
        char            wave[4];                // = "WAVE"
        char            fmt[4];                 // = "fmt "
        int             fmt_size;        // = 下一个结构体的大小 : 16

        short int       format_tag;             // = PCM : 1
        short int       channels;               // = 通道数 : 1
        int             samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
        int             avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
        short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
        short int       bits_per_sample;        // = 量化比特数: 8 | 16

        char            data[4];                // = "data";
        int             data_size;              // = 纯数据长度 : FileSize - 44
    } wave_pcm_hdr;

    /* 默认wav音频头部数据 */
    wave_pcm_hdr default_wav_hdr =
    {
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

    typedef actionlib::SimpleActionClient<sound_play::SoundRequestAction> SoundPlayClient;

    class XF_TTS{
    private:
        const char* FileName = "/home/oem/catkin_ws/src/Olders_helper/audio/voice.wav";
        const char* PlayPath = "play /home/oem/catkin_ws/src/Olders_helper/audio/voice.wav";

        ros::NodeHandle nh_;
        ros::Subscriber sub = nh_.subscribe<std_msgs::String>("/voice/xf_tts_topic",3, bind(&XF_TTS::TTSCallback, this, _1));
        SoundPlayClient* sound_play_client_ptr = new SoundPlayClient("/sound_play", true);

    public:
        int text_to_speech(const char* src_text, const char* des_path, const char* params);
        int makeTextToWav(const char* text, const char* filename);
        void PlayWav();
        void TTSCallback(const std_msgs::String::ConstPtr &msg);
    };
}
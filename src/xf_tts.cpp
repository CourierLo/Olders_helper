/*
* 语音合成（Text To Speech，TTS）技术能够自动将任意文字实时转换为连续的
* 自然语音，是一种能够在任何时间、任何地点，向任何人提供语音信息服务的
* 高效便捷手段，非常符合信息时代海量数据、动态更新和个性化查询的需求。
*/
#include <xf_tts.h>
/* 文本合成 */
namespace xf_tts{
    XF_TTS::XF_TTS(ros::NodeHandle &nh) : nh_(nh) {
        // sound_play_client_ptr = new SoundPlayClient("/sound_play", true);
    }

    int XF_TTS::text_to_speech(const char* src_text, const char* des_path, const char* params)
    {
        int          ret          = -1;
        FILE*        fp           = NULL;
        const char*  sessionID    = NULL;
        unsigned int audio_len    = 0;
        wave_pcm_hdr wav_hdr      = default_wav_hdr;
        int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

        if (NULL == src_text || NULL == des_path)
        {
            printf("params is error!\n");
            return ret;
        }
        fp = fopen(des_path, "wb");
        if (NULL == fp)
        {
            printf("open %s error.\n", des_path);
            return ret;
        }
        /* 开始合成 */
        sessionID = QTTSSessionBegin(params, &ret);
        if (MSP_SUCCESS != ret)
        {
            printf("QTTSSessionBegin failed, error code: %d.\n", ret);
            fclose(fp);
            return ret;
        }
        ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
        if (MSP_SUCCESS != ret)
        {
            printf("QTTSTextPut failed, error code: %d.\n",ret);
            QTTSSessionEnd(sessionID, "TextPutError");
            fclose(fp);
            return ret;
        }
        printf("正在合成 ...\n");
        fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
        while (1)
        {
            /* 获取合成音频 */
            const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
            if (MSP_SUCCESS != ret)
                break;
            if (NULL != data)
            {
                fwrite(data, audio_len, 1, fp);
                wav_hdr.data_size += audio_len; //计算data_size大小
            }
            if (MSP_TTS_FLAG_DATA_END == synth_status)
                break;
            printf(">");
            usleep(15*1000); //防止频繁占用CPU
        }//合成状态synth_status取值请参阅《讯飞语音云API文档》
        printf("\n");
        if (MSP_SUCCESS != ret)
        {
            printf("QTTSAudioGet failed, error code: %d.\n",ret);
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
        if (MSP_SUCCESS != ret)
        {
            printf("QTTSSessionEnd failed, error code: %d.\n",ret);
        }

        return ret;
    }

    /**
    *   将文本发送到讯飞服务器，获取语音文件文件
    **/
    int XF_TTS::makeTextToWav(const char* text, const char* filename)
    {
        int         ret                  = MSP_SUCCESS;
        const char* login_params         = "appid = 5e841d96, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动
        /*
        * rdn:           合成音频数字发音方式
        * volume:        合成音频的音量
        * pitch:         合成音频的音调
        * speed:         合成音频对应的语速
        * voice_name:    合成发音人
        * sample_rate:   合成音频采样率
        * text_encoding: 合成文本编码格式
        *
        * 详细参数说明请参阅《讯飞语音云MSC--API文档》
        */
        const char* session_begin_params = "voice_name = xiaoyan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 0";

        /* 用户登录 */
        ret = MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://open.voicecloud.cn注册获取
        if (MSP_SUCCESS != ret)
        {
            printf("MSPLogin failed, error code: %d.\n", ret);
            goto exit ;//登录失败，退出登录
        }
        /* 文本合成 */
        printf("开始合成 ...\n");
        ret = text_to_speech(text, filename, session_begin_params);
        if (MSP_SUCCESS != ret)
        {
            printf("text_to_speech failed, error code: %d.\n", ret);
        }
        printf("合成完毕\n");

    exit:

        MSPLogout(); //退出登录
        return 0;
    }

    /**
    * play the wav file
    **/
    void XF_TTS::PlayWav()
    {
        system(PlayPath);
    }
    /**
     *接受/voice/xf_tts_topic话题的字符串的回调函数
    */
    bool XF_TTS::TextToSpeech(olders_helper::tts_text::Request& req, olders_helper::tts_text::Response& res) {
        std::cout<<"get topic text: " << req.text.c_str();
        makeTextToWav(req.text.c_str(), FileName);
        //PlayWav();

        // if(sound_play_client_ptr->getState() == actionlib::SimpleClientGoalState::ACTIVE)   
        //     sound_play_client_ptr->cancelGoal();

        // // 必须注明音量
        // sound_play::SoundRequestGoal goal;
        // goal.sound_request.arg = "/home/oem/catkin_ws/src/Olders_helper/audio/voice.wav";
        // goal.sound_request.sound = goal.sound_request.PLAY_FILE;
        // goal.sound_request.command = goal.sound_request.PLAY_ONCE;
        // goal.sound_request.volume = 20.0;

        // //std::cout << goal.sound_request.arg << "\n";

        // sound_play_client_ptr->sendGoal(goal);
        res.tts_ok = true;

        return true;
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv,"xf_tts_node");
    ros::NodeHandle nh_;
    const char* start = "你好";
    //const char* FileName = "/home/oem/catkin_ws/src/Olders_helper/audio/voice.wav";
    std::string wav_file_path;
    nh_.getParam("wav_file_path", wav_file_path);
    
    xf_tts::XF_TTS tts_client(nh_);

    tts_client.makeTextToWav(start, wav_file_path.c_str());
    tts_client.PlayWav();

    // 必须得按NodeHandle的例程来写，不要加ConstPtr，函数必须是bool返回值
    ros::ServiceServer tts_srv = nh_.advertiseService("xf_tts", &xf_tts::XF_TTS::TextToSpeech, &tts_client);

    ros::spin();

    return 0;
}
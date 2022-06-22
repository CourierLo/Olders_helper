#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <olders_helper/robotAction.h>
#include <move_base_msgs/MoveBaseAction.h> 
#include <iostream>
#include <string>
#include <tf/transform_datatypes.h>
#include <boost/bind.hpp>
#include <audio_client.h>
#include <MQTTClient.h>
#include <std_msgs/String.h>
#include <sound_play/SoundRequestAction.h>
#include <sound_play/SoundRequestGoal.h>
#include <olders_helper/tts_text.h>

// don't split checking and broadcasting by now
// for now, we get the number and then speak it out.
namespace robot_action{
    enum STATUS {
        RESET = 0, 
        CHECK_LIGHT, CHECK_GAS, CHECK_TEMP,
        BROADCAST_LIGHT, BROADCAST_GAS, BROAD_TEMP,
        SWITCH_LIGHT, SWITCH_STOVE, SWITCH_FAN, SWITCH_CURTAIN,
        REACH_LIVINGROOM, REACH_BEDROOM, REACH_KITCHEN,
        TOUR, BROADCAST_STATUS, 
        LIGHT_ON, LIGHT_OFF, FAN_ON, FAN_OFF, CURTAIN_ON, CURTAIN_OFF, STOVE_ON, STOVE_OFF
    } status;

    enum SITE {
        ORIGIN = 0, LIVING_ROOM, BEDROOM, KITCHEN
    } site;

    enum TRIG_NUM{
        TEMP_TRIG = 24, LIGHT_TRIG = 500, GAS_TRIG = 0
    } trig;

    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    typedef actionlib::SimpleActionClient<sound_play::SoundRequestAction> SoundPlayClient;

    #define GOALS_NUM 5
    #define QOS 0
    #define ON (int)1
    #define OFF (int)0
    #define INV (int)-1

    class RobotAction{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<olders_helper::robotAction> as_;
        std::string action_name_;
        olders_helper::robotFeedback feedback_;
        MoveBaseClient* move_base_ac_ptr;
        move_base_msgs::MoveBaseGoal candidate_goal[GOALS_NUM];
        std::string sites_name[GOALS_NUM] = { "origin", "living_room", "bedroom", "kitchen" };
        // 候选目标点以及传感器的数值直接读参数服务器的param
        double x, y, yaw;
        //AudioClient audio_client;
        double light_intensity, temperature, humidity;
        int smokeExist;
        int state_fan = 0, state_stove = 1, state_curtain = 1, state_light = 0;

        MQTTClient client;
        MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
        MQTTClient_message publish_msg = MQTTClient_message_initializer;
        MQTTClient_deliveryToken token;

        bool turnOffCurtainFlag = false;

        // TTS and Player
        olders_helper::tts_text text_srv;
        SoundPlayClient* sound_play_client_ptr;
        ros::ServiceClient tts_client;
        std::string wav_file_path;

    public:
        RobotAction(std::string name);
        
        // 收到客户端发送goal后的回调函数
        // 应该不需要在Tour行为的连接处加点延时，因为audio_client和action是一个线程
        // TODO: need to modify feedback and result
        void excuteCB(const olders_helper::robotGoalConstPtr &goal);
            
        // 客户端发送cancel后执行的回调函数
        void preemptCB();


        void initAllGoals();

        // 只是想用来测试关窗帘的计时器好不好使，结果是还不如直接sleep掉, abandon
        void turnOffCurtain(const ros::TimerEvent& event);

        bool headForLocation(SITE loc);

        bool ttsAndPlay(std::string& text);

        void applianceSwitch(int id, int op);
    };
}

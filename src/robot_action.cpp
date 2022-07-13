#include <robot_action.h>

// don't split checking and broadcasting by now
// for now, we get the number and then speak it out.
namespace robot_action{
    RobotAction::RobotAction(std::string name) : 
            as_(nh_, name, boost::bind(&RobotAction::excuteCB, this, _1), false), 
            action_name_(name) {
        as_.registerPreemptCallback(boost::bind(&RobotAction::preemptCB, this));
        as_.start();

        move_base_ac_ptr = new MoveBaseClient("/move_base", true);
        ROS_INFO("Waiting for move_base server to start...");
        move_base_ac_ptr->waitForServer();
        ROS_INFO("Successfully connect to server.");
        initAllGoals();
        
        // TTS and Player
        tts_client = nh_.serviceClient<olders_helper::tts_text>("xf_tts");     // tts client
        sound_play_client_ptr = new SoundPlayClient("/sound_play", true);      // player action client
        ROS_INFO("Waiting for tts server to start...");
        sound_play_client_ptr->waitForServer();
        ROS_INFO("Successfully connect to server.");
        nh_.getParam("wav_file_path", wav_file_path);
        ROS_INFO("\033[32mWav File Path is: %s \033[0m", wav_file_path.c_str());

        std::string speech_text = "欢迎使用E S A C实验室助老机器人系统， 有什么可以帮到您？";
        ttsAndPlay(speech_text);

        // MQTT
        conn_opts.keepAliveInterval = 1800;
        conn_opts.cleansession = 1;
        MQTTClient_create(&client, "tcp://192.168.2.248:1883", "mqtt_publish_client", MQTTCLIENT_PERSISTENCE_NONE, NULL);
        int rv;
        if ((rv = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
            printf("MQTTClient_connect failure:%s\n", strerror(errno));
            return ;
        }
        publish_msg.qos = QOS;
        publish_msg.retained = 0;
        ROS_INFO("Connect to MQTT server successfully.");
    }
    
    void RobotAction::turnOffCurtain(const ros::TimerEvent& event){
        std::string payload = "sw1_off";
        publish_msg.payload = (void *)payload.c_str();
        publish_msg.payloadlen = payload.length();
        MQTTClient_publishMessage(client, "m1_switch_topic", &publish_msg, &token);
        state_curtain = OFF;
        turnOffCurtainFlag = true;
    }

    bool RobotAction::ttsAndPlay(std::string text){
        // XF TTS Service
        text_srv.request.text = text;
        tts_client.call(text_srv);

        // Sound play
        sound_play::SoundRequestGoal goal;
        goal.sound_request.arg = wav_file_path;
        goal.sound_request.sound = goal.sound_request.PLAY_FILE;
        goal.sound_request.command = goal.sound_request.PLAY_ONCE;
        goal.sound_request.volume = 20.0;
        sound_play_client_ptr->sendGoal(goal);

        // check whether get preempted or not
        ros::Rate r(10);
        while(sound_play_client_ptr->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
            if(as_.isPreemptRequested()){
                // no need to set result
                return false;
            }
            r.sleep();
        }

        return true;
    }

    bool RobotAction::headForLocation(SITE loc){
        ros::Rate r(10);
        move_base_ac_ptr->sendGoal(candidate_goal[loc]);
        while(move_base_ac_ptr->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
            if(as_.isPreemptRequested()){
                // no need to set result
                return false;
            }
            r.sleep();
        }
        return true;
    }

    void RobotAction::applianceSwitch(int id, int op){
        std::string topic = "m" + boost::to_string(id) + "_switch_topic";
        std::string payload = "sw" + boost::to_string(id) + (op ? "_on" : "_off");
        publish_msg.payload = (void *)(payload.c_str());
        publish_msg.payloadlen = payload.length();
        MQTTClient_publishMessage(client, topic.c_str(), &publish_msg, &token);
        ros::Duration(0.25).sleep();
    }

    // 收到客户端发送goal后的回调函数
    // 应该不需要在Tour行为的连接处加点延时，因为audio_client和action是一个线程
    // TODO: need to modify feedback and result
    void RobotAction::excuteCB(const olders_helper::robotGoalConstPtr &goal){
        int command = goal->command;
        ros::Rate r(10);
        feedback_.progress.clear();
        std::string speech_text;
        switch(command){
            case TOUR:
                if(!ttsAndPlay("开始巡逻")) return;
            case REACH_LIVINGROOM:
                //audio_client.receive_audio_and_play("前往客厅");
                speech_text = "前往客厅";
                if(!ttsAndPlay(speech_text))  return;
                if(!headForLocation(LIVING_ROOM))  return;
                //audio_client.receive_audio_and_play("到达客厅");
                speech_text = "到达客厅";
                if(!ttsAndPlay(speech_text))  return ;
                feedback_.progress = "reached living room.";
                as_.publishFeedback(feedback_);
                if(command != TOUR) break;

            case CHECK_LIGHT:
                nh_.getParam("/illuminance", light_intensity);
                ROS_INFO("light intensity is: %lf", light_intensity);
                //audio_client.receive_audio_and_play("当前光照强度为" + boost::to_string(light_intensity));
                speech_text = "当前光照强度为" + boost::to_string(light_intensity);
                if(!ttsAndPlay(speech_text))  return;
                feedback_.progress = "checked lights.";
                as_.publishFeedback(feedback_);
                if(command != TOUR) break;

            case SWITCH_LIGHT:
                feedback_.progress = "switching lights...";
                as_.publishFeedback(feedback_);
                if(command == TOUR && light_intensity <= LIGHT_TRIG){
                    //audio_client.receive_audio_and_play("检测到光线太弱，为您打开灯和窗帘。");
                    speech_text = "检测到光线太弱，为您打开灯和窗帘。";
                    if(!ttsAndPlay(speech_text))  return;
                    // turn on lights: s4 ~ s5
                    applianceSwitch(4, ON);  applianceSwitch(5, ON);
                    state_light = ON;
                }
                else if(command != TOUR){
                    // switch
                    // 这里最好获取状态，人也有可能关灯的, 但是不知道如何获取，那只能只能根据保存的状态判断了
                    int next_state = state_light ? OFF : ON;
                    applianceSwitch(4, next_state);  applianceSwitch(5, next_state);
                    state_light ^= state_light;  // 异或就是取反
                    break;
                }
            
            case SWITCH_CURTAIN:
                // TODO： 我不知道是哪个switch， 所以这里要改
                feedback_.progress = "switching curtain...";
                as_.publishFeedback(feedback_);
                if(command == TOUR && light_intensity <= LIGHT_TRIG){
                    //audio_client.receive_audio_and_play("检测到光线太弱，为您打开灯和窗帘。");
                    // turn on lights: s1
                    applianceSwitch(1, ON);

                    // 计时器并不好用
                    // ros::Timer timer = nh_.createTimer(ros::Duration(5.0), boost::bind(&RobotAction::turnOffCurtain, this, _1));
                    // ros::Rate r(30);
                    // while(!turnOffCurtainFlag){
                    //     ros::spinOnce();
                    //     r.sleep();
                    // }
                    //turnOffCurtainFlag = false;

                    sleep(5);   // 因为没有给窗帘编码，所以得如此丑陋地停掉电机
                    applianceSwitch(1, OFF);
                }
                else if(command != TOUR){
                    // switch
                    // 这里最好获取状态，人也有可能关灯的, 但是不知道如何获取，那只能只能根据保存的状态判断了
                    if(state_curtain == 1)  applianceSwitch(1, ON);
                    else if(state_curtain == 0) applianceSwitch(1, INV);
                    sleep(5);
                    applianceSwitch(1, OFF);
                    state_curtain ^= state_curtain;  // 异或就是取反
                    break;
                }

            case REACH_BEDROOM:
                //audio_client.receive_audio_and_play("前往卧室");
                speech_text = "前往卧室";
                if(!ttsAndPlay(speech_text)) return;
                if(!headForLocation(BEDROOM)) return;
                //audio_client.receive_audio_and_play("到达卧室");
                speech_text = "到达卧室";
                if(!ttsAndPlay(speech_text)) return;
                feedback_.progress = "reached bedroom.";
                as_.publishFeedback(feedback_);
                if(command != TOUR) break;

            case CHECK_TEMP:
                nh_.getParam("/temprature", temperature);
                nh_.getParam("/humidity", humidity);
                ROS_INFO("Current temp and humidity is: %lf, %lf", temperature, humidity);
                //audio_client.receive_audio_and_play("当前温度为" + boost::to_string(temperature) + ", 湿度为" + boost::to_string(humidity));
                speech_text = "当前温度为" + boost::to_string(temperature) + ", 湿度为" + boost::to_string(humidity);
                if(!ttsAndPlay(speech_text))  return;
                feedback_.progress = "checked temperature.";
                as_.publishFeedback(feedback_);
                if(command != TOUR) break;

            case SWITCH_FAN:
                feedback_.progress = "switching fans...";
                as_.publishFeedback(feedback_);
                if(command == TOUR && temperature > TEMP_TRIG){
                    //audio_client.receive_audio_and_play("检测到室内温度较高，为您打开风扇。");
                    speech_text = "检测到室内温度较高，为您打开风扇。";
                    if(!ttsAndPlay(speech_text))  return;
                    // turn on fans: s2 ~ s3
                    applianceSwitch(2, ON);  applianceSwitch(3, ON);
                    state_fan = ON;
                }
                else if(command != TOUR) {
                    // switch fans
                    int next_state = state_fan ? OFF : ON;
                    applianceSwitch(2, next_state);  applianceSwitch(3, next_state);
                    state_fan ^= state_fan;
                    break;
                }

            case REACH_KITCHEN:
                //audio_client.receive_audio_and_play("前往厨房");
                speech_text = "前往厨房";
                if(!ttsAndPlay(speech_text))  return;
                if(!headForLocation(KITCHEN)) return;
                //audio_client.receive_audio_and_play("到达厨房");
                speech_text = "到达厨房";
                if(!ttsAndPlay(speech_text))  return;

                feedback_.progress = "reached kitchen.";
                as_.publishFeedback(feedback_);
                if(command != TOUR)  break;

            case CHECK_GAS:
                nh_.getParam("/smokeExist", smokeExist);
                ROS_INFO("Does Smoke Exist: %d", smokeExist);
                //audio_client.receive_audio_and_play("当前烟雾浓度为" + boost::to_string(20));
                speech_text = "当前烟雾浓度为" + boost::to_string(20);
                if(!ttsAndPlay(speech_text))  return;

                feedback_.progress = "checked gas.";
                as_.publishFeedback(feedback_);

                if(command != TOUR)  break;

            case SWITCH_STOVE:
                feedback_.progress = "switching stove...";
                as_.publishFeedback(feedback_);
                if(command == TOUR && smokeExist > GAS_TRIG){
                    //audio_client.receive_audio_and_play("检测到有较大烟雾，为您关闭煤气灶。");
                    speech_text = "检测到有较大烟雾，为您关闭煤气灶。";
                    if(!ttsAndPlay(speech_text))  return;
                    // turn off stove
                    applianceSwitch(6, OFF);
                    state_stove = OFF;
                }
                else if(command != TOUR) {
                    // switch
                    int next_state = state_stove ? OFF : ON;
                    applianceSwitch(6, next_state);
                    state_stove ^= state_stove;

                    break;
                }
            
            case RESET:
                // 回原点
                //audio_client.receive_audio_and_play("我回去罚站了...");
                speech_text = "我回去罚站了...";
                if(!ttsAndPlay(speech_text))  return;
                candidate_goal[ORIGIN].target_pose.header.stamp = ros::Time::now();
                move_base_ac_ptr->sendGoal(candidate_goal[ORIGIN]);
                while(move_base_ac_ptr->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
                    if(as_.isPreemptRequested()){
                        // no need to set result
                        return;
                    }
                    r.sleep();
                }
                feedback_.progress = "back to the origin.";
                as_.publishFeedback(feedback_);
                break;
            
            // 接下来是为了适应安卓写下的代码，我个人仍为只需要模拟开关就可以了，但是安卓那边把开灯关灯等分开了，那就只好分开写算了，这样搞代码就很丑
            case LIGHT_ON:
                applianceSwitch(4, ON);  applianceSwitch(5, ON);
                break;
            case LIGHT_OFF:
                applianceSwitch(4, OFF);  applianceSwitch(5, OFF);
                break;
            case FAN_ON:
                applianceSwitch(2, ON);  applianceSwitch(3, ON);
                break;
            case FAN_OFF:
                applianceSwitch(2, OFF);  applianceSwitch(3, OFF);
                break;
            case CURTAIN_ON:
                applianceSwitch(1, ON);
                break;
            case CURTAIN_OFF:  
                applianceSwitch(1, OFF);
                break;
            case STOVE_ON:
                applianceSwitch(6, ON);
                break;
            case STOVE_OFF:
                applianceSwitch(6, OFF);
                break;
            case TELL_A_JOKE:
                ttsAndPlay("两天占领基辅’特别军事行动现已进入第三个月。");
                break;
            case INTRODUCING:
                ttsAndPlay("我是来自广东工业大学计算机学院ESAC实验室的助老小车！但别指望我是铁臂阿童木嗷。");
                break;
            case GREETING:
                ttsAndPlay("你好呀，有什么可以帮到您的吗？");
                break;
            case FALLEN:
                ttsAndPlay("警告，警告，有人摔倒了！正在拨打紧急电话求助......");
                sleep(15);
                break;
            default:
                ttsAndPlay("我不懂你在说什么呢。您能再说一遍吗？");
                break;
        }

        // 整个过程没有发生抢占，设为成功
        feedback_.progress = "tour finished.";
        as_.publishFeedback(feedback_);
        as_.setSucceeded();

        return ;
    }

    // 客户端发送cancel后执行的回调函数
    void RobotAction::preemptCB(){
        ROS_ERROR("[%s] I got preempted !", action_name_.c_str());

        // 如果在跑， cancel, 严格意义上说，语音也要终止，所以也得can掉
        if(move_base_ac_ptr->getState() == actionlib::SimpleClientGoalState::ACTIVE)    move_base_ac_ptr->cancelGoal();
        if(sound_play_client_ptr->getState() == actionlib::SimpleClientGoalState::ACTIVE)   sound_play_client_ptr->cancelGoal();

        as_.setPreempted();
    }

    void RobotAction::initAllGoals(){
        for(site = ORIGIN; site <= KITCHEN; site = (SITE)(site + 1)){
            nh_.getParam("/" + sites_name[site] + "/x", x);  
            nh_.getParam("/" + sites_name[site] + "/y", y);  
            nh_.getParam("/" + sites_name[site] + "/yaw", yaw);

            ROS_INFO("%s : %lf, %lf, %lf", sites_name[site].c_str(), x, y, yaw);

            candidate_goal[site].target_pose.header.frame_id = "map";
            candidate_goal[site].target_pose.pose.position.x = x;
            candidate_goal[site].target_pose.pose.position.y = y;
            candidate_goal[site].target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
    }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_action_node");

  robot_action::RobotAction rac("robot_action_server");
  ros::spin();

  return 0;
}
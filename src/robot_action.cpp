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
        initAllGoals();

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

    // 收到客户端发送goal后的回调函数
    // 应该不需要在Tour行为的连接处加点延时，因为audio_client和action是一个线程
    // TODO: need to modify feedback and result
    void RobotAction::excuteCB(const olders_helper::robotGoalConstPtr &goal){
        int command = goal->command;
        ros::Rate r(2);
        feedback_.progress.clear();
        switch(command){
            case RESET:
                // 回原点
                audio_client.receive_audio_and_play("我回去罚站了...");
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

            case TOUR:
            case REACH_LIVINGROOM:
                audio_client.receive_audio_and_play("前往客厅");
                move_base_ac_ptr->sendGoal(candidate_goal[LIVING_ROOM]);
                while(move_base_ac_ptr->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
                    if(as_.isPreemptRequested()){
                        // no need to set result
                        return;
                    }
                    r.sleep();
                }
                audio_client.receive_audio_and_play("到达客厅");
                feedback_.progress = "reached living room.";
                as_.publishFeedback(feedback_);
                if(command != TOUR) break;

            case CHECK_LIGHT:
                nh_.getParam("/illuminance", light_intensity);
                ROS_INFO("light intensity is: %lf", light_intensity);
                audio_client.receive_audio_and_play("当前光照强度为" + boost::to_string(light_intensity));
                feedback_.progress = "checked lights.";
                as_.publishFeedback(feedback_);
                if(command != TOUR) break;

            case SWITCH_LIGHT:
                feedback_.progress = "switching lights...";
                as_.publishFeedback(feedback_);
                if(command == TOUR && light_intensity <= LIGHT_TRIG){
                    audio_client.receive_audio_and_play("检测到光线太弱，为您打开灯和窗帘。");
                    // turn on lights: s4 ~ s5
                    for(int i = 4; i <= 5; ++i){
                        std::string topic = "m" + boost::to_string(i) + "_switch_topic";
                        std::string payload = "sw" + boost::to_string(i) + "_on";
                        publish_msg.payload = (void *)(payload.c_str());
                        publish_msg.payloadlen = payload.length();
                        MQTTClient_publishMessage(client, topic.c_str(), &publish_msg, &token);
                        ros::Duration(0.15).sleep();
                    }
                    state_light = ON;
                }
                else if(command != TOUR){
                    // switch
                    // 这里最好获取状态，人也有可能关灯的, 但是不知道如何获取，那只能只能根据保存的状态判断了
                    for(int i = 4; i <= 5; ++i){
                        std::string topic = "m" + boost::to_string(i) + "_switch_topic";
                        std::string payload = "sw" + boost::to_string(i) + (state_fan ? "_off" : "_on");
                        publish_msg.payload = (void *)(payload.c_str());
                        publish_msg.payloadlen = payload.length();
                        MQTTClient_publishMessage(client, topic.c_str(), &publish_msg, &token);
                        ros::Duration(0.15).sleep();
                    }
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
                    std::string payload = "sw1_on";
                    publish_msg.payload = (void *)payload.c_str();
                    publish_msg.payloadlen = payload.length();
                    MQTTClient_publishMessage(client, "m1_switch_topic", &publish_msg, &token);
                    state_light = OFF;

                    ros::Timer timer = nh_.createTimer(ros::Duration(5.0), boost::bind(&RobotAction::turnOffCurtain, this, _1));
                    ros::Rate r(30);
                    while(!turnOffCurtainFlag){
                        ros::spinOnce();
                        r.sleep();
                    }
                    turnOffCurtainFlag = false;
                }
                else if(command != TOUR){
                    // switch
                    // 这里最好获取状态，人也有可能关灯的, 但是不知道如何获取，那只能只能根据保存的状态判断了
                    std::string payload = "sw1";
                    payload += state_light == 0 ? "_off" : "_on";
                    publish_msg.payload = (void *)(payload.c_str());
                    publish_msg.payloadlen = payload.length();
                    MQTTClient_publishMessage(client, "m1_switch_topic", &publish_msg, &token);
                    state_curtain ^= state_curtain;  // 异或就是取反
                    break;
                }

            case REACH_BEDROOM:
                audio_client.receive_audio_and_play("前往卧室");
                move_base_ac_ptr->sendGoal(candidate_goal[BEDROOM]);
                while(move_base_ac_ptr->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
                    if(as_.isPreemptRequested()){
                        // no need to set result
                        return;
                    }
                    r.sleep();
                }
                audio_client.receive_audio_and_play("到达卧室");
                feedback_.progress = "reached bedroom.";
                as_.publishFeedback(feedback_);
                if(command != TOUR) break;

            case CHECK_TEMP:
                nh_.getParam("/temprature", temperature);
                nh_.getParam("/humidity", humidity);
                ROS_INFO("Current temp and humidity is: %lf, %lf", temperature, humidity);
                audio_client.receive_audio_and_play("当前温度为" + boost::to_string(temperature) + ", 湿度为" + boost::to_string(humidity));
                feedback_.progress = "checked temperature.";
                as_.publishFeedback(feedback_);
                if(command != TOUR) break;

            case SWITCH_FAN:
                feedback_.progress = "switching fans...";
                as_.publishFeedback(feedback_);
                if(command == TOUR && temperature > TEMP_TRIG){
                    audio_client.receive_audio_and_play("检测到室内温度较高，为您打开风扇。");
                    // turn on fans: s2 ~ s3
                    for(int i = 2; i <= 3; ++i){
                        std::string topic = "m" + boost::to_string(i) + "_switch_topic";
                        std::string payload = "sw" + boost::to_string(i) + "_on";
                        publish_msg.payload = (void *)(payload.c_str());
                        publish_msg.payloadlen = payload.length();
                        MQTTClient_publishMessage(client, topic.c_str(), &publish_msg, &token);
                        ros::Duration(0.15).sleep();
                    }
                    state_fan = ON;
                }
                else if(command != TOUR) {
                    // switch fans
                    for(int i = 2; i <= 3; ++i){
                        std::string topic = "m" + boost::to_string(i) + "_switch_topic";
                        std::string payload = "sw" + boost::to_string(i) + (state_fan ? "_off" : "_on");
                        publish_msg.payload = (void *)(payload.c_str());
                        publish_msg.payloadlen = payload.length();
                        MQTTClient_publishMessage(client, topic.c_str(), &publish_msg, &token);
                        ros::Duration(0.15).sleep();
                    }
                    state_fan ^= state_fan;
                    break;
                }

            case REACH_KITCHEN:
                audio_client.receive_audio_and_play("前往厨房");
                move_base_ac_ptr->sendGoal(candidate_goal[KITCHEN]);
                while(move_base_ac_ptr->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
                    if(as_.isPreemptRequested()){
                        // set result
                        return;
                    }
                    r.sleep();
                }
                audio_client.receive_audio_and_play("到达厨房");
                feedback_.progress = "reached kitchen.";
                as_.publishFeedback(feedback_);
                if(command != TOUR)  break;

            case CHECK_GAS:
                nh_.getParam("/smokeExist", smokeExist);
                ROS_INFO("Does Smoke Exist: %d", smokeExist);
                audio_client.receive_audio_and_play("当前烟雾浓度为" + boost::to_string(20));
                feedback_.progress = "checked gas.";
                as_.publishFeedback(feedback_);
                if(command != TOUR)  break;

            case SWITCH_STOVE:
                feedback_.progress = "switching stove...";
                as_.publishFeedback(feedback_);
                if(command == TOUR && 21 > GAS_TRIG){
                    audio_client.receive_audio_and_play("检测到有较大烟雾，为您关闭煤气灶。");
                    // turn off stove

                }
                else if(command != TOUR) {
                    // switch

                    break;
                }
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

        //虽然说直接cancel不太好，但是也能这么做
        move_base_ac_ptr->cancelGoal();
        // 严格意义上说，语音也要终止，所以也得can掉

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
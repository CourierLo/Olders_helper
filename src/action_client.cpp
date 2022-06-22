#include <ros/ros.h>
#include <olders_helper/robotAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <audio_client.h>

typedef actionlib::SimpleActionClient<olders_helper::robotAction> Client;

class ActionClient{
protected:
    ros::NodeHandle nh;
    olders_helper::robotGoal goal;
    actionlib::SimpleActionClient<olders_helper::robotAction>* ac_ptr;
    // AudioClient player;
    
public:
    ActionClient(){
        ac_ptr = new actionlib::SimpleActionClient<olders_helper::robotAction>("robot_action_server", true);
        ROS_INFO("Waiting for robot action server to start...");
        ac_ptr->waitForServer();
        ROS_INFO("Successfully connect to server.");

        //player.receive_audio_and_play("欢迎使用E S A C实验室助老机器人系统， 有什么可以帮到您？");
        // 必须得bind，不然编译编译不通过，原因不知道
        // 而且你的subscriber必须得加ss<class>
        ros::Subscriber operation_sub = nh.subscribe<std_msgs::Int32>("operation", 100, boost::bind(&ActionClient::operationCB, this, _1));
        ros::Subscriber operation_str_sub = nh.subscribe<std_msgs::String>("operation_str", 100, boost::bind(&ActionClient::operationStrCB, this, _1));

        ros::spin();
    }



    void operationCB(const std_msgs::Int32::ConstPtr& msg){
        ROS_INFO("Receive command code: %d", msg->data);
        if(ac_ptr->getState() == actionlib::SimpleClientGoalState::ACTIVE)  ac_ptr->cancelGoal();

        goal.command = msg->data;

        // 用了类就得bind
        // Client::SimpleActiveCallback(), Client::SimpleFeedbackCallback()
        ac_ptr->sendGoal(goal, boost::bind(&ActionClient::finishCB, this, _1, _2), boost::bind(&ActionClient::startCB, this), boost::bind(&ActionClient::feedbackCB, this, _1));
    }

    void sendInstruction(int op){
        if(ac_ptr->getState() == actionlib::SimpleClientGoalState::ACTIVE)  ac_ptr->cancelGoal();

        goal.command = op;
        ac_ptr->sendGoal(goal, boost::bind(&ActionClient::finishCB, this, _1, _2), boost::bind(&ActionClient::startCB, this), boost::bind(&ActionClient::feedbackCB, this, _1));
    }

    void operationStrCB(const std_msgs::String::ConstPtr& msg){
        ROS_INFO("\033[32mReceive command string: %s \033[0m", msg->data);
        // enum STATUS {
        //     RESET = 0, 
        //     CHECK_LIGHT = 1, CHECK_GAS = 2, CHECK_TEMP = 3,
        //     BROADCAST_LIGHT = 4, BROADCAST_GAS = 5, BROAD_TEMP = 6,
        //     SWITCH_LIGHT = 7, SWITCH_STOVE = 8, SWITCH_FAN = 9, SWITCH_CURTAIN = 10,
        //     REACH_LIVINGROOM = 11, REACH_BEDROOM = 12, REACH_KITCHEN = 13,
        //     TOUR = 14, BROADCAST_STATUS = 15
        //     LIGHT_ON = 16, LIGHT_OFF, FAN_ON, FAN_OFF, CURTAIN_ON, CURTAIN_OFF, STOVE_ON, STOVE_OFF
        // } status;
        if(msg->data.find("客厅"))          sendInstruction(11);
        else if(msg->data.find("卧室"))     sendInstruction(12);
        else if(msg->data.find("厨房"))     sendInstruction(13);
        else if(msg->data.find("开始巡逻"))  sendInstruction(14);
        else if(msg->data.find("开光"))     sendInstruction(16);
        else if(msg->data.find("关灯"))     sendInstruction(17);
        else if(msg->data.find("打开风扇"))  sendInstruction(18);
        else if(msg->data.find("关闭风扇"))  sendInstruction(19);
        else if(msg->data.find("煤气检测"))  sendInstruction(2);
        else if(msg->data.find("打开窗帘"))  sendInstruction(20);
        else if(msg->data.find("关闭窗帘"))  sendInstruction(21);
    }

    void finishCB(const actionlib::SimpleClientGoalState& state, const olders_helper::robotResultConstPtr& res){
        ROS_WARN("Action status:[%s]", state.getText());
        //res是空的所以不管它了
    }

    void startCB(){
        ROS_INFO("ActionServer start action...");
    }

    void feedbackCB(const olders_helper::robotFeedbackConstPtr& feedback){
        ROS_INFO("Client get feedback:[%s]", feedback->progress);
    }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "action_client");
    ActionClient robot_ac;


    return 0;
}
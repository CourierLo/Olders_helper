#include <ros/ros.h>
#include <olders_helper/robotAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>
#include <audio_client.h>

typedef actionlib::SimpleActionClient<olders_helper::robotAction> Client;

class ActionClient{
protected:
    ros::NodeHandle nh;
    olders_helper::robotGoal goal;
    actionlib::SimpleActionClient<olders_helper::robotAction>* ac_ptr;
    AudioClient player;
    
public:
    ActionClient(){
        ac_ptr = new actionlib::SimpleActionClient<olders_helper::robotAction>("robot_action_server", true);
        ROS_INFO("Waiting for action server to start...");
        ac_ptr->waitForServer();
        ROS_INFO("Successfully connect to server.");

        player.receive_audio_and_play("欢迎使用E S A C实验室助老机器人系统， 有什么可以帮到您？");
        // 必须得bind，不然编译编译不通过，原因不知道
        // 而且你的subscriber必须得加ss<class>
        ros::Subscriber operation_sub = nh.subscribe<std_msgs::Int32>("operation", 100, boost::bind(&ActionClient::operationCB, this, _1));

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

    void finishCB(const actionlib::SimpleClientGoalState& state, const olders_helper::robotResultConstPtr& res){
        ROS_WARN("Action status:[%s]", state.toString().c_str());
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
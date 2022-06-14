#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <MQTTClient.h>
#include <string>
#include <iostream>
#include <ros/ros.h>

#define QOS 1

MQTTClient client;
MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
MQTTClient_message publish_msg = MQTTClient_message_initializer;
MQTTClient_deliveryToken token;

void turnOffCurtain(const ros::TimerEvent& event){
    std::string payload = "sw1_off";
    publish_msg.payload = (void *)payload.c_str();
    publish_msg.payloadlen = payload.length();
    MQTTClient_publishMessage(client, "m1_switch_topic", &publish_msg, &token);
    std::cout << "Send turn off message...\n";
}

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_action_node");
    ros::NodeHandle nh_;
    conn_opts.keepAliveInterval = 1800;
    conn_opts.cleansession = 1;
    MQTTClient_create(&client, "tcp://192.168.2.248:1883", "mqtt_publish_client", MQTTCLIENT_PERSISTENCE_NONE, NULL);
    int rv;
    std::cout << "tring to connect the server..." << std::endl;
    if ((rv = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        printf("MQTTClient_connect failure:%s\n", strerror(errno));
        return -1;
    }
    publish_msg.qos = QOS;
    publish_msg.retained = 0;
    
    std::cout << "try to turn on...\n";
    std::string payload;
    // turn on lights: s1
    payload = "sw1_on";
    publish_msg.payload = (void *)payload.c_str();
    publish_msg.payloadlen = payload.length();
    MQTTClient_publishMessage(client, "m1_switch_topic", &publish_msg, &token);

    // turn on fans
    for(int i = 2; i <= 5; ++i) {
        std::string topic = "m" + std::to_string(i) + "_switch_topic";
        std::string payload = "sw" + std::to_string(i) + "_on";
        std::cout << topic << "     " << payload << "\n";
        publish_msg.payload = (void *)(payload.c_str());
        publish_msg.payloadlen = payload.length();
        MQTTClient_publishMessage(client, topic.c_str(), &publish_msg, &token);
        ros::Duration(0.1).sleep();
    }

    std::cout << "turn on OK.\n";

    ros::Timer timer = nh_.createTimer(ros::Duration(5.0), turnOffCurtain);

    // sleep(5);

    // std::cout << "try to turn off...\n";
    // // turn off lights: s1
    // payload = "sw1_off";
    // publish_msg.payload = (void *)payload.c_str();
    // publish_msg.payloadlen = payload.length();
    // MQTTClient_publishMessage(client, "m1_switch_topic", &publish_msg, &token);

    // // turn on fans
    // for(int i = 2; i <= 5; ++i) {
    //     std::string topic = "m" + std::to_string(i) + "_switch_topic";
    //     std::string payload = "sw" + std::to_string(i) + "_off";
    //     publish_msg.payload = (void *)(payload.c_str());
    //     publish_msg.payloadlen = payload.length();
    //     MQTTClient_publishMessage(client, topic.c_str(), &publish_msg, &token);
    // }
    // std::cout << "turn off OK.\n";
    ros::spin();

    return 0;
}
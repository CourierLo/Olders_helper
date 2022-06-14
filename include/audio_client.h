#include<netinet/in.h>   // sockaddr_in
#include<sys/types.h>    // socket
#include<sys/socket.h>   // socket
#include<stdio.h>        // printf
#include<stdlib.h>       // exit
#include<string.h>       // bzero
#include<unistd.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<string>

#define SERVER_PORT 5652
#define CLIENT_PORT 6789
#define BUFFER_SIZE 1024
#define FILE_NAME_MAX_SIZE 512

class AudioClient{
public:
    AudioClient();
    void receive_audio_and_play(const std::string& speech);

private:
    int client_socket_fd;
    struct sockaddr_in client_addr;

    struct sockaddr_in server_addr;
    socklen_t server_addr_length;

    char text[FILE_NAME_MAX_SIZE + 1];
    char audio_filename[FILE_NAME_MAX_SIZE + 1] = "/home/pi/catkin_ws/src/Olders_helper/audio/audio.wav";
    char buffer[BUFFER_SIZE];
};
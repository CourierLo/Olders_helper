#include <audio_client.h>

AudioClient::AudioClient(){
    // 声明并初始化一个客户端的socket地址结构
    bzero(&client_addr, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_addr.s_addr = htons(INADDR_ANY);
    client_addr.sin_port = htons(CLIENT_PORT);

    // 声明一个服务器端的socket地址结构，并用服务器那边的IP地址及端口对其进行初始化，用于后面的连接
    // 此处可修改服务器IP地址
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    // ESAC服务器地址 "10.21.15.156"， 127.0.0.1用于本地测试
    if(inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr) == 0) {
        perror("Server IP Address Error:");
        exit(1);
    }
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr_length = sizeof(server_addr);
}

void AudioClient::receive_audio_and_play(const std::string& speech){
    // 创建socket，若成功，返回socket描述符
    client_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(client_socket_fd < 0) {
        perror("Create Socket Failed:");
        exit(1);
    }
 
    // 绑定客户端的socket和客户端的socket地址结构 非必需
    if(-1 == (bind(client_socket_fd, (struct sockaddr*)&client_addr, sizeof(client_addr)))) {
        perror("Client Bind Failed:");
        exit(1);
    }

    // 向服务器发起连接，连接成功后client_socket_fd代表了客户端和服务器的一个socket连接
    printf("ready to connect...\n");
    if(connect(client_socket_fd, (struct sockaddr*)&server_addr, server_addr_length) < 0) {
        perror("Can Not Connect To Server IP:");
        exit(0);
    }

    // 文件只要一个 并放到缓冲区buffer中等待发送
    // test
    strcpy(text, speech.c_str());
    printf("%s\n", text);

    bzero(buffer, BUFFER_SIZE);
    strncpy(buffer, text, strlen(text) > BUFFER_SIZE?  BUFFER_SIZE : strlen(text));
    
    // 向服务器发送buffer中的数据, 根据那边的要求说不要把整个buffer发过去
    if(send(client_socket_fd, buffer, strlen(text) > BUFFER_SIZE?  BUFFER_SIZE : strlen(text), 0) < 0) {
        perror("Send File Name Failed:");
        exit(1);
    }

    // 打开文件，准备写入
    FILE *fp = fopen(audio_filename, "w");
    if(NULL == fp) {
        printf("File:\t%s Can Not Open To Write\n", audio_filename);
        exit(1);
    }
    
    // 从服务器接收数据到buffer中
    // 每接收一段数据，便将其写入文件中，循环直到文件接收完并写完为止
    bzero(buffer, BUFFER_SIZE);
    int length = 0;
    while((length = recv(client_socket_fd, buffer, BUFFER_SIZE, 0)) > 0)
    {
        if(fwrite(buffer, sizeof(char), length, fp) < length)
        {
            printf("File:\t%s Write Failed\n", text);
            break;
        }
        bzero(buffer, BUFFER_SIZE);
    }
 
    // 接收成功后，关闭文件，关闭socket
    printf("Receive File:\t%s From Server IP Successful!\n", text);
    fclose(fp);

// -----------------NLP组的要求--------------------
    // // 给他发个success
    // strcpy(buffer, "success");
    // // 向服务器发送buffer中的数据
    // if(send(client_socket_fd, buffer, BUFFER_SIZE, 0) < 0) {
    //     perror("Send File Name Failed:");
    //     exit(1);
    // }
// ---------------end sending success--------------

    close(client_socket_fd);

    // 用sox播放器播放， 小车先得安装sox
    char cmd[100] = "play /home/pi/catkin_ws/src/Olders_helper/audio/audio.wav";
    system(cmd);

    return ;
}
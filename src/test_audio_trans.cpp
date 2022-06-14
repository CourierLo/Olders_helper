#include "audio_client.h"

int main(){
    AudioClient audio_client;

    audio_client.receive_audio_and_play("下午好");

    // 本地没问题

    return 0;
}
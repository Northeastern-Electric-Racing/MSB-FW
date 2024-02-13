
#include "api/msb_common.h"
#include "api/msb_can.h"

#define MAX_SIZE 25

char* queue[MAX_SIZE];
int queue_index;
int running = 0;

void loop_publish_can() {
    while (running) {
        // every x amount of time, output queue to can
        // for now just print each string
    }
}

void start_can_publisher() {
    running = 1;
    loop_publish_can();
}

void stop_can_publisher() {
    running = 0;
}

short push_can_queue(char* message) {
    if (queue_index == MAX_SIZE) return 0;
    
    queue[queue_index++] = message;
    return 1;
}

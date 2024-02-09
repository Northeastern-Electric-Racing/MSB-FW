
#include "api/msb_common.h"
#include "api/msb_can.h"

char* queue[25];
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

void push_can_queue(string message) {
    queue[queue_index++] = message;
}

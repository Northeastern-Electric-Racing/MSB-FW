
#include "api/msb_main.h"
#include "api/msb_can.h"

osThreadId_t canTaskHandle;

void init_msb() {
    // spawn CAN output thread
    // canTaskHandle = osThreadNew(start_can_publisher, NULL, NULL);
    start_can_publisher();
}

void start_msb() {

    init_msb();

    while (1) {
        // loop query tasks
    }

}


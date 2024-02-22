
#include "api/msb_main.h"
#include "api/msb_can.h"
#include "cmsis_os.h"

//Thread ID identifies the thread.
osThreadId_t canTaskHandle;

void init_msb() {
    // Create a thread to run start_can_publisher
    canTaskHandle = osThreadNew(start_can_publisher, NULL, NULL);
    if (canTaskHandle == NULL) {
        // Handle error, need implementation in the future
    }
}

// test
void start_msb() {

    init_msb();

    while (1) {
        // loop query tasks
    }

}

void stop_msb() {
    if (canTaskHandle != NULL) {
        osThreadTerminate(canTaskHandle);
        //put Null for now, implementation in the future
        canTaskHandle = NULL;
    }
}

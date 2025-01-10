/// All commonly changed settings and constants that could effect multiple factors, esp. in telemetry

// DELAYS

// MONITOR
#define DELAY_TEMP_SENSOR_REFRESH 500
#define DELAY_IMU_REFRESH	  500
#define DELAY_SHOCKPOT_REFRESH	  500
#define DELAY_STRAIN_REFRESH	  500
#define DELAY_TOF_REFRESH	  500
#define DELAY_WHEEL_TEMP_REFRESH  500

// CONTROLLER
#define DELAY_DEBUG_LED_REFRESH 250

#define DELAY_CAN_DISPATCH 2

// CAN IDS
#define CANID_TEMP_SENSOR  0x602
#define CANID_IMU_ACCEL	   0x603
#define CANID_IMU_GYRO	   0x604
#define CANID_STRAIN_SENSE 0x605
#define CANID_SHOCK_SENSE  0x606
#define CANID_TOF	   0x607
#define CANID_WHEEL_TEMP   0x608

// Sensors to use, comment out to disable

// internal
#define CAN_ENABLE

//on central
#define SENSOR_TEMP
#define SENSOR_SHOCKPOT
#define SENSOR_STRAIN
//#define SENSOR_TOF

// #define SENSOR_IMU       //NOTE
// on knuckle or /wheel
//#define SENSOR_WHEEL_TEMP

// VERBOSE LOGGING
#define LOG_VERBOSE

//Note
//STACK SIZES
#define CAN_DISPATCH_STACK_SIZE 128 * 8 //can_dispatch_handle
#define LED_CONTROLLER_STACK_SIZE 32 * 8 //led_controller_handle
#define DEFAULT_TASK_STACK_SIZE 128 * 4 //defaultTaskHandle
#define TEMP_MONITOR_STACK_SIZE 64 * 8 //temp_monitor_handle
#define IMU_MONITOR_STACK_SIZE 128 * 8 //imu_monitor_handle
#define TOF_MONITOR_STACK_SIZE 128 * 8 //tof_monitor_handle
#define SHOCKPOT_MONITOR_STACK_SIZE 64 * 8 //shockpot_monitor_handle
#define STRAIN_MONITOR_STACK_SIZE 64 * 8 //strain_monitor_handle
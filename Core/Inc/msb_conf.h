/// All commonly changed settings and constants that could effect multiple factors, esp. in telemetry

// DELAYS
#define DELAY_CAN_DISPATCH		  5
#define DELAY_TEMP_SENSOR_REFRESH 500
#define DELAY_IMU_REFRESH		  500
#define DELAY_SHOCKPOT_REFRESH	  500
#define DELAY_STRAIN_REFRESH	  500
#define DELAY_TOF_REFRESH		  500
#define DELAY_WHEEL_TEMP_REFRESH  500

#define DELAY_DEBUG_LED_REFRESH 250

// CAN IDS
#define CANID_TEMP_SENSOR  0x602
#define CANID_IMU_ACCEL	   0x603
#define CANID_IMU_GYRO	   0x604
#define CANID_STRAIN_SENSE 0x605
#define CANID_SHOCK_SENSE  0x606
#define CANID_TOF		   0x607
#define CANID_WHEEL_TEMP   0x608

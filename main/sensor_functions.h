/*!
 * @file sensor_functions.h
 * @brief Compilation of sensor function prototypes
 *
 * @author 
 * @version  V1.0
 * @date  2023-03-17
 */

#ifndef sensor_functions_h
#define sensor_functions_h

// setup serial communication and join i2c bus
void setup_serial_i2c();

// setup ultrasonic srf02 sensor
void setupUltrasonic();

// setup tof vl53l0x sensor
void setupTOF();

// read nnn cm of ultrasonic sensor distance detection
float readUltra();

// read nnnn.nn mm of TOF sensor distance detection
int readTOF();

#endif

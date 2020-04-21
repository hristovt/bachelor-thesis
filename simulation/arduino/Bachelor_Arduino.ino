#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
MPU9250 IMU(Wire,0x68);
int status;
float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;

// Create a union to easily convert float to byte
typedef union{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

// Create the variable you want to send
FLOATUNION_t a_x;
FLOATUNION_t a_y;
FLOATUNION_t a_z;


void setup() {
  // serial to display data
  Serial.begin(115200);
  // start communication with IMU 
  status = IMU.begin();

}

void loop() {
  IMU.readSensor();
  
  acc_x=IMU.getAccelX_mss();
  acc_y=IMU.getAccelY_mss();
  acc_z=IMU.getAccelZ_mss();

  gyro_x=IMU.getGyroX_rads();
  gyro_y=IMU.getGyroY_rads();
  gyro_z=IMU.getGyroZ_rads();

  a_x.number = acc_x;
  a_y.number = acc_y;
  a_z.number = acc_z;

  Serial.write('A'); 

  for (int i=0; i<4; i++){
   Serial.write(a_x.bytes[i]);
  }

  for (int i=0; i<4; i++){
   Serial.write(a_y.bytes[i]);
  }

  for (int i=0; i<4; i++){
   Serial.write(a_z.bytes[i]);
  }
      
  Serial.print('\n');
  
  delay(50);
}

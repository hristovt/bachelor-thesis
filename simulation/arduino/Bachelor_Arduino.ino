#include "MPU9250.h"
#include "MadgwickAHRS.h"
#include "Servo.h"
#include "PID_v1.h"
#include "math.h"
#define PIN_SERVO_P 9
#define PIN_SERVO_Y 10

typedef union{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t delta_p;
FLOATUNION_t delta_y;

MPU9250 IMU(Wire, 0x68); //I2C conntection with the IMU
int status;

float ax, ay, az, gx, gy, gz, mx, my, mz; //Sensor accelerometar, gyro and magnetometar x,y and z components
float ag = 0.101972;                      //Transformation from m/s^2 to G's
float pitch, yaw, roll;

Madgwick filter;
unsigned long microsPerReading, microsPrevious;

double Setpoint_p, Input_p, Output_p;
float Kp_p=1;
float Kd_p=1.5;
float Ki_p=0;
PID tvc_p(&Input_p, &Output_p, &Setpoint_p, Kp_p, Ki_p, Kd_p, DIRECT);

double Setpoint_y, Input_y, Output_y;
float Kp_y=1;
float Kd_y=1.5;
float Ki_y=0;
PID tvc_y(&Input_y, &Output_y, &Setpoint_y, Kp_y, Ki_y, Kd_y, DIRECT);


unsigned long time;

Servo servo_p;
Servo servo_y;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  status = IMU.begin();                                 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_5HZ);

  IMU.calibrateAccel();
  IMU.calibrateGyro();
  //IMU.calibrateMag();
  
  filter.begin(25);
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();

  Input_p=pitch;
  Setpoint_p=0;
  tvc_p.SetMode(AUTOMATIC);
  tvc_p.SetSampleTime(250);
  //tvc_p.SetOutputLimits(0, 10);
  servo_p.attach(PIN_SERVO_P);
  servo_p.write(90);

  Input_y=yaw;
  Setpoint_y=0;
  tvc_y.SetMode(AUTOMATIC);
  tvc_y.SetSampleTime(250);
  //tvc_y.SetOutputLimits(-10, 10);
  servo_y.attach(PIN_SERVO_Y);
  servo_y.write(90);
}

void loop() {
  unsigned long microsNow;
  microsNow = micros();
  time=millis();
  if (microsNow - microsPrevious >= microsPerReading) {

    IMU.readSensor();

    //M/s^2 to G's
    ax = IMU.getAccelX_mss() * ag;
    ay = IMU.getAccelY_mss() * ag;
    az = IMU.getAccelZ_mss() * ag;

    //Rad/s to deg/s
    gx = IMU.getGyroX_rads() * (180 / PI);
    gy = IMU.getGyroY_rads() * (180 / PI);
    gz = IMU.getGyroZ_rads() * (180 / PI);


    filter.updateIMU(gx, gy, gz, ax, ay, az);

    
    pitch = filter.getPitch();
    
    
    yaw = filter.getRoll();
    
    
    if(yaw>=-180 && yaw<=0){
      yaw=(yaw+180.0f)*-1;
    }
    
    if(yaw>0 && yaw<=180){
      yaw=(yaw-180.0f)*-1;
    }

    Serial.print("Pitch: ");
    Serial.print(pitch, 2);
    Serial.print("\t");

    Serial.print("Yaw: ");
    Serial.print(yaw, 2);
    Serial.print("\t");



    if(time>40000){
      if(pitch<0){
        tvc_p.SetControllerDirection(DIRECT);
        Input_p=round(pitch);
        tvc_p.Compute();
        servo_p.write(90+Output_p*-1);
        delta_p.number=Output_p*-1;
      }

      if(pitch>0){
        tvc_p.SetControllerDirection(REVERSE);
        Input_p=round(pitch);
        tvc_p.Compute();
        servo_p.write(90+Output_p);
        delta_p.number=Output_p;
      }

      if(yaw<0){
        tvc_y.SetControllerDirection(DIRECT);
        Input_y=round(yaw);
        tvc_y.Compute();        
        servo_y.write(90+Output_y);
        delta_y.number=Output_y;
      }

       if(yaw>0){
        tvc_y.SetControllerDirection(REVERSE);
        Input_y=round(yaw);
        tvc_y.Compute();
        servo_y.write(90+Output_y*-1);
        delta_y.number=Output_y*-1;
      }

      Serial.print("Output_y: ");
      Serial.print(Output_y);
      Serial.print("\t");

      Serial.print("Output_p: ");
      Serial.print(Output_p);
      Serial.print("\n");

        //Serial.write('A');

        //for (int i=0; i<4; i++){
        //   Serial.write(delta_p.bytes[i]);
        //}

        //for (int i=0; i<4; i++){
        //   Serial.write(delta_y.bytes[i]);
        //}
        
        //Serial.print("\n");
    }

    microsPrevious = microsPrevious + microsPerReading;
  }

  delay(50);
}

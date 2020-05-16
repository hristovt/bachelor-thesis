#include "MPU9250.h"
#include "MadgwickAHRS.h"
#include "Servo.h"          //Including the libraries that wil be needed for the code.
#include "PID_v1.h"         //The ones that are harder to find will be on the repository.
#include "math.h"
//---------------------------------------------------------//
#define PIN_SERVO_P 9        //Defining the pins for the servos
#define PIN_SERVO_Y 10
//---------------------------------------------------------//
typedef union{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;                 //Converting the TVC angles into bits to transfer them, through Serial connection, 
                                //to the Simulink Simulation
FLOATUNION_t delta_p;
FLOATUNION_t delta_y;
//--------------------------------------------------------//
MPU9250 IMU(Wire, 0x68);                                  //I2C conntection with the IMU
int status;
//--------------------------------------------------------//
float ax, ay, az, gx, gy, gz;                             //Sensor accelerometar and gyro  x,y and z components
float ag = 0.101972;                                      //Transformation from m/s^2 to G's
float pitch, yaw, roll;
//--------------------------------------------------------//
Madgwick filter;                                          //Creating the filter object
unsigned long microsPerReading, microsPrevious;           //Variables for keeping track of the filter
//--------------------------------------------------------//
double Setpoint_p, Input_p, Output_p;
float Kp_p=1;                                             //PID controller for the pitch angle motor
float Kd_p=1.5;
float Ki_p=0;
PID tvc_p(&Input_p, &Output_p, &Setpoint_p, Kp_p, Ki_p, Kd_p, DIRECT);
//--------------------------------------------------------//
double Setpoint_y, Input_y, Output_y;
float Kp_y=1;                                             //PID controller for the yaw angle motor
float Kd_y=1.5;
float Ki_y=0;
PID tvc_y(&Input_y, &Output_y, &Setpoint_y, Kp_y, Ki_y, Kd_y, DIRECT);
//--------------------------------------------------------//
unsigned long time;

Servo servo_p;                                            //Declaring the servo motors, pitch and yaw
Servo servo_y;
//--------------------------------------------------------//
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
                                                 
  status = IMU.begin();                                   // Start communication with IMU 
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");                             //If the communication with the IMU is unsuccessful, it will display a message
    Serial.println(status);
    while(1) {}
  }
                                
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);           //Setting the IMU settings
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_5HZ);

  IMU.calibrateAccel();                                   //Callibrating the IMU
  IMU.calibrateGyro();
  
  filter.begin(25);                                       
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();

  Input_p=pitch;                                          //Setting the PID controller for the pitch angle servo
  Setpoint_p=0;
  tvc_p.SetMode(AUTOMATIC);
  tvc_p.SetSampleTime(250);
  //tvc_p.SetOutputLimits(0, 10);

  Input_y=yaw;                                            //Setting the PID controller for the yaw angle servo
  Setpoint_y=0;
  tvc_y.SetMode(AUTOMATIC);
  tvc_y.SetSampleTime(250);
  //tvc_y.SetOutputLimits(-10, 10);
    
  servo_p.attach(PIN_SERVO_P);                            //Setting the servo for the pitch angle
  servo_p.write(90);
  
  servo_y.attach(PIN_SERVO_Y);                            //Setting the servo for the yaw angle
  servo_y.write(90);
}

void loop() {
  unsigned long microsNow;
  microsNow = micros();
  time=millis();
  
  if (microsNow - microsPrevious >= microsPerReading) {

    IMU.readSensor();                                    //Read the IMU values
    
    ax = IMU.getAccelX_mss() * ag;                       //Converting from m/s^2 to G's
    ay = IMU.getAccelY_mss() * ag;
    az = IMU.getAccelZ_mss() * ag;

    gx = IMU.getGyroX_rads() * (180 / PI);               //Converting from rad/s to deg/s
    gy = IMU.getGyroY_rads() * (180 / PI);
    gz = IMU.getGyroZ_rads() * (180 / PI);

    filter.updateIMU(gx, gy, gz, ax, ay, az);            //Giving the filter the acc and gyro values

    pitch = filter.getPitch();                           //Getting the pitch and yaw angles
    yaw = filter.getRoll();                              //The yaw is equal to the roll is because of the possition of the IMU
    
    if(yaw>=-180 && yaw<=0){                             //Converting the yaw angle to start from 0 degrees and to go + or - from there
      yaw=(yaw+180.0f)*-1;
    }
    
    if(yaw>0 && yaw<=180){
      yaw=(yaw-180.0f)*-1;
    }

    if(time>40000){                                     //Waiting 40 seconds for the IMU to callibrate and then to start to calcualte with the PID
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
//--------------------------------------------------------//
        Serial.write('A');

        for (int i=0; i<4; i++){
           Serial.write(delta_p.bytes[i]);
        } 
                                                          //Sending the PID values to to the Simulink simulation
        for (int i=0; i<4; i++){
           Serial.write(delta_y.bytes[i]);
        }
        
        Serial.print("\n");
//--------------------------------------------------------//      
    }

    microsPrevious = microsPrevious + microsPerReading;
  }

  delay(50);
}

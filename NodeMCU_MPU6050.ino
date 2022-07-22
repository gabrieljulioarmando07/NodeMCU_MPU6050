#include <Wire.h>
float gyro_x,gyro_y,gyro_z;
float gyro_x_cal, gyro_y_cal, gyro_z_cal;
float angle_pitch,angle_roll;
float accel_x,accel_y,accel_z,acc_total_vector,angle_pitch_acc, angle_roll_acc;
long loop_timer,loop_timerprev;
float elapsed_time;
boolean set_gyro_angles;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Wire.setClock(400000);
  setup_mpu_6050_registers();
  calibrate_gyro();
  Serial.println("Gyro Calibrated");
  loop_timer = micros();
}

void loop() {
  loop_timerprev = loop_timer;            //This three lines used for getting amount of time passed
  loop_timer = micros();
  elapsed_time = (float)(loop_timer-loop_timerprev)/(float)1000000;
  
  // Calling functions bellow
  read_gyro();
  read_accelerometer();

  //This code make sure we get the angles from all condition
  if(set_gyro_angles){
  // This code blend the angles from both gyro and accelerometer, so the result will be more accurate
  angle_roll = angle_roll * 0.996 + angle_roll_acc * 0.004;
  angle_pitch = angle_pitch * 0.996 + angle_pitch_acc * 0.004;
  }
  else{
  //This code makes sure mpu still gives right value although the calculation is not begining on the flat surface
  angle_roll = angle_roll_acc;
  angle_pitch = angle_pitch_acc;
  set_gyro_angles = true;
  }
  Serial.print("Pitch : ");
  Serial.print(angle_pitch);Serial.print("  ");
  Serial.print("Roll : ");
  Serial.print(angle_roll);Serial.println("  ");
}

//Code for obtain the angles from the gyroscope
void read_gyro(){
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);

  gyro_x = (int16_t)(Wire.read()<<8 |Wire.read());
  gyro_y = (int16_t)(Wire.read()<<8 |Wire.read());
  gyro_z = (int16_t)(Wire.read()<<8 |Wire.read());

  //reduce angles with calibration result
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  angle_pitch += gyro_x * elapsed_time * 0.01526717557;
  angle_roll += gyro_y * elapsed_time * 0.01526717557;

  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  }

//Code for obtain the angles from accelerometer
void read_accelerometer(){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);

  accel_x = (int16_t)(Wire.read()<<8 |Wire.read());
  accel_y = (int16_t)(Wire.read()<<8 |Wire.read());
  accel_z = (int16_t)(Wire.read()<<8 |Wire.read());

  acc_total_vector = sqrt((accel_x * accel_x)+(accel_y * accel_y)+(accel_z * accel_z));
  angle_pitch_acc = asin((float)accel_y/acc_total_vector)*57.296;
  angle_roll_acc = asin((float)accel_x/acc_total_vector)* -57.296;
  }

//Code for Calibrating Gyro
void calibrate_gyro(){
  for(int i = 0; i< 2000; i++){
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);

  gyro_x = (int16_t)(Wire.read()<<8 |Wire.read());
  gyro_y = (int16_t)(Wire.read()<<8 |Wire.read());
  gyro_z = (int16_t)(Wire.read()<<8 |Wire.read());

  gyro_x_cal += gyro_x;
  gyro_y_cal += gyro_y;
  gyro_z_cal += gyro_z;
  delay(3);
    }
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
  }

void setup_mpu_6050_registers(){
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  //Accelerometer 8G
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission(true);                                          //End the transmission
  
  //Gyro 1000dps ---> 32.8 LSB
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission(true);                                          //End the transmission
}

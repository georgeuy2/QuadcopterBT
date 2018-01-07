///////////////////////////////////////////////////////////////////////////////////////
// Name: George Uy de Ong II
// CS193 Individual Project 
// Drone/Quadcopter controlled by Bluetooth with Android app
// Board: Arduino Uno
// Gyro: MPU-6050 gyro
// Parts list is located in the folder called "BOM DroneBT"
//
// Warning: Please remove the propellers before flying the drone if its not ready to fly. 
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>                          // Wire library is used for the MPU-6050 gyro and acceleramotor 
#include <EEPROM.h>                        // EEPROM library is used to store information of gyro and joystick data
#include <SoftwareSerial.h>                // SoftwareSerial library is used for Bluetooth 

const byte rx_pin = 0;                      // Connect TX pin from Bluetooth to RX of the Arduino Uno
const byte tx_pin = 1;
SoftwareSerial BTserial(rx_pin, tx_pin);

///////////////////////////////////////////////////////////////////////////////////////
//  PID VARIABLES
///////////////////////////////////////////////////////////////////////////////////////
float p_roll = 2.6;                        // P gain roll
float i_roll = 0.07;                       // I gain roll
float d_roll = 20.0;                       // D gain roll
int MAX_ROLL = 100;                        // Absolute max value for PID for roll

float p_pitch = p_roll;                    // P gain pitch (Same values as roll)
float i_pitch = i_roll;                    // I gain pitch (Same values as roll)
float d_pitch = d_roll;                    // D gain pitch (Same values as roll)
int MAX_PITCH = MAX_ROLL;                  // Absolute max value for PID for pitch

float p_yaw = 4.0;                         // P gain yaw
float i_yaw = 0.02;                        // P gain yaw
float d_yaw = 0.0;                         // P gain yaw
int MAX_YAW = 100;                         // Absolute max value for PID for yaw

///////////////////////////////////////////////////////////////////////////////////////
//  Global Variables
///////////////////////////////////////////////////////////////////////////////////////
const float SPIRIT_Y = 0, SPIRIT_X = 0;    // For first time, this must be edited before flying the drone
                                           // You may, get a tiled drone.
float current_i_roll, current_i_pitch, current_i_yaw;
float setpoint_roll, setpoint_pitch, setpoint_yaw;
float input_gyro_roll, input_gyro_pitch, input_gyro_yaw;
float output_roll, output_pitch, output_yaw;
float prev_roll_error, prev_pitch_error, prev_yaw_error;
float accel_angle_x, accel_angle_y, angle_x, angle_y;    
float roll_lvl, pitch_lvl;
float pid_error;

unsigned long timer_ch1, timer_ch2, timer_ch3, timer_ch4, esc_timer, loop_timer;

long accel_x, accel_y, accel_z, total_accel;

volatile int rx_ch1, rx_ch2, rx_ch3, rx_ch4;

int esc1, esc2, esc3, esc4;
int throttle, battery_voltage, start, temperature, calibration_count, gyro_addr;
int accel_axis[4], gyro_axis[4], rx_input[5];
int low_channel[5], center_channel[5], high_channel[5];


double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];

const byte number_bytes = 32;
byte rx_bytes[number_bytes];
byte numbers_rx = 0;
byte eeprom_data[36];
boolean gyro_angles_set;
boolean auto_lvl;                 
boolean new_data;
boolean check_crc;

///////////////////////////////////////////////////////////////////////////////////////
//  Setup
///////////////////////////////////////////////////////////////////////////////////////
void setup(){
  // Local variables
  // Mapping range high, center, and low for throttle, pitch, roll, and yaw
  int data[13] = { 1010,  1999,  1599,           // Roll: low | high | center
                   1010,  1999,  1500,           // Pitch: low | high | center     
                   1000,  1999,  1502,           // Throttle: low | high | center 
                   1010,  1990,  1612 };         // Yaw: low | high | center                                 
  int countPause = 0;                            // A counter to display ". . . ."
  // Setting variables
  auto_lvl = true;
  new_data = false;
  check_crc = false;
  
  //Declaring D4-D7 and B12 B13 as outputs
  DDRD |= B11110000;                                                       
  DDRB |= B00110000;                                                       
  
  //Serial.begin(115200);
  //intro();
  BTserial.begin(115200);
  
  // Get datas from EEPROM 
  for(start = 0; start <= 35; start++){
    eeprom_data[start] = EEPROM.read(start);
  }
  start = 0;                                     // Reset start to zero
  gyro_addr = eeprom_data[32];                   // Storing gyro address                   

  // Setting data to corresponding low, high, and center channels                  
  low_channel[1] = data[0]; high_channel[1] = data[1];  center_channel[1] = data[2];
  low_channel[2] = data[3]; high_channel[2] = data[4];  center_channel[2] = data[5];
  low_channel[3] = data[6]; high_channel[3] = data[7];  center_channel[3] = data[8];
  low_channel[4] = data[9]; high_channel[4] = data[10];  center_channel[4] = data[11];

  // Set receiver inputs to 1500 and throttle to 1000
  rx_input[1] = 1599;                           // Roll
  rx_input[2] = 1500;                           // Pitch
  //rx_input[3] = 1000;                         // Throttle
  rx_input[4] = 1500;                           // Yaw
  
  Wire.begin();                                 // Start I2C

  TWBR = 12;                                    // Setting I2C clock speed of 400kHz

  digitalWrite(12,HIGH);                        // Turn on LED
  
  // Check EEPROM is setup correctly
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B'){
    delay(10);
  }
 
  // Checks MPU-6050 is working. If not, stop program.
  if(eeprom_data[31] == 2 || eeprom_data[31] == 3){
    delay(10);
  }

  set_gyro_reg();                          // Set each gyro registers

  // Before calibrating, wait 5 seconds while setting motors high and low to avoid beeping.
  for (calibration_count = 0; calibration_count < 1250 ; calibration_count ++){                           
    PORTD |= B11110000;                          // Set ports high
    delay(1);                                    // Wait 1000us
    PORTD &= B00001111;                          // Set ports low                          
    delay(3);                                    // Wait 3000us                          
  }
  // Serial.print("calibrating.");
  countPause = 0;
  // Takes a bunch of gyro samples and get the average offset
  // Collecting 2000 samples
  for (calibration_count = 0; calibration_count < 2000 ; calibration_count ++){
    // Flashing LED that indicates drone is calibrating                          
    if(calibration_count % 15 == 0){
      digitalWrite(12, !digitalRead(12));         
    }
    if(countPause < 200){
      countPause++;
    }
    else {
     // Serial.print(".");
      countPause = 0;
    }
    
    get_gyro_data();                              // Get gyro data
    
    // Adding roll, pitch, yaw values to roll_cal, pitch_cal, yaw_call, respectively.
    gyro_axis_cal[1] += gyro_axis[1];                                       
    gyro_axis_cal[2] += gyro_axis[2];                                      
    gyro_axis_cal[3] += gyro_axis[3];                                    
    
    // Setting motors to high and low to avoid beeping.
    PORTD |= B11110000;                                                     
    delay(1);                                                
    PORTD &= B00001111;                                                    
    delay(3);                                                  
  } // end of calibration loop
  
  //Serial.println();
  // Getting gyro average offset by dividing 2000 since we collected 2000 samples.
  gyro_axis_cal[1] /= 2000;                                                
  gyro_axis_cal[2] /= 2000;                                                
  gyro_axis_cal[3] /= 2000;             

  //Serial.println("Drone is ready!");
  
  // Waiting until user set the throttle to the lowest position
  // This is for safety to avoid drone flying up right after calibration.
  while(rx_ch3 < 990 || rx_ch3 > 1120 || rx_ch4 < 1400){
    start++;                                      
    rx_ch3 = convert_channel(3);                  // Convert BT throttle signals to 1000 to 2000
    rx_ch4 = convert_channel(4);                  // Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    
    // Again, while waiting we set motors low and high to avoid annoying beeps
    PORTD |= B11110000;                                                     
    delay(1);                                               
    PORTD &= B00001111;                                                    
    delay(3);      

    // About 500ms change the LED status                                                      
    if(start == 125){                            
      digitalWrite(12, !digitalRead(12)); 
      start = 0;                                            
    }
  }// end of while loop
  
  start = 0;                                       // Reset start to zero.

  // To calculate the battery volatge we also have to consider the diode and the analog read 
  // Since we are using a Lipo battery that has a maximum voltage of 12.6V and the range of analog we want to read is from 0-1023,
  // We divide 1260/1023 = 1.2317
  // Also, we have to consider the diode, which is 65nw. 
  battery_voltage = (analogRead(0) + 65) * 1.2317;

  loop_timer = micros();                           // Set the current time to loop_timer

  // When the drone is ready, turn off the LED.
  digitalWrite(12,LOW);                                            
}

///////////////////////////////////////////////////////////////////////////////////////
//  Main Loop
///////////////////////////////////////////////////////////////////////////////////////
void loop(){

  // The MPU-6050 sensitivity scale is 65.5 = 1 degree per sec
  // This information is located in the MPU-6050 datasheet at page 12 in parameter called gyroscope sensitivity.
  input_gyro_yaw = (input_gyro_yaw * 0.7) + ((gyro_yaw / 65.5) * 0.3);     
  input_gyro_roll = (input_gyro_roll * 0.7) + ((gyro_roll / 65.5) * 0.3);   
  input_gyro_pitch = (input_gyro_pitch * 0.7) + ((gyro_pitch / 65.5) * 0.3);
  

  // The refresh rate is every 4ms = 250Hz.
  // We can get the gyro output for every 4ms. Therefore, we do (raw_gyro_data/250/65.5) = travel angle or
  // 0.0000611 = 1 / (250Hz / 65.5)
  // Calculate the traveled roll and pitch angle then add them to the corresponding x and y axis (x = roll, pitch = y).
  angle_y += gyro_pitch * 0.0000611;             
  angle_x += gyro_roll * 0.0000611;  

  // Since the sin functions accept radians only, we have to convert 0.0000611 to radians.
  // 0.000001066 = 0.0000611 * (pi/180) radians
  angle_y -= angle_x * sin(gyro_yaw * 0.000001066);   // If the drone turns in the yaw direction when the roll becomes the pitch               //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_x += angle_y * sin(gyro_yaw * 0.000001066);   // Similar as for the prev line of code, but pitch becomes the roll
  
  // Calcualte the total accelerometer vector
  // v_vector = sqrt(x^2+y^2+z^2)
  total_accel = sqrt((accel_x*accel_x)+(accel_y*accel_y)+(accel_z*accel_z));       

  
  if(abs(accel_y) < total_accel){                                      // Check if accel is less than total to avoid NaN because of asin
    accel_angle_y = asin((float)accel_y/total_accel)* 57.296;          // Convert 180/pi to radians = 57.296. Calculate the pitch angle.
  }
  if(abs(accel_x) < total_accel){                                      
    accel_angle_x = asin((float)accel_x/total_accel)* -57.296;         // Calculate the roll angle.
  }
  
  // This substart the actual level of the drone when it is placed in a leveled surface. 
  accel_angle_y -= SPIRIT_Y;                                  // Substart the actual level in y axis
  accel_angle_x -= SPIRIT_X;                                  // Substart the actual level in x axis
  
  angle_y = angle_y * 0.03819 + accel_angle_y * 0.045;        // This corrects the drift of the drone in y-axis.
  angle_x = angle_x * 0.03819 + accel_angle_x * 0.045;        // This corrects the drift of the drone in x-axis.

  pitch_lvl = angle_y * 15;                                   // Calculate the angle correction for the y-axis(pitch).
  roll_lvl = angle_x * 15;                                    // Calculate the angle correction for the x-axis(roll).

  // When auto_lvl is false, set pitch and roll angle correction to zero. 
  if(!auto_lvl){                                                        
    pitch_lvl = 0;                                                 
    roll_lvl = 0;                                            
  }

  // To start the motors, throttle has to be the lowest position and the yaw as to be at the leftmost position then back to center.
  if(rx_ch3 < 1150 && rx_ch4 < 1150){
    start = 1;                                                // Indicating to start the drone
  }
  
  // When yaw is at center, the motors start
  if(start == 1 && rx_ch3 < 1150 && rx_ch4 > 1400){
    start = 2;                                                // Indicating that the motors start

    // Set gyro angle = accel angle, when starting
    // This covers the case when the drone is tilted.
    angle_y = accel_angle_y;                                         
    angle_x = accel_angle_x;                                         
    gyro_angles_set = true;                                   //Set the IMU flag to true.

    // Reset PID controllers
    current_i_roll = 0;
    prev_roll_error = 0;
    current_i_pitch = 0;
    prev_pitch_error = 0;
    current_i_yaw = 0;
    prev_yaw_error = 0;
  } // end of starting motors loop

  // To stop the motors set throttle to the lowest position and yaw to the rightmost position.
  if(start == 2 && rx_ch3 < 1150 && rx_ch4 > 1950){
    start = 0;                                               // Indicating that the motors stop.
  }
  
  // The roll receiver determines the PID setpoint in degress per second
  setpoint_roll = 0;
  // To get better readings, we need to 16us deadband 
  if(rx_ch1 > 1508){
    setpoint_roll = rx_ch1 - 1508;
  }
  else if(rx_ch1 < 1492){
    setpoint_roll = rx_ch1 - 1492;
  }
  // To get degrees per second subtract from the standard roll value and divide it by 3
  setpoint_roll -= roll_lvl;                                  
  setpoint_roll /= 3.0;                                       

  // Similarly from the last few lines of code but it covers pitch
  setpoint_pitch = 0;
  // To get better readings, we need to 16us deadband 
  if(rx_ch2 > 1508){
    setpoint_pitch = rx_ch2 - 1508;
  }
  else if(rx_ch2 < 1492){
    setpoint_pitch = rx_ch2 - 1492;
  }
  setpoint_pitch -= pitch_lvl;                                 
  setpoint_pitch /= 3.0;                                         
  
  // Covers yaw
  setpoint_yaw = 0;
  if(rx_ch3 > 1150){                                           // Yaw must not be rightmost position to avoid motors turning off
    if(rx_ch4 > 1508){
      setpoint_yaw = (rx_ch4 - 1508)/3.0;
    }
    else if(rx_ch4 < 1492){
      setpoint_yaw = (rx_ch4 - 1492)/3.0;
    }
  }
  
  calculate_pid();                                             // Calculate the PID outputs

  // We need to compensate the battery voltage. We need to filter it
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  // When battery is low, turn on the LED
  if(battery_voltage < 1000 && battery_voltage > 600){
    digitalWrite(12, HIGH);
  }

  throttle = rx_ch3;                                           

  // This is where the control of the motors is after PID calculation.
  if (start == 2){                                             // Start motors.
    if (throttle > 1800){                                      // If throttle data exceeds 1800, keep it 1800 (max).
      throttle = 1800;                                  
    }
    esc1 = throttle - output_pitch + output_roll - output_yaw; // Front right motor: ESC1 pulse calculation
    esc2 = throttle + output_pitch + output_roll + output_yaw; // Rear right motor: ESC2 pulse calculation 
    esc3 = throttle + output_pitch - output_roll - output_yaw; // Rear left motor: ESC3 pulse calculation 
    esc4 = throttle - output_pitch - output_roll + output_yaw; // Front left motor: ESC4 pulse calculation 

    if (battery_voltage < 1240 && battery_voltage > 800){      // Battery must be connected
      // Consider the voltage drop for each ESC pulse
      esc1 += esc1 * ((1240 - battery_voltage)/(float)3500);   
      esc2 += esc2 * ((1240 - battery_voltage)/(float)3500);   
      esc3 += esc3 * ((1240 - battery_voltage)/(float)3500);  
      esc4 += esc4 * ((1240 - battery_voltage)/(float)3500);   
    } 

    // When the ESC is below 1100, keep the motors running.
    if (esc1 < 1100){
      esc1 = 1100;                                        
    }
    if (esc2 < 1100){
      esc2 = 1100;                                         
    }
    if (esc3 < 1100){
      esc3 = 1100;                                       
    }
    if (esc4 < 1100){
      esc4 = 1100;                                         
    }

    // ESC max value is 2000
    if(esc1 > 2000){
      esc1 = 2000;                                          
    }
    if(esc2 > 2000){
      esc2 = 2000;                    
    }
    if(esc3 > 2000){
      esc3 = 2000;                                        
    }
    if(esc4 > 2000){
      esc4 = 2000;                                 
    }
  }
  else{
    // If drone not flying, set ESC to 1000, to avoid annoying beeps
    esc1 = 1000;                                                           
    esc2 = 1000;                                                           
    esc3 = 1000;                                                     
    esc4 = 1000;                                                           
  } // end of if and else control motors

  //displayData();  

  // We want to stay within the 4000us, if not turn on the LED. This indicates we exceed 4000us!
  if(micros() - loop_timer > 4050){
    digitalWrite(12, HIGH);                  
  }
  
  // Wait until 4ms has passed
  while(micros() - loop_timer < 4000);                                      
  loop_timer = micros();                                          // Set loop timer for the next loop

  PORTD |= B11110000;                                             // Set D4-D7 high
  // Calc time of the falling edge of each ESC
  timer_ch1 = esc1 + loop_timer;                                     
  timer_ch2 = esc2 + loop_timer;                                     
  timer_ch3 = esc3 + loop_timer;                                     
  timer_ch4 = esc4 + loop_timer;                                   
  
  get_gyro_data();                                                // Get gyro data

  
  while(PORTD >= 16){                                             // Stay in loop until D4-D7 is low.
    esc_timer = micros();                                         // Get the current time.
    // Set D4-D7 when ESC is greater than the timer
    if(timer_ch1 <= esc_timer){
      PORTD &= B11101111;                
    }
    if(timer_ch2 <= esc_timer){
      PORTD &= B11011111;               
    }
    if(timer_ch3 <= esc_timer){
      PORTD &= B10111111;               
    }
    if(timer_ch4 <= esc_timer){
      PORTD &= B01111111;              
    }
  }
} // End of Main Loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Gets gyro data
void get_gyro_data(){

  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_addr);                                   
    Wire.write(0x3B);                                                     
    Wire.endTransmission();                                              
    Wire.requestFrom(gyro_addr,14);                             // Request 14 bytes from gyro                                   

    // Convert receiver data to 1000 to 2000
    rx_ch1 = convert_channel(1);                
    rx_ch2 = convert_channel(2);               
    rx_ch3 = convert_channel(3);                
    rx_ch4 = convert_channel(4);                
    
    while(Wire.available() < 14);                                // Waiting for 14 bytes of data from gyro

    // Getting all the axis accelleromet data from gyro
    accel_axis[1] = Wire.read() << 8|Wire.read();                              
    accel_axis[2] = Wire.read() << 8|Wire.read();                          
    accel_axis[3] = Wire.read() << 8|Wire.read();    
                         
    temperature = Wire.read() << 8|Wire.read();                  // Storing temperature but not needed for now...

    // Get all the axis angular data from gyro                            
    gyro_axis[1] = Wire.read()<<8|Wire.read();                              
    gyro_axis[2] = Wire.read()<<8|Wire.read();                              
    gyro_axis[3] = Wire.read()<<8|Wire.read();                           
  }

  // After calibrating, we have to substract the compensate the errors
  if(calibration_count == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                                      
    gyro_axis[2] -= gyro_axis_cal[2];                                     
    gyro_axis[3] -= gyro_axis_cal[3];                                     
  }

  // Set the gyro_roll to the right axis from EEPROM
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];                     
  // Invert the roll, if the MSB is 1
  if(eeprom_data[28] & 0b10000000){
    gyro_roll *= -1;                         
  }
  // Repeat for gyro_pitch, gyro_yaw, and accel_x/y/z
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];                     
  if(eeprom_data[29] & 0b10000000){
    gyro_pitch *= -1;                        
  }
  
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];                    
  if(eeprom_data[30] & 0b10000000){
    gyro_yaw *= -1;                          
  }

  accel_x = accel_axis[eeprom_data[29] & 0b00000011];                           
  if(eeprom_data[29] & 0b10000000){
    accel_x *= -1;                             
  }
  
  accel_y = accel_axis[eeprom_data[28] & 0b00000011];                          
  if(eeprom_data[28] & 0b10000000){
    accel_y *= -1;                             
  }
  
  accel_z = accel_axis[eeprom_data[30] & 0b00000011];                          
  if(eeprom_data[30] & 0b10000000){
    accel_z *= -1;                          
  }
}// end of get_gyro_data

void calculate_pid(){
//////////////////////////////////////////////////////////////////////////////////////////////////
  // PID calculation
  // Variables:           error = gyro-receiver
  //                      p_gain (2.6), i_gain (0.07), d_gain (20) are given in the top page of this code. 
  // Proportional part:   p_out = error*p_gain 
  // Integral part:       i_out = i_out + (error*i_gain)
  // Derivative part:     d_out = (error - prev_error)*d_gain
  // pid_out = p_out + i_oput + d_out
//////////////////////////////////////////////////////////////////////////////////////////////////  
  //Roll calculations
  pid_error = input_gyro_roll - setpoint_roll;                                                          // Error for roll
  current_i_roll += i_roll * pid_error;                                                                 // i_out = i_gain*error
  // We need to set the limit or else it will go out of control when the value goes too high
  if(current_i_roll > MAX_ROLL){
    current_i_roll = MAX_ROLL;
  }
  else if(current_i_roll < MAX_ROLL * -1){
    current_i_roll = MAX_ROLL * -1;
  }
  // pid_out = p_out + i_out + d_out
  output_roll = p_roll * pid_error + current_i_roll + d_roll * (pid_error - prev_roll_error);
  
  // Limit out the extreme values after calculation
  if(output_roll > MAX_ROLL){
    output_roll = MAX_ROLL;
  }
  else if(output_roll < MAX_ROLL * -1){
    output_roll = MAX_ROLL * -1;
  }
  // Set current error to previous error for next calculation
  prev_roll_error = pid_error;
//////////////////////////////////////////////////////////////////////////////////////////////////
// Repeat for pitch
//////////////////////////////////////////////////////////////////////////////////////////////////
  //Pitch calculations
  pid_error = input_gyro_pitch - setpoint_pitch;
  current_i_pitch += i_pitch * pid_error;
  if(current_i_pitch > MAX_PITCH){
    current_i_pitch = MAX_PITCH;
  }
  else if(current_i_pitch < MAX_PITCH * -1){
    current_i_pitch = MAX_PITCH * -1;
  }

  output_pitch = p_pitch * pid_error + current_i_pitch + d_pitch * (pid_error - prev_pitch_error);
  
  if(output_pitch > MAX_PITCH){
    output_pitch = MAX_PITCH;
  }
  else if(output_pitch < MAX_PITCH * -1){
    output_pitch = MAX_PITCH * -1;
  }

  prev_pitch_error = pid_error;
//////////////////////////////////////////////////////////////////////////////////////////////////
// Repeat for yaw
//////////////////////////////////////////////////////////////////////////////////////////////////
  //Yaw calculations
  pid_error = input_gyro_yaw - setpoint_yaw;
  current_i_yaw += i_yaw * pid_error;
  
  if(current_i_yaw > MAX_YAW){
    current_i_yaw = MAX_YAW;
  }
  else if(current_i_yaw < MAX_YAW * -1){
    current_i_yaw = MAX_YAW * -1;
  }

  output_yaw = p_yaw * pid_error + current_i_yaw + d_yaw * (pid_error - prev_yaw_error);
  
  if(output_yaw > MAX_YAW){
    output_yaw = MAX_YAW;
  }
  else if(output_yaw < MAX_YAW * -1){
    output_yaw = MAX_YAW * -1;
  }

  prev_yaw_error = pid_error;
}// end of calculate_pid

// Convert Bluetooth receiver signals to 1000-2000 values
int convert_channel(int function) {
//////////////////////////////////////////////////////////////////////////////////////////////////
// Local variables
//////////////////////////////////////////////////////////////////////////////////////////////////
  // Receiver from Bluetooth
  static boolean rx_in_progress = false;
  static byte index = 0;
  byte start_marker = 0x3C;                                            // Marker is < in ASCII
  byte end_marker = 0x3E;                                              // Marker is > is ASCII
  byte temp_rx_byte;

  // Conversion part
  //    1-bit      3-byte      3-byte     2-byte
  //  ___________________________________________
  // |    0   |   throttle  |   yaw    |   CRC   |
  // |    1   |     pitch   |   roll   |   CRC   |
  //  -------------------------------------------
  byte reverse;                                                      
  int low, center, high, actual;
  int divisor_crc = 16;                                                    // Divisor for the CRC 
  int msb = 2, second_segment = 0, third_segment = 0, forth_segment = 0;    
  int difference;
//////////////////////////////////////////////////////////////////////////////////////////////////

  // Code for the receiver
  // This code only gets data that between < and > 
  // For example we get a stream a data in ASCII like, 0 30 100 23 < 1 23 100 4 > 23 4 90
  // The code only gets 1 23 100 4
 //Serial.println("convert");
  while(BTserial.available() > 0 && new_data == false){               // Waiting to get any data from Bluetooth
    // Read from Bluetooth receiver
    temp_rx_byte = BTserial.read();

    if( rx_in_progress == true){                                      // Goes in here when data has a marker starts with '<'
      if(temp_rx_byte != end_marker){                                 // Gets data until end marker (>)
        rx_bytes[index] = temp_rx_byte;                               // Gets each string of data between < >
        index++;                                                      // Increment index so we know how long is the data
        if(index >= number_bytes){                                    // Once it is greater than 32 bytes set to 31
          index = number_bytes - 1;
        }
      }
      else{
        rx_bytes[index] = '\0';                                       // when data sending is finish
        rx_in_progress = false;                                       // set to false
        numbers_rx = index;                                           // Save the number of element that data was sent
        index = 0;                                                    // reset index
        new_data = true;                                              // set new data true, to prepare the next stream of data
      }
    }
    else if ( temp_rx_byte == start_marker){                          // when data starts with '<' (0x3C), begin collecting the stream of data
      rx_in_progress = true;
    }
 }

  // Start to convert the received data to cordinates for the drone
  // |FIRST BIT | SECOND SEGM | THIRD SEGM | FOURTH SEGM|
  // |    0     |   THROTTLE  |     YAW    |    CRC     |
  // |    1     |     PITCH   |     ROLL   |    CRC     | 
  //
  if(new_data == true){
    // The length of data we want should be 9 bits. If not, the rest of data is garbage. So, we ignore them.
    if(numbers_rx == 9){
      for(byte n= 0; n < numbers_rx; n++){
        // Convert from ASCII char to DEC
        switch(n){
          case 0:
            msb = rx_bytes[n] - 48;                                   
           break;
                
          case 1:           
              second_segment = (rx_bytes[n] - 48)*100;                       
            break;
            
          case 2:
              second_segment += (rx_bytes[n] - 48)*10;
            break;
            
          case 3:
              second_segment += (rx_bytes[n] - 48)*1;
            break;

          case 4:
              third_segment += (rx_bytes[n] - 48)*100; 
            break;
          
          case 5:
              third_segment += (rx_bytes[n] - 48)*10; 
            break;
          
          case 6:
              third_segment += (rx_bytes[n] - 48)*1;
            break;

          case 7:
              forth_segment += (rx_bytes[n] - 48)*10;
            break;

          case 8:
              forth_segment += (rx_bytes[n] - 48)*1;
            break;

          default:
             // Serial.println("ERROR");
            break;            
        }
      } // end of for loop

      // Checking if we our data was manipulated during transmission. So we use the CRC method
      // Check CRC is match. 
      int tempCRC = (msb + second_segment + third_segment)%divisor_crc;
      
      if(tempCRC == forth_segment){                                       // If match send the data accordly; using the table above
        check_crc = true;
        //displayCord(msb,second_segment,third_segment,forth_segment );

        // Mapping the cordinates from 0-254 to 1000-2000
        second_segment = map(second_segment, 235,0,1000,2000);
        third_segment = map(third_segment, 0, 245, 1000, 2000);
        
        if(msb == 0 && check_crc == true){
          // Set throttle
          rx_input[3] = second_segment;

          // Set yaw
          rx_input[4] = third_segment;
        }
        else if( msb == 1 && check_crc == true){
          // Set pitch
          rx_input[2] = second_segment;

          // Set roll
          rx_input[1] = third_segment;
        }
      } // end of if(tempCRC == forth_segment)
      else{
        check_crc = false;
      }
    } // end of if(numbers_rx == 9)
    new_data = false;
  } // end of if(new_data == true)

  

  // if reverse, invert the number range
  if(function == 1){
    reverse = 0;
  }
  else if(function == 2){
    reverse = 1;
  }
  else if(function == 3){
    reverse = 0;
  }
  else if(function == 4){
    reverse = 0;
  }

  actual = rx_input[function];                                              // Read the corresponding rx
  low = low_channel[function];                                              // Set the corresponding lowest value
  center = center_channel[function];                                        // Set the corresponding center value
  high = high_channel[function];                                            // Set the corresponding high value
  
  // When actual position of the joystick is lower than center
  if (actual < center) {
    // If the actual position of the joystick is lower than the 1000 then set it to 1000                                                     
    if (actual < low){
      actual = low;                                             
    }
    // Caluclate and scale the actual position of the joystick between the range of 1000-2000
    difference = ((long)500) *(long)(center - actual)  / (center - low);     
    // If inverted, reverse the value
    if (reverse == 1){
      return 1500 + difference;                                
    }
    else{
      return 1500 - difference;                                           
    }
  }
  // Similar when actual position of the joystick is higher than center
  else if (actual > center) {                                          
    // Do it similar from previous code but this time when the actual joystick is high                           
    if (actual > high){
      actual = high;                                           
    }
    difference = ((long)500) * (long)(actual - center) / (high - center);      
    if (reverse == 1){
      return 1500 - difference;                               
    }
    else{
      return 1500 + difference;                                           
    }
  }
  // Otherwise leave it at the center (1500)
  else return 1500;
}// end of convert_channel

void displayCord(int msb, int second_segment, int third_segment, int forth_segment){
  // Serial.print("first element is: ");
  Serial.print(msb); 
  Serial.print("\t"); //Serial.print("Second segment is: ");
  Serial.print(second_segment);
  Serial.print("\t"); //Serial.print("Third segment is: ");
  Serial.print(third_segment);  
  Serial.print("\t"); //Serial.print("Fourth segment is: ");
  Serial.println(forth_segment);

}

void displayData(){
  Serial.print("ESC 1 (RF): ");
  Serial.print(esc1);
  Serial.print("\t\t");
  Serial.print("Throttle: ");
  Serial.print(rx_ch3);
  Serial.print("\t\t");
  
  Serial.print("ESC 2 (RR): ");
  Serial.print(esc2);
  Serial.print("\t\t");
  Serial.print("Yaw: ");
  Serial.print(rx_ch4);
  Serial.print("\t\t");

  Serial.print("ESC 3 (LR): ");
  Serial.print(esc3);
  Serial.print("\t\t");
  Serial.print("Pitch: ");
  Serial.print(rx_ch2);
  Serial.print("\t\t"); 

  Serial.print("ESC 4: (LF)");
  Serial.print(esc4);
  Serial.print("\t\t");
  Serial.print("Roll: ");
  Serial.println(rx_ch1);
}

void set_gyro_reg(){
  // Refer to MPU-6050 Register Map and Descriptions in Revision pdf
  if(eeprom_data[31] == 1){
    // Turn on gyro by writing in register 0x6B (pg 40 of Register Map and Desictiption)
    Wire.beginTransmission(gyro_addr);                               // Start communication to gyro
    Wire.write(0x6B);                                                
    Wire.write(0x00);                                                          
    Wire.endTransmission();                                          // End communication to gyro
    // Set gyro configuration a full scale range of 500 degree per sec in register 0x1B and set it to 0x08
    Wire.beginTransmission(gyro_addr);                                      
    Wire.write(0x1B);                                                        
    Wire.write(0x08);                                                         
    Wire.endTransmission();                                                   
    // Set accelerometer a full scale range of 8g in register 0x1C and set it to 0x10 (8g).
    Wire.beginTransmission(gyro_addr);                                      
    Wire.write(0x1C);                                                       
    Wire.write(0x10);                                                         
    Wire.endTransmission();                                                 

    // Test if values are set correct
    Wire.beginTransmission(gyro_addr);                                      
    Wire.write(0x1B);                                                          
    Wire.endTransmission();                                                   
    Wire.requestFrom(gyro_addr, 1);                                  // Set 1 byte from the gyro
    while(Wire.available() < 1);                                     // Wait until gyro sends 6 bytes
    if(Wire.read() != 0x08){                                         // Check if the value is 0x08
      digitalWrite(12,HIGH);                                         // Turn on LED if wrong!
      while(1){
        delay(10);                                                   // Stay in loop, so drone wont fly if gyro has a problem.
      }
    }
    // Set configuration to filter accelerometer to 44Hz and gyroscope 42hz
    Wire.beginTransmission(gyro_addr);                                     
    Wire.write(0x1A);                                                         
    Wire.write(0x03);                                                     
    Wire.endTransmission();                                                  
  }  
}

void intro(){
  Serial.println(F("==================================================="));
  delay(1500);
  Serial.println(F(""));
  Serial.println(F("Welcome"));
  delay(500);
  Serial.println(F("  to"));
  delay(500);
  Serial.println(F("    Drone"));
  delay(500);
  Serial.println(F("      Controller"));
  delay(1000);
  Serial.println(F(""));
  Serial.println(F("Initializing drone"));
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  delay(1500);
  Serial.println(F("Wait..."));
}
// END OF CODE


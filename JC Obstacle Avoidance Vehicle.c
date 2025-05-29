#include "simpletools.h"
#include "servo.h"
#include "serial.h"
#include "adcDCpropab.h"

// Pins
const int servoRLPin = 16;           // Initialize rear left servo pin
const int servoRRPin = 17;           // Initialize rear right servo pin
const int USFrontTrig = 3;           // Initialize front ultrasonic trigger pin
const float USFrontEcho = 4;         // Initialize front ultrasonic echo pin
const int USRearTrig = 6;            // Initialize rear ultrasonic trigger pin
const float USRearEcho = 5;          // Initialize rear ultrasonic echo pin
const int pushbtnPin = 0;            // Initialize rear ultrasonic echo pin
const int lcdPin = 15;               // Initialize lcd pin
const int ON = 22;                   // Pin to turn on lcd
const int CLR = 12;                  // Pin to clear lcd screen
serial *lcd;                         // lcd instance created for serial

// Global Variables (readable and can be edited by all cogs)
static volatile float cmDistFront = 20;             // Front US measurement initialized to 20 cm
static volatile float cmDistRear = 0;               // Rear US measurement initialized to 0 cm
static volatile float servoSpeedTurn = 0;           // Turn vehicle from joystick
static volatile float servoSpeedForBack = 0;        // Move vehicle from joystick
static volatile float udVmap, lrVmap;               // Value of joystick readings
static volatile float pushBtn = 1;                  // 1-system off, 0-system on

// Global Flags (used in conditions and to set flag values)
static volatile int servoStatus[] = {0};          // servoStatus: 0-stop, 1-move, 2-object detected
static volatile int robotStatus[] = {0};          // Robot status: 0-off ground, 1-on ground
static volatile int objDetect[] = {0};            // Object detection: 0-no object, 1-object detected


// Front US function
void USFront()
{
  while(1)
  {
    low(USFrontTrig);                                     // trigger the sensor
    pulse_out(USFrontTrig, 10);                           // Send pulse
    cmDistFront = (pulse_in(USFrontEcho, 1)) / 58.0;      // Calculate distance in cm
    
    if(cmDistFront <= 7)                                  // Check whether object is within 7 cm of range
    {
      objDetect[0] = 1;                                   // Object is present
    }
    else
    {
      objDetect[0] = 0;                                   // Object is not present
    }
    pause(500);
  }
}


// Joystick function
void joystick()
{
  pause(1000);
  adc_init(21,20,19,18);                              // ADC pins initialized
  
  float lrV, udV;
  
  while(1)
  {
    udV = adc_volts(2);                               // Y axis values are stored to udV
    lrV = adc_volts(3);                               // X axis values are stored to lrV
    udVmap = udV - 2.6;                               // udV mapped to range -2.5 to 2.5 volts
    udVmap = ((int)(udVmap * 10.0 + 0.5)) / 10.0;     // udVmap limited to 1 decimal place
    lrVmap = lrV - 2.5;                               // lrV mapped to range -2.5 to 2.5 volts
    lrVmap = ((int)(lrVmap * 10.0 + 0.5)) / 10.0;     // lrVmap limited to 1 decimal place
    pause(100);
  }
}


void pushButton()
{
  while(1)
  {
    if (input(pushbtnPin) == 0)       // Condition checking status of push button
    {
      pushBtn = !pushBtn;             // Latch function
      pause(500);
    }
  }
}


// Modulus function
float mod(float x)
{
  if (x > 0) {
      return x;
  } else if (x < 0) {
      return -x;
  } else {
      return 0;
  }
}


// Servo motor function
void servoMotor()
{
  int counter = 0;
  while(1)
  {
    while(robotStatus[0] == 1 && objDetect[0] == 0)                   // Condition checking if robot is on ground & object is not detected
    {
      if(mod(udVmap) > mod(lrVmap))
      {
        servo_speed(servoRLPin, servoSpeedForBack);                   // Servo speed is set
        servo_speed(servoRRPin, -servoSpeedForBack);
      }
      else
      {
        if(mod(lrVmap) > mod(udVmap))
        {
          servo_speed(servoRLPin, servoSpeedTurn);
          servo_speed(servoRRPin, servoSpeedTurn);
        }
        else
        {
          servo_speed(servoRLPin, 0);
          servo_speed(servoRRPin, 0);
        }
      }
    }
    
    while(robotStatus[0] == 0)                                        // Condition checking if robot is on ground
    {
      servo_disable(servoRLPin);                                      // Servo is disabled
      servo_disable(servoRRPin);
    }
    
    while(objDetect[0] == 1 && robotStatus[0] == 1 && pushBtn == 0)   // Condition checking is object is detected & robot is on ground and push button is enabled
    {
      counter = 0;
      while(robotStatus[0] != 0 && counter <= 17)                     // Condition checking in steps to stop servo motors if robot suddenly detects it is off ground
      {
        servo_speed(servoRLPin, -100);
        servo_speed(servoRRPin, 100);
        pause(50);
        servo_speed(servoRLPin, -100);
        servo_speed(servoRRPin, -100);
        pause(50);   
        counter = counter + 1;
      }    
    }
  }
}


// Rear US function
void USRear()
{
  while(1)
  {
    low(USRearTrig);                                    // trigger the sensor
    pulse_out(USRearTrig, 10);                          // Send pulse
    cmDistRear = (pulse_in(USRearEcho, 1)) / 58.0;      // Calculate distance in cm
    
    if(cmDistRear >= 8)
    {
      robotStatus[0] = 0;                               // Robot is off the ground
    }
    if(cmDistRear < 8)
    {
      robotStatus[0] = 1;                               // Robot is on the ground
    }
    pause(100);
  }
}


// LCD function
void lcdisplay()
{
  lcd = serial_open(15, 15, 0, 9600);           // Serial communication is initialized
  writeChar(lcd, ON);                           // lcd display turned on
  writeChar(lcd, CLR);                          // lcd display cleared
  serial_txChar(lcd,17);                        // lcd backlight on
  pause(5);
  dprint(lcd, "Hello Tribus");                  // Prints statement
  
  pause(1000);
  
  while(1)
  {
    if(pushBtn == 0)
    {
      if(robotStatus[0] == 0)                   // Prints if robot is off ground & push button is enabled
      {
        writeChar(lcd, CLR);
        serial_txChar(lcd,17);
        dprint(lcd, "Robot Off Ground");
      }
      else                                      // Prints if robot is on ground & push button is enabled
      {
        writeChar(lcd, CLR);
        serial_txChar(lcd,17);
        dprint(lcd, "Robot On Ground");
      }
    }
    else                                        // Prints if push button is not enabled
    {
      writeChar(lcd, CLR);
      serial_txChar(lcd,17);
      dprint(lcd,"TRIBUS HALT");
      writeChar(lcd, 13);
      dprint(lcd,"Press Start Btn");
    }
    pause(500);
  }
}


// Main function
int main()
{
  set_direction(pushbtnPin,0);                        // Push button on joystick is set as input
  
  // Cogs are called and run for functions
  cog_run(lcdisplay,128);
  cog_run(pushButton,128);
  cog_run(joystick, 128);
  cog_run(USFront, 128);
  cog_run(USRear, 128);
  cog_run(servoMotor, 128);
  
  while(1)
  {
    pause(100);
    while(pushBtn == 0)                               // Condition checking if push button is enabled
    {
      servoSpeedTurn = lrVmap * 80;                   // lrVmap values are mapped to servo motor's range of -200 to 200
      servoSpeedForBack = udVmap * 80;                // udVmap values are mapped to servo motor's range of -200 to 200
      
      // Prints latest value of push button
      print("Push Button = %d\n",pushBtn);
      print("\n");
      
      // Prints latest value of distance sensed by US
      print("Front Distance = %f\n",cmDistFront);
      print("Rear Distance = %f\n",cmDistRear);
      print("\n");
      
      // Prints status of flags
      print("Robot Status = %d\n",robotStatus[0]);
      print("Object Detected = %d\n",objDetect[0]);
      print("\n");
      
      // Prints current values of joystick axes
      print("Left Right values = %f\n",lrVmap);
      print("Up Down values = %f\n",udVmap);
      
      putChar(HOME);                                  // Places terminal cursor back to home position
      
      pause(200);
    }      
  }
}

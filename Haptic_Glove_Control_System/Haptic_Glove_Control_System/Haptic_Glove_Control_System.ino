#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <SerialCommand.h>

SerialCommand sCmd;
/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  70 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  480 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
const int fsrPin = A0; //pin A0 to read analog input
int value; //save analog value
// our servo # counter
uint8_t servonum = 0;

// Define the number of samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input. Using a constant rather than a normal variable lets us use this
// value to determine the size of the readings array.
const int numReadings = 3;
bool isFreeSpace = true;
int readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int total = 0;              // the running total
int average = 0;            // the average
int desiredForce = 0;



void setup() {
  Serial.begin(9600);
  while (!Serial);

  sCmd.addCommand("PING", pingHandler);
  //Serial.println("8 channel Servo test!");

    // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  pinMode(LED_BUILTIN, OUTPUT);


  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

void loop() {

  updateReading();
  
  checkSerial();

  if (isFreeSpace){
    value = map(average, 0, 500, SERVOMIN+200, SERVOMAX);//Map value 0-1023 to 0-255 (PWM)

    
  } else {
    value = 150;
  }
  pwm.setPWM(servonum, 0, value);          //Send PWM value to led
  
 delay(10); 
}

void updateReading(){
  // subtract the last reading:
  total = total - readings[readIndex];
  
  // read from the sensor:
  readings[readIndex] = analogRead(fsrPin);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
}
void checkSerial(){
  // Reading in data in game project
  if (Serial.available() > 0){
    sCmd.readSerial();
  }
}

void pingHandler(const char *command){
  Serial.println("PONG");
}

void echoHandler () {
  char *arg;
  arg = sCmd.next();
  if (arg != NULL)
    Serial.println(arg);
  else
    Serial.println("nothing to echo");
}

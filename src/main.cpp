#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

#define SERVOMIN 150  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600  // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define USMIN 0       // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 3000    // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600

int servonum = 0;
int totalServo = 16;

void setup()
{
  pwm.begin();
  Serial.begin(115200);
  if (!Serial)
    return;

  Serial.println("Servo test!");

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  delay(10);
}

int angleToPulse(int ang)
{
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX); // map angle of 0 to 180 to Servo min and Servo max
  Serial.print("Angle: ");
  Serial.print(ang);
  Serial.print(" pulse: ");
  Serial.println(pulse);
  return pulse;
}

void loop()
{
  // TODO: run servo one by one
  for (int angle = 0; angle <= 180; angle++)
  {
    pwm.setPWM(servonum, 0, angleToPulse(angle));
    delay(10);
  }

  // TODO: run all servo at together
  for (int angle = 0; angle <= 180; angle++)
  {
    for (int i = 0; i < totalServo; i++)
    {
      pwm.setPWM(i, 0, angleToPulse(angle));
    }
  }

  // TODO: run each servo one at a time using pulse value
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
  {
    pwm.setPWM(servonum, 0, pulselen);
    delay(1);
  }

  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
  {
    pwm.setPWM(servonum, 0, pulselen);
    delay(1);
  }

  // TODO: run all servo at together using writeMicroseconds(), it's not precise due to calculation rounding! The writeMicroseconds() function is used to mimic the Arduino Servo library writeMicroseconds() behavior.
  for (uint16_t microsec = USMIN; microsec < USMAX; microsec += 10)
  {
    for (int i = 0; i < totalServo; i++)
    {
      pwm.writeMicroseconds(i, microsec);
      delay(2);
    }
  }

  for (uint16_t microsec = USMAX; microsec > USMIN; microsec -= 10)
  {
    for (int i = 0; i < totalServo; i++)
    {
      pwm.writeMicroseconds(i, microsec);
      delay(2);
    }
  }

  // TODO: run servo in sine wave
  float freq = 0.0005;
  float theta = millis() * freq;
  int amplitude = 180;
  pwm.setPWM(servonum, 0, angleToPulse(amplitude * sin(theta))); // based on Asin(ωθ)

  delay(50);

  servonum++;
  if (servonum > totalServo)
    servonum = 0;
}
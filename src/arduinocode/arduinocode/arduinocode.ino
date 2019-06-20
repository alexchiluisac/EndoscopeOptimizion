/*
  Arduino Bootcamp
  - Pan Tilt Assembly Control - Demo of taking readings from the Wii Nunchuk
 * ArduinoNunchukDemo.ino
 *
 * Copyright 2011-2013 Gabriel Bianconi, http://www.gabrielbianconi.com/
 *
 * Project URL: http://www.gabrielbianconi.com/projects/arduinonunchuk/
 * 
 * modified 10/30/2016
 * by: Lee Assam
 *
 */

#include <Wire.h>
#include <ArduinoNunchuk.h>

#define BAUDRATE 38400

ArduinoNunchuk nunchuk = ArduinoNunchuk();

// SCL is connected to A5
// SDA is connected to A4
// +3.3V connected to +
// GND connected to -

void setup()
{
  Serial.begin(BAUDRATE);
  nunchuk.init();
}

void loop()
{
  nunchuk.update();
  Serial.print(nunchuk.analogX);
  Serial.print(' ');
  Serial.print(nunchuk.analogY);
  Serial.print(' ');
  Serial.print(nunchuk.accelX);
  Serial.print(' ');
  Serial.print(nunchuk.accelY);
  Serial.print(' ');
  Serial.print(nunchuk.accelZ);
  Serial.print(' ');
  Serial.print(nunchuk.zButton);
  Serial.print(' ');
  Serial.print(nunchuk.cButton);
  Serial.println();
  delay(30);

}

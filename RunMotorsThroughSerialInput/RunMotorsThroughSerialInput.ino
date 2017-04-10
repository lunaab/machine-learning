/*
 Created 1 Jul 2009
 by Tom Igoe

 This example code borrowed from the public domain, modified to run small motors.

 http://www.arduino.cc/en/Tutorial/SwitchCase2
 */

void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  // initialize the motors on PWM pins 5, 6, 9, 10, and 11:
  for (int thisPin = 5; thisPin < 12; thisPin++) {
    pinMode(thisPin, OUTPUT);
  }
}

void loop() {
  // read the sensor:
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    // 'a' = run motor on PWM 5, 'b' = run motor on PWM 6, and so forth:

    switch (inByte) {
      case 'a':
        // will run motor on PWM 5
        analogWrite( 5 , 153 );  // 60% duty cycle
        delay(500);              // play for 0.5s
        analogWrite( 5 , 0 );    // 0% duty cycle (off)
        delay(2000);             // wait for 4s
        
      case 'b':
        // will run motor on PWM 6
        analogWrite( 6 , 153 );  // 60% duty cycle
        delay(500);              // play for 0.5s
        analogWrite( 6 , 0 );    // 0% duty cycle (off)
        delay(2000);             // wait for 4s
        break;
        
      case 'c':
        // will run motor on PWM 9
        analogWrite( 9 , 153 );  // 60% duty cycle
        delay(500);              // play for 0.5s
        analogWrite( 9 , 0 );    // 0% duty cycle (off)
        delay(2000);             // wait for 4s
        break;
        
      case 'd':
        // will run motor on PWM 10
        analogWrite( 10 , 153 );  // 60% duty cycle
        delay(500);               // play for 0.5s
        analogWrite( 10 , 0 );    // 0% duty cycle (off)
        delay(2000);              // wait for 4s
        break;
        
      case 'e':
        // will run motor on PWM11
        analogWrite( 11 , 153 );  // 60% duty cycle
        delay(500);               // play for 0.5s
        analogWrite( 11 , 0 );    // 0% duty cycle (off)
        delay(2000);              // wait for 4s
        break;
        
      default:
        // turn all the motors off:
        for (int thisPin = 5; thisPin < 12; thisPin++) {
          analogWrite( thisPin , 0 );
        }
    }
  }
}


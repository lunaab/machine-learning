/*
  Switch statement  with serial input

 Demonstrates the use of a switch statement.  The switch
 statement allows you to choose from among a set of discrete values
 of a variable.  It's like a series of if statements.

 To see this sketch in action, open the Serial monitor and send any character.
 The characters a, b, c, d, and e, will turn on LEDs.  Any other character will turn
 the LEDs off.

 The circuit:
 * 5 LEDs attached to digital pins 2 through 6 through 220-ohm resistors

 created 1 Jul 2009
 by Tom Igoe

This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/SwitchCase2
 */

void setup() {
  // initialize serial communication:
  Serial.begin(19200);
  // initialize the LED pins:
  for (int thisPin = 5; thisPin < 12; thisPin++) {
    pinMode(thisPin, OUTPUT);
  }
}

void loop() {
  // read the sensor:
  if (Serial.available() > 0) {

    String str = Serial.readStringUntil(',');
    if (str.equals("a")) {
      str = Serial.readStringUntil(',');
      analogWrite(5, str.toInt());
      delay(7);
      analogWrite(5, 0);
    }
    if (str.equals("b")) {
      str = Serial.readStringUntil(',');
      analogWrite(6, str.toInt());
      delay(7);
      analogWrite(6, 0);
    }
    if (str.equals("c")) {
      str = Serial.readStringUntil(',');
      analogWrite(9, str.toInt());
      delay(7);
      analogWrite(9, 0);
    }
    if (str.equals("d")) {
      str = Serial.readStringUntil(',');
      analogWrite(10, str.toInt());
      delay(7);
      analogWrite(10, 0);
    }
    if (str.equals("e")) {
      str = Serial.readStringUntil(',');
      analogWrite(11, str.toInt());
      delay(7);
      analogWrite(11, 0);
    }
    
    }
    //*int inByte = Serial.read();
    // do something different depending on the character received.
    // The switch statement expects single number values for each case;
    // in this exmaple, though, you're using single quotes to tell
    // the controller to get the ASCII value for the character.  For
    // example 'a' = 97, 'b' = 98, and so forth:

    /*switch (inByte) {
      case 'a':
        Serial.print(" PWM 5 Start ");
        analogWrite( 5 , 153 );  // 60% duty cycle
        //analogWrite( 6 , 153 );  // 60% duty cycle
        delay(7);              // play for 0.5s
        analogWrite( 5 , 0 );    // 0% duty cycle (off)
        //delay(2000);             // wait for 4s
        Serial.print(" PWM 5 End ");
        break;
      case 'b':
        analogWrite( 6 , 153 );  // 60% duty cycle
        delay(7);              // play for 0.5s
        analogWrite( 6 , 0 );    // 0% duty cycle (off)
        //delay(2000);             // wait for 4s
        Serial.print(" PWM 6 ");
        break;
      case 'c':
        analogWrite( 9 , 153 );  // 60% duty cycle
        delay(7);              // play for 0.5s
        analogWrite( 9 , 0 );    // 0% duty cycle (off)
        //delay(2000);             // wait for 4s
        Serial.print(" PWM 9 ");
        break;
      case 'd':
        analogWrite( 10 , 153 );  // 60% duty cycle
        delay(7);              // play for 0.5s
        analogWrite( 10 , 0 );    // 0% duty cycle (off)
        //delay(2000);             // wait for 4s
        Serial.print(" PWM 10 ");
        break;
      case 'e':
        analogWrite( 11 , 153 );  // 60% duty cycle
        delay(7);              // play for 0.5s
        analogWrite( 11 , 0 );    // 0% duty cycle (off)
        //delay(2000);             // wait for 4s
        //Serial.print(" PWM 11 "); 
        break;
      default:
        // turn all the LEDs off:
        for (int thisPin = 5; thisPin < 12; thisPin++) {
          analogWrite( thisPin , 0 );
        }
    }*/
  //}
}

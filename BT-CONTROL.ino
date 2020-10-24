#include <Servo.h>

char BT;


Servo servo_2;
Servo servo_8;

void setup(){
  Serial.begin(9600);
  servo_2.attach(5);
  servo_8.attach(11);
}

void loop(){
  if (Serial.available() > 0) {
    BT = Serial.read();
    Serial.println(BT);
    if (BT == '0') {
      Serial.println("STOP");
      servo_2.write(90);
      //delay(0);
      servo_8.write(90);
      delay(500);

    }
    if (BT == '1') {
      Serial.println("UP");
      servo_2.write(135);
      //delay(0);
      servo_8.write(45);
      delay(500);

    }
    if (BT == '2') {
      Serial.println("DOWN");
      servo_8.write(135);
      //delay(0);
      servo_2.write(45);
      delay(500);

    }

    if (BT == '3') {
      Serial.println("TOP");
      servo_8.write(20);
      //delay(0);
      servo_2.write(160);
      delay(500);

    }

    if (BT == '4') {
      Serial.println("BOTTOM");
      servo_8.write(160);
      //delay(0);
      servo_2.write(20);
      delay(500);

    }

    if (BT == '5') {
      Serial.println("LEFT");
      servo_8.write(20);
      //delay(0);
      servo_2.write(90);
      delay(500);

    }

    if (BT == '6') {
      Serial.println("RIGHT");
      servo_8.write(90);
      //delay(0);
      servo_2.write(160);
      delay(500);

    }
    if (BT == '7') {
      Serial.println("UP RIGHT");
      servo_8.write(45);
      //delay(0);
      servo_2.write(175);
      delay(500);

    }
    if (BT == '8') {
      Serial.println("UP LEFT");
      servo_8.write(5);
      //delay(0);
      servo_2.write(45);
      delay(500);

    }

    if (BT == '9') {
      Serial.println("SWING");

        servo_8.write(10);
        //delay(0);
        servo_2.write(170);
        delay(500);
      
      for(int i = 0;i <5;i++){
        
        servo_8.write(5);
        servo_2.write(90);
        delay(500);

        servo_8.write(90);
        servo_2.write(175);
        delay(500);

      }
      servo_8.write(10);
        //delay(0);
        servo_2.write(170);
        delay(500);

    }

  }
  if (BT == 'a') {
      Serial.println("test");
      while(1){
        
        servo_2.write(170);
        servo_8.write(10);
        delay(1200);

        servo_8.write(170);
        servo_2.write(10);
        delay(1200);
      }

    }

}

#include <Servo.h>
Servo servo1;
Servo servo2;
const int Y_PIN = A0;  
const int X_PIN = A1;
const int BUTTON_PIN = 8;
const int SERVO1_PIN = 5;
const int SERVO2_PIN = 10;
const int LED_PIN = 13;
float Step = 180.0 / 1024;

boolean isClicked = false;

void setup()
{
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(90);
  servo2.write(90);
}

void loop()
{
  if (digitalRead(BUTTON_PIN))
  {
    if (!isClicked)
    {
      digitalWrite(LED_PIN, HIGH);
      int yVal = analogRead(Y_PIN);
      int xVal = analogRead(X_PIN);
      float yAngle = yVal * Step;
      float xAngle = xVal * Step;
      servo1.write(yAngle);
      servo2.write(xAngle);
    }
  }
  else     
  {
    digitalWrite(LED_PIN, LOW);
    isClicked = !isClicked;
  }
}

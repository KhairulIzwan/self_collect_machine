#include <Wire.h>

#define SLAVE_ADD 0x09 // Set Arduino I2C address

#define BUTTON1 2
#define BUTTON2 3
#define BUTTON3 4

#define LED1 8
#define LED2 9
#define LED3 10

boolean buttonPressed1 = false;
boolean buttonPressed2 = false;
boolean buttonPressed3 = false;

volatile int mode = 0;
int prevMode = 10000;

void setup()
{
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  
  Wire.begin(SLAVE_ADD);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

// Where the Magic Happens
void loop()
{
//  Read
  if (mode != prevMode)
  {
    prevMode = mode;

    if (mode == 0)
    {
      delay(1000);
      digitalWrite(LED1, HIGH);
      delay(1000);
      digitalWrite(LED1, LOW);
      delay(1000);
    }
    else if (mode == 1)
    {
      delay(1000);
      digitalWrite(LED2, HIGH);
      delay(1000);
      digitalWrite(LED2, LOW);
      delay(1000);
    }
    else if (mode == 2)
    {
      delay(1000);
      digitalWrite(LED3, HIGH);
      delay(1000);
      digitalWrite(LED3, LOW);
      delay(1000);
    }
  }
//  Write
  if (digitalRead(BUTTON1) == LOW && buttonPressed1 == false)
  {
    buttonPressed1 = true;
  }
  else if (digitalRead(BUTTON1) == HIGH && buttonPressed1 == true)
  {
    buttonPressed1 = false;
  }
  else if (digitalRead(BUTTON2) == LOW && buttonPressed2 == false)
  {
    buttonPressed2 = true;
  }
  else if (digitalRead(BUTTON2) == HIGH && buttonPressed2 == true)
  {
    buttonPressed2 = false;
  }
  else if (digitalRead(BUTTON3) == LOW && buttonPressed3 == false)
  {
    buttonPressed3 = true;
  }
  else if (digitalRead(BUTTON3) == HIGH && buttonPressed3 == true)
  {
    buttonPressed3 = false;
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  int inData = Wire.read(); // receive byte as an integer
  Serial.println(inData); // print the integer
  mode = inData;
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
  if (buttonPressed1 == true && buttonPressed2 == true && buttonPressed3 == true)
  {
    Wire.write(1);
  }
  else if (buttonPressed1 == true && buttonPressed2 == true && buttonPressed3 == false)
  {
    Wire.write(2);
  }
  else if (buttonPressed1 == true && buttonPressed2 == false && buttonPressed3 == true)
  {
    Wire.write(3);
  }
  else if (buttonPressed1 == true && buttonPressed2 == false && buttonPressed3 == false)
  {
    Wire.write(4);
  }
  else if (buttonPressed1 == false && buttonPressed2 == true && buttonPressed3 == true)
  {
    Wire.write(5);
  }
  else if (buttonPressed1 == false && buttonPressed2 == true && buttonPressed3 == false)
  {
    Wire.write(6);
  }
  else if (buttonPressed1 == false && buttonPressed2 == false && buttonPressed3 == true)
  {
    Wire.write(7);
  }
  else if (buttonPressed1 == false && buttonPressed2 == false && buttonPressed3 == false)
  {
    Wire.write(8);
  }
}

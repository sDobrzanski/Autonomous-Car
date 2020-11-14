#include <Servo.h>

const uint8_t  motorRightPin  = 12;  //kierunek prawo Pin
const uint8_t  motorLeftPin  = 11;  //kierunek lewo Pin
const uint8_t  VelocPin  = 10; //predkosc silnika Pin
const uint8_t servoPin = 9; //servo Pin
const uint8_t triggerPin1 = 7; //czujnik 1 triggerPin
const uint8_t triggerPin2 = 6; //czujnik 2 triggerPin
const uint8_t triggerPin3 = 5; //czujnik 3 triggerPin
const uint8_t echoPin1 = 7; //czujnik 1 echoPin
const uint8_t echoPin2 = 6; //czujnik 2 echoPin
const uint8_t echoPin3 = 5; //czujnik 3 echoPin
Servo servo;  //stworzenie obiektu klasy Servo
int odl1 = 0; //odleglosc przeszkody od 1 czujnika(z lewej)
int odl2 = 0; //odleglosc przeszkody od 2 czujnika(srodek)
int odl3 = 0; //odleglosc przeszkody od 3 czujnika(z prawej)

long odczytOdl1()
{
  pinMode(triggerPin1, OUTPUT); // Clear the trigger
  digitalWrite(triggerPin1, LOW);
  delayMicroseconds(2);
  // Sets the trigger pin to HIGH state for 10 microseconds
  digitalWrite(triggerPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin1, LOW);
  pinMode(echoPin1, INPUT);
  // Reads the echo pin
  //pulseIn(echoPin1, HIGH) is eqaul to sound wave travel time in microseconds
  odl1 = 0.01723 *pulseIn(echoPin1, HIGH); //returns the sound wave travel time in cm
  Serial.println(odl1);
  return odl1; //odleglosc od sciany w cm
}

long odczytOdl2()
{
  pinMode(triggerPin2, OUTPUT);
  digitalWrite(triggerPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin2, LOW);
  pinMode(echoPin2, INPUT);
  odl2 = 0.01723 *pulseIn(echoPin2, HIGH);
  Serial.println(odl2);
  return odl2; //odleglosc od sciany w cm
}

long odczytOdl3()
{
  pinMode(triggerPin3, OUTPUT);
  digitalWrite(triggerPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin3, LOW);
  pinMode(echoPin3, INPUT);
  odl3 = 0.01723 *pulseIn(echoPin3, HIGH);
  Serial.println(odl3);
  return odl3; //odleglosc od sciany w cm
}

void setup(){
    Serial.begin(9600);
    
    pinMode(motorRightPin, OUTPUT);
    pinMode(motorLeftPin, OUTPUT);
   pinMode(VelocPin, OUTPUT);
  
    digitalWrite(motorRightPin, HIGH);
    digitalWrite(motorLeftPin, LOW);
  
    servo.attach(servoPin); //przypisanie Pinu do servo
    servo.write(0); //ustawienie servo w pozycji 0
}


void loop(){
  odczytOdl1(); //odczyt odleglosci z czujnikow
  odczytOdl2();
  odczytOdl3();
  for (int i = 0; i <= 255; i++) 
  {
      analogWrite(VelocPin, i);
      delay(25);
  }
}

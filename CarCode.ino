WERSJA KOŃCOWA
#include <Servo.h>dal
#include <PID_v1.h>

#define motorRightPin  12    //kierunek prawo Pin
#define motorLeftPin     11   //kierunek lewo Pin
#define VelocPin    10  //predkosc silnika Pin
#define servoPin     9    //servo Pin
#define triggerPin1  7    //czujnik 1 triggerPin
#define triggerPin2  6    //czujnik 2 triggerPin
#define triggerPin3  5    //czujnik 3 triggerPin
#define echoPin1    7   //czujnik 1 echoPin
#define echoPin2    6     //czujnik 2 echoPin
#define echoPin3    5     //czujnik 3 echoPin
Servo servo;      //stworzenie obiektu klasy Servo
double Input, Output;
double Setpoint = 50;
const double Kp = 2;
const double Ki = 5;
const double Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
const int a = 0;
const int b =23;    //zmienne dotyczące regulowanego przez PID kąta wychylenia //serwo [PMW]
int predkosc = 0;     //PWM silnika dla predkosci autka
int odl1 = 0;       //odleglosc przeszkody od 1 czujnika(z lewej)
int odl2 = 0;     //odleglosc przeszkody od 2 czujnika(srodek)
int odl3 = 0;     //odleglosc przeszkody od 3 czujnika(z prawej)
bool isFound = false;
float angleVal = 93; //punkt zerowy servo
const int criticalPoint = 10; //odleglosc od sciany w ktorej auto sie zatrzyma
int g = 0;
int wynik = 0;
g = 255/(b-a); // oblicza (zaokrągla do int) ile “stopni” PMW przypada dla jednego stopnia //skrętu serwo


void setup(){
    Serial.begin(9600);
    
    pinMode(motorRightPin, OUTPUT);
    pinMode(motorLeftPin, OUTPUT);
    pinMode(VelocPin, OUTPUT);
  
    digitalWrite(motorRightPin, HIGH);
    digitalWrite(motorLeftPin, LOW);
  
    servo.attach(servoPin); //przypisanie Pinu do servo
    servo.write(angleVal); //ustawienie servo w pozycji poczatkowej

    myPID.SetMode(AUTOMATIC); //turn the PID on
    myPID.SetOutputLimits(a, b); //ustawienie limitu PIDA na a-b stopni zeby kola obracaly sie max o (b-a) stopnie
    carStart(); //funkcja aby auto podjechalo do pierwszej sciany
}


void loop(){
   carStop();
   PIDcontrol();
   Serial.print(angleVal);
   Serial.println(" kąt");
   servo.write(angleVal);
   delay(100); //wykonanie co 0.1 sekundy
}

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
  return odl2;      //odleglosc od sciany w cm
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
  return odl3;      //odleglosc od sciany w cm
}

int obliczenie_predkosci (double output)
// funkcja pobiera założony zakres wychylenia serwo (a,b) oraz wyjscie (Output) z funkcji //PID; im większa regulacja, tym prędkość autka mniejsza
{
  
  wynik = 255 - g*output ;
    if (wynik<30)
{
        wynik = 30;
      }
  return wynik;
  
}

float PIDcontrol()
{
  odczytOdl1(); //odczyt odleglosci z czujnikow
  odczytOdl2();
  odczytOdl3();

  if(odl1<= odl2 && odl1<= odl3)
  {
     Input = odl1;
  }

  if(odl2<= odl1 && odl2<= odl3)
  {
     Input = odl2;
  }

  if(odl3<= odl1 && odl3<= odl1)
  {
     Input = odl3;
  }

myPID.Compute();

  if (Input <= Setpoint)
  {
  predkosc = obliczenie_predkosci(Output);

     analogWrite(VelocPin,predkosc); //zmniejszenie predkosci silnika zależnie od wielkości //regulacji (Output)
  }
  
  
 if (odl1<odl3){
  if(odl1<Setpoint)
{
      angleVal = 93 + Output; //skret w prawo
  }else if(odl1>Setpoint) 
{
    angleVal = 93 - Output; //skret w lewo
  }else 
{
      angleVal = 93; //jazda prosto
    }
}else{
    if(odl3<Setpoint)
{
      angleVal = 93 - Output; //skret w lewo
  }else if(odl3>Setpoint) 
{
    angleVal = 93 + Output; //skret w prawo
  }else 
{
      angleVal = 93; //jazda prosto
 }
}
return angleVal;
}

void carStart(){
    analogWrite(VelocPin, 255);
    while(isFound==false){
      odczytOdl1(); //odczyt odleglosci z czujnikow
      odczytOdl2();
      odczytOdl3();
      if(odl1 <= Setpoint || odl2 <= Setpoint || odl3 <= Setpoint){
      analogWrite(VelocPin, 128);
      isFound=true;
      }
    }
}

void carStop()
{
    while(odl1 <= criticalPoint || odl2 <=criticalPoint  || odl3 <= criticalPoint )
{
    analogWrite(VelocPin, 0);
    Serial.println(“Autko za blisko ściany”);
}
}// Sprawdza czy auto nie znajduje sie zbyt blisko sciany aby wykonac skret

//KONIEC

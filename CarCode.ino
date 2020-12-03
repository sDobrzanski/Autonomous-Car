#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Servo.h>
#include <PID_v1.h>

char auth[] = "blynkId";
char ssid[] = "wifiLogin";
char pass[] = "wifiHaslo";
#define motorRightPin  4   //kierunek prawo Pin
#define motorLeftPin    3   //kierunek lewo Pin
#define VelocPin    10  //predkosc silnika Pin
#define servoPin     9    //servo Pin
#define triggerPin1  7    //czujnik 1 triggerPin
#define triggerPin2  6    //czujnik 2 triggerPin
#define triggerPin3  5    //czujnik 3 triggerPin
#define triggerPin4  8    //czujnik 4 triggerPin
#define echoPin1    7   //czujnik 1 echoPin
#define echoPin2    6     //czujnik 2 echoPin
#define echoPin3    5     //czujnik 3 echoPin
#define echoPin4    8     //czujnik 4 echoPin
#define krancowka1  1
#define krancowka2  2
#define krancowka3  11
Servo servo;      //stworzenie obiektu klasy Servo
double Input, Output;
double Setpoint = 50;
const double Kp = 2;
const double Ki = 5;
const double Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
const int a = 0;
const int b =23;    //zmienne dotyczące regulowanego przez PID kąta wychylenia 
int predkosc = 0;     //PWM silnika dla predkosci autka
int odl1 = 0;     //odleglosc przeszkody od 1 czujnika(z lewej)
int odl2 = 0;     //odleglosc przeszkody od 2 czujnika(srodek)
int odl3 = 0;     //odleglosc przeszkody od 3 czujnika(z prawej)
int odl4 = 0;     //odleglosc przeszkody od 4 czujnika(z tylu)
bool isFound = false;
float angleVal = 93; //punkt zerowy servo
const int criticalPoint = 10; //odleglosc od sciany w ktorej auto sie zatrzyma
int g = 255/(b-a); // oblicza (zaokrągla do int) ile “stopni” PMW przypada dla jednego stopnia //skrętu serwo
int wynik = 0;
int sliderAngle = 0;
int sliderVeloc = 0;
uint8_t trybJazdy;
uint8_t startStop;
uint8_t kierunekJazdy;
uint8_t krancowka1Wart;
uint8_t krancowka2Wart;
uint8_t krancowka3Wart;

BLYNK_WRITE(V4)
{
  sliderAngle  = param.asInt(); //kat obrotu
}

BLYNK_WRITE(V5)
{
  sliderVeloc= param.asInt(); // predkosc jazdy 
}

BLYNK_WRITE(V6)
{
trybJazdy = param.asInt(); // 0 dla manualnego, 1 dla autonomicznego
}
BLYNK_WRITE(V7) 
{
  startStop = param.asInt(); // 0 dla wylaczonego silnika, 1 dla odpalonego
}
BLYNK_WRITE(V8)
{
  kierunekJazdy = param.asInt(); //0 dla jazdy w przod, 1 żeby wrzucić wsteczny
}

BLYNK_READ(V0) 
{
  Blynk.virtualWrite(V0, odczytOdl1()); 
}
BLYNK_READ(V1) 
{
  Blynk.virtualWrite(V1, odczytOdl2());
}
BLYNK_READ(V2)
{
  Blynk.virtualWrite(V2, odczytOdl3());
}

void setup(){
    Serial.begin(9600);
    ESP.eraseConfig();
    Blynk.begin(auth, ssid, pass);

    pinMode(motorRightPin, OUTPUT);
    pinMode(motorLeftPin, OUTPUT);
    pinMode(VelocPin, OUTPUT);
    pinMode(krancowka1, INPUT);
    pinMode(krancowka2, INPUT);
    pinMode(krancowka3, INPUT);

    digitalWrite(motorRightPin, LOW);
    digitalWrite(motorLeftPin, LOW);
  
    servo.attach(servoPin); //przypisanie Pinu do servo
    servo.write(angleVal); //ustawienie servo w pozycji poczatkowej

    myPID.SetMode(AUTOMATIC); //turn the PID on
    myPID.SetOutputLimits(a, b); //ustawienie limitu PIDA na a-b stopni zeby kola obracaly //sie max o (b-a) stopnie
   
}


void loop(){
 krancowka1Wart = digitalRead(krancowka1);
 krancowka2Wart = digitalRead(krancowka2);
 krancowka3Wart = digitalRead(krancowka3);
   
if(startStop  == 1)
{
digitalWrite(motorRightPin, HIGH);
if(trybJazdy  == 1)
{
  if(krancowka1Wart ==HIGH && krancowka2Wart ==HIGH && krancowka3Wart ==HIGH) //przyciski luzne(nieprzycisniete)
  { digitalWrite(motorLeftPin, LOW);
    carStart(); //funkcja aby auto podjechalo do pierwszej sciany
    PIDcontrol();
  }else{
    servo.write(93);
    digitalWrite(motorRightPin, LOW);
    digitalWrite(motorLeftPin, HIGH);
    analogWrite(VelocPin,128);
    while(odczytOdl4() >= criticalPoint || (odczytOdl1() < Setpoint && odczytOdl2() < Setpoint && odczytOdl2() < Setpoint))
    {
      //pusta pętla, auto jedzie do czasu aż odsunie się od ściany albo
      //napotka przeszkode z tylu
    }
  }
}
else //tryb ręczny
{ 
isFound = false;
if(kierunekJazdy  == 0)
  { 
  digitalWrite(motorRightPin, HIGH); 
  digitalWrite(motorLeftPin, LOW); 
  analogWrite(VelocPin, sliderVeloc);
  angleVal = 93 + sliderAngle ; 
  }
  else
  {
  digitalWrite(motorRightPin, LOW); 
  digitalWrite(motorLeftPin, HIGH);
  analogWrite(VelocPin, sliderVeloc);
  angleVal = 93 + sliderAngle ;
  }
}}
else
{
  digitalWrite(motorRightPin, LOW);
  digitalWrite(motorLeftPin, LOW);
  isFound = false;
}
   Serial.print(angleVal);
   Serial.println(" kąt");
   servo.write(angleVal);
   Blynk.run();
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
  odl1 = 0.01723 *pulseIn(echoPin1, HIGH) -2;  //returns the sound wave travel time in cm
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
  odl2 = 0.01723 *pulseIn(echoPin2, HIGH) -7;
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
  odl3 = 0.01723 *pulseIn(echoPin3, HIGH)-2;
  return odl3;      //odleglosc od sciany w cm
}

long odczytOdl4()
{
  pinMode(triggerPin4, OUTPUT);
  digitalWrite(triggerPin4, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin4, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin4, LOW);
  pinMode(echoPin4, INPUT);
  odl4 = 0.01723 *pulseIn(echoPin4, HIGH)-X;// X - do ustalenia po zamontowaniu czujnika
  return odl4;      //odleglosc od sciany w cm
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
      if(odczytOdl1() <= Setpoint || odczytOdl2() <= Setpoint || odczytOdl3() <= Setpoint){
      analogWrite(VelocPin, 128);
      isFound=true;
      }
    }
}

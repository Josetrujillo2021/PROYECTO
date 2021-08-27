//Universidad Del Valle de Guatemala
//BE3015: Electrónica Digital 2
//José Trujillo
//Proyecto #1
//Termometro 
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
//Librerías
//----------------------------------------------------------------------------------------------------------------------

#include <Arduino.h>

//----------------------------------------------------------------------------------------------------------------------
//Definición de pines
//----------------------------------------------------------------------------------------------------------------------

//defino los pines que voy a utilizar del microprocesador para la toma de lectura (B1) y el sensor de temperatura
#define B1 32
#define Sensor 33

//Defino los pines de salida de los leds y el servomotor 
#define Servo 21
#define LV 19
#define LA 18
#define LR 5

//defino los pines de salida del display
#define A 25
#define B 26
#define C 27
#define D 14
#define E 12
#define F 13
#define G 23

//defino los pines para los transitores de activación de los 3 displays
#define T1 4
#define T2 22
#define T3 15

//Definiendo las configuracioens del PWM para el motor y las leds
#define resolucionPWM 8
#define FreqPWM 50

//canales del PWM
#define LVChannel 1
#define LAChannel 2
#define LRChannel 3
#define ServoChannel 4

//Definiendo los nombres de las variables de resolución, largo y rango del ADC
#define resolucionADC 9
#define ancho 9
#define pinRef 35 //sugeto a cambios.


//----------------------------------------------------------------------------------------------------------------------
//Prototipos de funciones
//----------------------------------------------------------------------------------------------------------------------

void MedidorTemperatura(void);
void ConfigurarPWM(void);

//---------------------------------------------------------------------------------------------------------------------
//Variables Globales
//----------------------------------------------------------------------------------------------------------------------
float Temperatura = 0.0; 

//----------------------------------------------------------------------------------------------------------------------
//ISR  (interrupciones)
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
//CONFIGURACIÓN
//----------------------------------------------------------------------------------------------------------------------
void setup() {

  Serial.begin(115200);
  ConfigurarPWM();

  pinMode(B1, INPUT_PULLUP);


  pinMode(Servo, OUTPUT);
  pinMode(LV, OUTPUT);
  pinMode(LA, OUTPUT);
  pinMode(LR, OUTPUT);

  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
  pinMode(E, OUTPUT);
  pinMode(F, OUTPUT);
  pinMode(G, OUTPUT);

  pinMode(T1, OUTPUT);
  pinMode(T2, OUTPUT);
  pinMode(T3, OUTPUT);

  digitalWrite(Servo, LOW);
  digitalWrite(LV, LOW);
  digitalWrite(LA, LOW);
  digitalWrite(LR, LOW);
  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(C, LOW);
  digitalWrite(D, LOW);
  digitalWrite(E, LOW);
  digitalWrite(F, LOW);
  digitalWrite(G, LOW);
  digitalWrite(T1, LOW);
  digitalWrite(T2, LOW);
  digitalWrite(T3, LOW);
}


//---------------------------------------------------------------------------------------------------------------------
//Loop principal
//---------------------------------------------------------------------------------------------------------------------
void loop() {
  MedidorTemperatura();
  Serial.println(Temperatura);
  delay(100);
}

//---------------------------------------------------------------------------------------------------------------------
//Función de ADC de temperatura
//---------------------------------------------------------------------------------------------------------------------

void MedidorTemperatura(void){
  if (digitalRead(B1)==LOW){
    Temperatura = analogRead(Sensor);
    Temperatura = Temperatura/10 //3300/40950 se puede hacer esa operación si quiero dividir aún más mi resolución
    //Pero con la ecuación de Vout=10mv/°C * T ya me sale, solo debo operarlo todo en mV
  }
}

//---------------------------------------------------------------------------------------------------------------------
//Funcion de configuracion de PWM
//---------------------------------------------------------------------------------------------------------------------
void ConfigurarPWM(void){

  //En esta función voy a setear las configuraciones del pwm para el servo y las leds
  ledcSetup(ServoChannel, FreqPWM, resolucionPWM);
  ledcAttachPin(Servo, ServoChannel);

  //Led Verde
  ledcSetup(LVChannel, FreqPWM, resolucionPWM);
  ledcAttachPin(LV, LVChannel);

  //Led Amarilla
  ledcSetup(LAChannel, FreqPWM, resolucionPWM);
  ledcAttachPin(LA, LAChannel);

  //Led Roja 
  ledcSetup(LRChannel, FreqPWM, resolucionPWM);
  ledcAttachPin(LR, LRChannel);
}
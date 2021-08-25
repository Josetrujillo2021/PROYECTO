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






//----------------------------------------------------------------------------------------------------------------------
//Prototipos de funciones
//----------------------------------------------------------------------------------------------------------------------

void ConfigurarPWM(void);

//---------------------------------------------------------------------------------------------------------------------
//Variables Globales
//----------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------
//ISR  (interrupciones)
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
//CONFIGURACIÓN
//----------------------------------------------------------------------------------------------------------------------
void setup() {

  ConfigurarPWM();
}


//---------------------------------------------------------------------------------------------------------------------
//Loop principal
//---------------------------------------------------------------------------------------------------------------------
void loop() {
  // put your main code here, to run repeatedly:
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
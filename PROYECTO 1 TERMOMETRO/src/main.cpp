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
#include "esp_adc_cal.h"

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
#define f 23
#define G 13

//defino los pines para los transitores de activación de los 3 displays
#define T1 4
#define T2 15
#define T3 22

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

//Temperaturas maximas y minimas
#define TempMin 21.0
#define TempMax 22.5
#define Cambio 0.7 //Este me permite hacer el cambio de cantidad que le voy a sumar al PWM del servo, 0.7 para 18 a 19.5 °C y 2.3 para 37 a 37.5°C

//definición de prescaler para timer
#define prescaler 1000000


//----------------------------------------------------------------------------------------------------------------------
//Prototipos de funciones
//----------------------------------------------------------------------------------------------------------------------

void MedidorTemperatura(void);
void ConfigurarPWM(void);
void IndicadorTemperatura(void);
void MovimientoServo(void);
void Displays(int valor);


//---------------------------------------------------------------------------------------------------------------------
//Variables Globales
//----------------------------------------------------------------------------------------------------------------------
float Temperatura = 0.0; 
float DutycicleS = 0; 
int decenas = 0; 
int unidades = 0;
int decimales = 0;
int valor = 0; 

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
  pinMode(f, OUTPUT);
  pinMode(G, OUTPUT);

  pinMode(T1, OUTPUT);
  pinMode(T2, OUTPUT);
  pinMode(T3, OUTPUT);

  digitalWrite(Servo, LOW);
  digitalWrite(LV, LOW);
  digitalWrite(LA, LOW);
  digitalWrite(LR, LOW);
  digitalWrite(A, HIGH);
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH);
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(f, HIGH);
  digitalWrite(G, HIGH);
  digitalWrite(T1, LOW);
  digitalWrite(T2, LOW);
  digitalWrite(T3, LOW);

}


//---------------------------------------------------------------------------------------------------------------------
//Loop principal
//---------------------------------------------------------------------------------------------------------------------
void loop() {
  
  Displays(valor);
  
  //la siguiente secuencia me permite mostra la temperatura en los displays sin números fantasma
  digitalWrite(T1, HIGH);
  digitalWrite(T2, LOW);
  digitalWrite(T3, LOW);
  Displays(decenas);
  delay(5);

  digitalWrite(T1, LOW);
  digitalWrite(T2, HIGH);
  digitalWrite(T3, LOW);
  Displays(unidades);
  delay(5);

  digitalWrite(T1, LOW);
  digitalWrite(T2, LOW);
  digitalWrite(T3, HIGH); 
  Displays(decimales);
  delay(5);
  
  MedidorTemperatura();
  IndicadorTemperatura();
  MovimientoServo();
  
  Serial.println(Temperatura);
  Serial.println(DutycicleS);
  Serial.print(decenas);
  Serial.print(unidades);
  Serial.print('.');
  Serial.println(decimales); 
  delay(1);
}

//---------------------------------------------------------------------------------------------------------------------
//Función de ADC de temperatura
//---------------------------------------------------------------------------------------------------------------------

void MedidorTemperatura(void){
  if (digitalRead(B1)==LOW){
    Temperatura = analogReadMilliVolts(Sensor); //me permite asignarle el valor analogico del sensor LM35
    Temperatura = Temperatura/10; //3300/40950 se puede hacer esa operación si quiero dividir aún más mi resolución
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
//---------------------------------------------------------------------------------------------------------------------
//Funcion de indicador de la temperatura de las leds
//---------------------------------------------------------------------------------------------------------------------
void IndicadorTemperatura(void){
  //cada if me permite encender que led mostrar dependiendo del valor de la temperatura mínima y máxima
  if (Temperatura<TempMin){
    ledcWrite(LVChannel, 255);
    ledcWrite(LAChannel, 0);
    ledcWrite(LRChannel, 0);
  }

  else if(Temperatura>=TempMin && Temperatura<TempMax){
    ledcWrite(LVChannel, 0);
    ledcWrite(LAChannel, 255);
    ledcWrite(LRChannel, 0);
  }

  else if(Temperatura>=TempMax){
    ledcWrite(LVChannel, 0);
    ledcWrite(LAChannel, 0);
    ledcWrite(LRChannel, 255);
  }
}
//---------------------------------------------------------------------------------------------------------------------
//Funcion de Movimiento Servomotor
//---------------------------------------------------------------------------------------------------------------------
void MovimientoServo(void){

  
  /*if (digitalRead(B1)==HIGH){
    estado = Temperatura;  
  }
  else {
    estado = estado; 
  }
  if (Temperatura > estado && DutycicleS <35 && digitalRead(B1)==LOW){
    DutycicleS = DutycicleS + Cambio; //0.7 para 18 a 19.5 °C y 2.3 para 37 a 37.5°C
    delay(100);
    ledcWrite(ServoChannel, DutycicleS);
  }

  else if (Temperatura < estado && DutycicleS >0 && digitalRead(B1)==LOW) {
   DutycicleS = DutycicleS - Cambio; //0.7 para 18 a 19.5 °C y 2.3 para 37 a 37.5°C
    delay(100);
    ledcWrite(ServoChannel, DutycicleS);
  }*/

  if (Temperatura <= 21.0){
    DutycicleS = 8.8;//es igual a 30°
    delay(5);
    ledcWrite(ServoChannel, DutycicleS);
  }

  else if (Temperatura > 21.0 &&Temperatura <= 22.5){
    DutycicleS = 17.5; //es igual a 90°
    delay(5);
    ledcWrite(ServoChannel, DutycicleS);
  } 

  else if (Temperatura > 22.5){
    DutycicleS = 26.3; //es igual a 135°
    delay(5);
    ledcWrite(ServoChannel, DutycicleS);

  }
}
//---------------------------------------------------------------------------------------------------------------------
//Funcion de Display Temperatura  
//---------------------------------------------------------------------------------------------------------------------
void Displays(int valor){
  decenas = Temperatura/10;
  unidades = Temperatura - decenas*10;
  decimales = (Temperatura*10) - (decenas*100) - (unidades*10); 

  //El valor va a ser la variable de decenas, unidades o decimales que le toque mostrar dependiendo del tiempo del temp
  //temporizador. 
  switch (valor)
  {
  case 0:
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW); 
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(f, LOW);
    digitalWrite(G, HIGH);
    break;
  case 1: 
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW); 
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(f, HIGH);
    digitalWrite(G, HIGH);
    break;
  case 2:
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, HIGH); 
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(f, HIGH);
    digitalWrite(G, LOW);
    break; 
  case 3: 
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW); 
    digitalWrite(D, LOW);
    digitalWrite(E, HIGH);
    digitalWrite(f, HIGH);
    digitalWrite(G, LOW);
    break;

  case 4: 
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW); 
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(f, LOW);
    digitalWrite(G, LOW);
    break;  
  case 5: 
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
    digitalWrite(C, LOW); 
    digitalWrite(D, LOW);
    digitalWrite(E, HIGH);
    digitalWrite(f, LOW);
    digitalWrite(G, LOW);
    break;
  case 6: 
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
    digitalWrite(C, LOW); 
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(f, LOW);
    digitalWrite(G, LOW);
    break;
  case 7: 
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW); 
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(f, HIGH);
    digitalWrite(G, HIGH);
    break;
  case 8: 
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW); 
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(f, LOW);
    digitalWrite(G, LOW);
    break;
  case 9: 
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW); 
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(f, LOW);
    digitalWrite(G, LOW);
    break;
  }
}


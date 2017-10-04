
#include "Motor_brazo.h"
#include "Encoder.h"

//Pines con interrupción en arduino mega 2, 3, 18, 19, 20, 21


#define PIN1ENCODER1 2 //con interrupción
#define PIN2ENCODER1 4
#define PULSOS_REV_ENCODER1 4320 //16*270

//Pines motor brazo
#define R_EN 5
#define L_EN 6
#define R_PWM 7 
#define L_PWM 8

#define KP 5 //Constante P para el control 

float pos_objetivo = 0; //posicion en grados positivo o negativo
long int cuenta_objetivo; //posicion segun los pulsos del encoder por vuelta

Motor_brazoClass motor1(R_EN, L_EN, R_PWM, L_PWM);
EncoderClass encoder1(PIN1ENCODER1, PIN2ENCODER1); //El pin 1 necesita interrupción
volatile long int cuenta1 = 0; //Es volatile para que las interrupciones puedan cambiarlo sin problemas

void setup()
{
	//Se puede cambiar por CHANGE para aumentar la resolucion si es necesario
	attachInterrupt(digitalPinToInterrupt(PIN1ENCODER1), comprobar_encoder1, RISING); 
}

void loop()
{

}

void serialEvent() {

}
void comprobar_encoder1() {
	cuenta1 = encoder1.actualizar_cuenta();
}

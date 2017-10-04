
#include "Encoder.h"

EncoderClass encoder1(2, 4); //pines 2 (con interrupción) y 4
volatile long int cuenta1 = 0; //Es volatile para que las interrupciones puedan cambiarlo sin problemas

void setup()
{
	//Se puede cambiar por CHANGE para aumentar la resolucion si es necesario
	attachInterrupt(digitalPinToInterrupt(2), comprobar_encoder1, RISING); 
}

void loop()
{



}

void comprobar_encoder1() {
	cuenta1 = encoder1.actualizar_cuenta();
}

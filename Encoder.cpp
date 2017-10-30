// 
// 
// 

#include "Encoder.h"


void EncoderClass::inicializar(int pin1, int pin2)
{
	pin_a = pin1;
	pin_b = pin2;
	cuenta = 0;

	pinMode(pin_a, INPUT_PULLUP);
	pinMode(pin_b, INPUT_PULLUP);
}

long int EncoderClass::actualizar_cuenta()
{
	if (digitalRead(pin_a)) digitalRead(pin_b) ? cuenta++ : cuenta--;

	//Esto solo afecta si la interrupción se configura como CHANGE
	else digitalRead(pin_b) ? cuenta-- : cuenta++;

	return cuenta;
}

void EncoderClass::setCuenta(long int aux)
{
	cuenta = aux;
}

long int EncoderClass::getCuenta()
{
	return cuenta;
}










// 
// 
// 

#include "Motor_base.h"

void Motor_baseClass::inicializar(int In1,int In2, int Ena) {
	in1 = In1;
	in2 = In2;
	ena = Ena;
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(ena, OUTPUT);
}
/*
void Motor_baseClass::ajustar_velocidad(char direccion, int velocidad)
{

	if (direccion == 'r') {
		digitalWrite(in1, HIGH);
		digitalWrite(in2, LOW);
		analogWrite(ena, velocidad);
	}

	else if (direccion == 'l') {
		digitalWrite(in1, LOW);
		digitalWrite(in2, HIGH);
		analogWrite(ena, velocidad);
	}
}
*/
void Motor_baseClass::velocidad(int velocidad)
{
	if (velocidad > 0) {
	//	ajustar_velocidad('r', velocidad);
		digitalWrite(in1, HIGH);
		digitalWrite(in2, LOW);
		analogWrite(ena, velocidad);
	}

	else if (velocidad < 0) {
	//	ajustar_velocidad('l', -velocidad);
		digitalWrite(in1, LOW);
		digitalWrite(in2, HIGH);
		analogWrite(ena, -velocidad);
	}

	else {
		digitalWrite(in1, LOW);
		digitalWrite(in2, LOW);
		analogWrite(ena, velocidad);
	}
}





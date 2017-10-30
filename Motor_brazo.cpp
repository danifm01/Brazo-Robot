// 
// 
// 

#include "Motor_brazo.h"
#include "PID.h"

void Motor_brazoClass::inicializar(int rEN, int lEN, int rPWM, int lPWM)
{
	r_en = rEN;
	l_en = lEN;
	l_pwm = rPWM;
	r_pwm = lPWM;
	pinMode(r_en, OUTPUT);
	pinMode(l_en, OUTPUT);
	digitalWrite(r_en, HIGH);
	digitalWrite(l_en, HIGH);
	pinMode(l_pwm, OUTPUT);
	pinMode(r_pwm, OUTPUT);
}

void Motor_brazoClass::ajustar_velocidad(char direccion, int velocidad)
{
	digitalWrite(l_en, HIGH);
	digitalWrite(r_en, HIGH);
	if (direccion == 'r') {
		analogWrite(r_pwm, 0);
		analogWrite(l_pwm, ((int)velocidad));
	}

	else if (direccion == 'l') {
		analogWrite(l_pwm, 1);
		analogWrite(r_pwm, ((int)velocidad));
	}

}

void Motor_brazoClass::velocidad(int velocidad)
{
	if (velocidad > 0) {
		ajustar_velocidad('r', velocidad);
	}

	else if (velocidad < 0) {
		ajustar_velocidad('l', -velocidad);
	}
	else {
		analogWrite(l_pwm, 0);
		analogWrite(r_pwm, 0);
	}
}




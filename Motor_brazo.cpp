// 
// 
// 

#include "Motor_brazo.h"

Motor_brazoClass::Motor_brazoClass(int rEN, int lEN, int rPWM, int lPWM)
{
	r_en = rEN;
	l_en = lEN;
	r_pwm = rPWM;
	l_pwm = lPWM;

	pinMode(r_en, OUTPUT);
	pinMode(l_en, OUTPUT);
	digitalWrite(r_en, false);
	digitalWrite(l_en, false);
	pinMode(r_pwm, OUTPUT);
	pinMode(l_pwm, OUTPUT);
}

void Motor_brazoClass::ajustar_velocidad(char direccion, int velocidad)
{
	if (direccion == 'r') {
		digitalWrite(l_en, false);
		digitalWrite(r_en, true);
		digitalWrite(r_pwm, velocidad);
	}

	else if (direccion == 'l') {
		digitalWrite(r_en, false);
		digitalWrite(l_en, true);
		digitalWrite(l_pwm, velocidad);
	}
}


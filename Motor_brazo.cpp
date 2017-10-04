// 
// 
// 

#include "Motor_brazo.h"

#define KP 5 //Constante P para el control 

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

void Motor_brazoClass::posicion_pulsos(float pulsos_obj, float pulsos_act)
{
	float error = pulsos_obj - pulsos_act;
	if (error >= 0) {
		ajustar_velocidad('r', map(error, 0, 4320, 0, 255) * KP);
	}
	else {
		ajustar_velocidad('l', map(error, -4320, 0, 255, 0) * KP);
	}
}




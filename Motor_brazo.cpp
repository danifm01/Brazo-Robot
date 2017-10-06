// 
// 
// 

#include "Motor_brazo.h"

#define KP 50 //Constante P para el control 


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
	digitalWrite(l_en, true);
	digitalWrite(r_en, true);
	if (direccion == 'r') {
		analogWrite(l_pwm, 0);
		analogWrite(r_pwm, velocidad);
	}

	else if (direccion == 'l') {
		analogWrite(r_pwm, 0);
		analogWrite(l_pwm, velocidad);
	}
}

void Motor_brazoClass::posicion_pulsos(float pulsos_obj, float pulsos_act)
{
	float error = pulsos_obj - pulsos_act;
	if (error > 50) {
		ajustar_velocidad('r', 255);
	//	ajustar_velocidad('r', map(error, 0, 4320, 0, 255) * KP);
	}
	else if (error < -50) {
		ajustar_velocidad('l', 255);
	//	ajustar_velocidad('l', map(error, -4320, 0, 255, 0) * KP);
	}
	else {
		ajustar_velocidad('r', 0);
		ajustar_velocidad('l', 0);
	}
	Serial.println(error);
}




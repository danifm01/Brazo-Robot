// 
// 
// 

#include "Motor_brazo.h"
#include "PID.h"


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

void Motor_brazoClass::posicion_pulsos(int velocidad)
{
/*	
float error = pulsos_obj - pulsos_act;
	//asumimos un error de 10 pulsos, menos de un grado
	if (error > 10) {
		ajustar_velocidad('r', 255);
	//	ajustar_velocidad('r', map(error, 0, 4320, 0, 255) * KP);
	}
	else if (error < -10) {
		ajustar_velocidad('l', 255);
	//	ajustar_velocidad('l', map(error, -4320, 0, 255, 0) * KP);
	}
	else {
		ajustar_velocidad('r', 0);
		ajustar_velocidad('l', 0);
	}
//	Serial.println(error);

*/

	if (velocidad > 0) {
		ajustar_velocidad('r', velocidad);
	}

	if (velocidad < 0) {
		ajustar_velocidad('l', velocidad);
	}


}




// Motor_brazo.h

#ifndef _MOTOR_BRAZO_h
#define _MOTOR_BRAZO_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class Motor_brazoClass
{
 protected:
	 int r_en;
	 int l_en;
	 int r_pwm;
	 int l_pwm;

 public:
	 Motor_brazoClass(int,int,int,int);
	 void ajustar_velocidad(char, int);
	 void posicion_pulsos(float pulsos_obj, float pulsos_act);
};

#endif


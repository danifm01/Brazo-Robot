// Motor_base.h

#ifndef _MOTOR_BASE_h
#define _MOTOR_BASE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class Motor_baseClass
{
 protected:
	 int in1;
	 int in2;
	 int ena;

 public:
	 void inicializar(int In1, int In2, int Ena);
//	 void ajustar_velocidad(char direccion, int velocidad);
	 void velocidad(int velocidad);
};


#endif


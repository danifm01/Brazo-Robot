// PID.h

#ifndef _PID_h
#define _PID_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class PIDClass
{
 protected:
	 float kp;
	 float ki;
	 float kd;
	 int cuenta_anterior;
	 int suma_salida;
	 int salidaMax;
	 int salidaMin;
	 int zona_muerta;
	 unsigned long int anterior;
	 unsigned int tiempo_pid;
	 double salida;

 public:
	 void inicializar(float kp, float kd, float ki, float tiempo_calculo, float zona_muerta);
	 int calcular(int cuenta_actual, int cuenta_objetivo);
	 void setConstantes(float tempkp, float tempki, float tempkd, float tiempo);
};


#endif


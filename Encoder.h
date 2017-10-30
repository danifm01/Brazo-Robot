// Encoder.h

#ifndef _ENCODER_h
#define _ENCODER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class EncoderClass
{
 protected:
	 long int cuenta;
	 int pin_a;
	 int pin_b;
 public:
	void inicializar (int,int);
	long int actualizar_cuenta();
	void setCuenta(long int);
	long int getCuenta();
};

#endif


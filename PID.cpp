// 
// 
// 

#include "PID.h"

void PIDClass::inicializar(float tempkp, float tempki, float tempkd, float tiempo_calculo, float tempzona_muerta) {
	setConstantes(tempkp, tempki, tempkd, tiempo_calculo);
	tiempo_pid = tiempo_calculo;
	anterior = 0;
	cuenta_anterior = 0;
	salida = 0;
	suma_salida = 0;
	salidaMax = 255;
	salidaMin = -255;
	zona_muerta = tempzona_muerta;
}

//se debe llamar regularmente
int PIDClass::calcular(int cuenta_actual, int cuenta_objetivo) {

	unsigned long ahora = millis();
	unsigned long tiempo_cambio = (ahora - anterior);
	if (tiempo_cambio >= tiempo_pid)
	{
		int error = cuenta_objetivo - cuenta_actual;
		int dentrada = (cuenta_actual - cuenta_anterior);
		suma_salida += (ki * error);


		if (suma_salida > salidaMax) suma_salida = salidaMax;
		else if (suma_salida < salidaMin) suma_salida = salidaMin;

		

		salida = kp * error + suma_salida - kd * dentrada;
	
		if (salida > salidaMax) salida = salidaMax;
		else if (salida < salidaMin) salida = salidaMin;

		if (salida > 0 && salida < zona_muerta) salida = zona_muerta;
		if (salida < 0 && salida > -zona_muerta) salida = -zona_muerta;
		cuenta_anterior = cuenta_actual;
		
	}


	return (int) salida;
}

void PIDClass::setConstantes(float tempkp, float tempki, float tempkd, float tiempo) {

	double tiempo_segundos = ((double)tiempo) / 1000;
	kp = tempkp;

	//permite cambiar la frecuencia con la que se calcula el pid sin variar el peso de las constantes
	ki = tempki * tiempo_segundos;
	kd = tempkd / tiempo_segundos;

}


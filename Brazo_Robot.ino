
#include "PID.h"
#include "Motor_brazo.h"
#include "Encoder.h"
#include "TimerOne.h"

//Pines con interrupción en arduino mega 2, 3, 18, 19, 20, 21


#define PIN1ENCODER1 2 //con interrupción
#define PIN2ENCODER1 4
#define PULSOS_REV_ENCODER1 4320 // 16*270
#define GRADOS_A_PULSOS1 12 // 4320/360 = 12

//Pines motor brazo
#define R_EN 5
#define L_EN 6
#define R_PWM 7 
#define L_PWM 8

//Constantes para el control pid
#define KP1 1
#define KI1 1
#define KD1 1
#define TIEMPO_PID 50 //cada cuanto tiempo se calcula el pid en ms
#define ZONA_MUERTA1 0 //zona muerta del motor 1 (en pwm 0-255)
#define ZONA_MUERTA2 0 //zona muerta del motor 2 (en pwm 0-255)

int operacion;
float datos[5];

int pos_objetivo = 0; //posicion en grados positivo o negativo
long int cuenta_objetivo = 0; //posicion segun los pulsos del encoder por vuelta


EncoderClass encoder1(PIN1ENCODER1, PIN2ENCODER1); //El pin 1 necesita interrupción
volatile long int cuenta1 = 0; //Es volatile para que las interrupciones puedan cambiarlo sin problemas

Motor_brazoClass motor1(R_EN, L_EN, R_PWM, L_PWM);

PIDClass pid1(KP1, KI1, KD1, TIEMPO_PID, ZONA_MUERTA1);
volatile int velocidad1 = 0;

void setup()
{
	//Se puede cambiar por CHANGE para aumentar la resolucion si es necesario
	attachInterrupt(digitalPinToInterrupt(PIN1ENCODER1), comprobar_encoder1, RISING); 

	//Creamos una interrupcion temporal para calcular el pid de forma regular
	Timer1.initialize(TIEMPO_PID); 
	Timer1.attachInterrupt(calcular_pid); //no usar pines 11 y 12 para pwm
	Timer1.detachInterrupt();
	Serial.begin(9600);
	Serial.println("Serial encendido a 9600 baudios");
}

void loop()
{
		switch ((int)datos[0])
		{
		case 1:
			break;
		case 2:
			if (datos[1] == 1) {
				//Hacer coordenadas cartesianas
			}

			if (datos[1] == 2) {
				cuenta_objetivo = (int)(datos[2] * GRADOS_A_PULSOS1);
				Timer1.attachInterrupt(calcular_pid);
				motor1.posicion_pulsos(velocidad1); 
			}

			break;
		case 3:
			break;
		default:
			break;
		}



}

void serialEvent() {
	int i = 0;

	for (i = 0; i < 5; i++) {
		datos[i] = 0;
	}

	i = 0;
	while (Serial.read() == '#') {	
		datos[i] = Serial.parseInt();
		i++;
	}

/*
	//Para comprobar
	for (i = 0; i < 5; i++) {
		Serial.print("Datos ");
		Serial.print(i);
		Serial.print(": ");
		Serial.println(datos[i]);

	}
*/

}
void comprobar_encoder1() {
	cuenta1 = encoder1.actualizar_cuenta();
}

void calcular_pid() {
	velocidad1 = pid1.calcular(cuenta1,cuenta_objetivo);
}


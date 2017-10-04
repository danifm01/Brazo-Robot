
#include "Motor_brazo.h"
#include "Encoder.h"

//Pines con interrupción en arduino mega 2, 3, 18, 19, 20, 21


#define PIN1ENCODER1 2 //con interrupción
#define PIN2ENCODER1 4
#define PULSOS_REV_ENCODER1 4320 //16*270

//Pines motor brazo
#define R_EN 5
#define L_EN 6
#define R_PWM 7 
#define L_PWM 8

int operacion;
float datos[5];

byte cambio_ope;
float pos_objetivo = 0; //posicion en grados positivo o negativo
long int cuenta_objetivo; //posicion segun los pulsos del encoder por vuelta

Motor_brazoClass motor1(R_EN, L_EN, R_PWM, L_PWM);
EncoderClass encoder1(PIN1ENCODER1, PIN2ENCODER1); //El pin 1 necesita interrupción
volatile long int cuenta1 = 0; //Es volatile para que las interrupciones puedan cambiarlo sin problemas

void setup()
{
	//Se puede cambiar por CHANGE para aumentar la resolucion si es necesario
	attachInterrupt(digitalPinToInterrupt(PIN1ENCODER1), comprobar_encoder1, RISING); 
	Serial.begin(9600);
	Serial.println("Serial encendido a 9600 baudios");
}

void loop()
{
		switch (operacion)
		{
		case 1:
			break;
		case 2:
			if (datos[1] == 1) {
				//Hacer coordenadas cartesianas
			}

			if (datos[1] == 2) {
				motor1.posicion_pulsos(datos[2] * 12, cuenta1);
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
		cambio_ope = 1;
	}

/*	//Para comprobar
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


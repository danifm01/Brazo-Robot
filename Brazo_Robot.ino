
#include "Motor_base.h"
#include "PID.h"
#include "Motor_brazo.h"
#include "Encoder.h"
#include <Servo.h>

//Pines con interrupción en arduino mega 2, 3, 18, 19, 20, 21

//Encoder 1
#define PIN1ENCODER1 2 //con interrupción
#define PIN2ENCODER1 4
#define PULSOS_REV_ENCODER1 4320 // 16*270
#define GRADOS_A_PULSOS1 12 // 4320/360 = 12
#define PULSOS_A_GRADOS1 0.08333 // 360/4320
//Encoder 0
#define PIN1ENCODER0 18 //con interrupción
#define PIN2ENCODER0 19
#define PULSOS_REV_ENCODER0 1700 //
#define GRADOS_A_PULSOS0 4.72 //1700/360
#define PULSOS_A_GRADOS0 0.21765 //360/1700

//Pines motor brazo (motor 1)
#define R_EN 30
#define L_EN 32
#define R_PWM 13 
#define L_PWM 10

//pines motor base (motor 0)
#define IN1 3
#define IN2 5 
#define ENA 11

//pin servo motor (motor 2)
#define PIN_SERVO 12

//Constantes para el control pid
#define KP1 1
#define KI1 0.002
#define KD1 1.5
#define KP0 10
#define KI0 0.01
#define KD0 6
#define TIEMPO_PID 100 //cada cuanto tiempo se calcula el pid en ms
#define ZONA_MUERTA1 12 //zona muerta del motor 1 (en pwm 0-255)
#define ZONA_MUERTA0 80 //zona muerta del motor 0 (en pwm 0-255)

//Pines finales de carrera
#define PIN_FINAL_CARRERA0 26
#define PIN_FINAL_CARRERA1 52
int operacion = 0;
float datos[5] = { 0,0,0,0,0 }; //guarda los datos de la lecura del serial

int pos_objetivo = 0; //posicion en grados positivo o negativo
long int cuenta_objetivo1 = 0; //posicion segun los pulsos del encoder por vuelta
long int cuenta_objetivo0 = 0;

EncoderClass encoder1; //El pin 1 necesita interrupción
EncoderClass encoder0;

Motor_brazoClass motor1;
Motor_baseClass motor0;
Servo servo;

PIDClass pid1;
PIDClass pid0;

volatile long int cuenta1 = 0; //Es volatile para que las interrupciones puedan cambiarlo sin problemas
volatile long int cuenta0 = 0;

int velocidad1 = 0;
int velocidad0 = 0;

//Banderas para hacer varias cosas a la vez
bool mov_motor_pos = false;
bool calibrado = false;

//Posiciones objetivo de los motores en pulsos
float pos_obj_grad0 = 0, pos_obj_grad1 = 0, pos_obj_grad2 = 0;



/////////////////////////////////////////////////////////////////////////////////////////
/*****************************************Setup*****************************************/
/////////////////////////////////////////////////////////////////////////////////////////

void setup()
{

	//Se puede cambiar por CHANGE para aumentar la resolucion si es necesario
	attachInterrupt(digitalPinToInterrupt(PIN1ENCODER1), comprobar_encoder1, RISING); 
	attachInterrupt(digitalPinToInterrupt(PIN1ENCODER0), comprobar_encoder0, RISING);
	pinMode(PIN_FINAL_CARRERA0, INPUT_PULLUP);
	pinMode(PIN_FINAL_CARRERA1, INPUT_PULLUP);

	servo.attach(PIN_SERVO);
	encoder1.inicializar(PIN1ENCODER1, PIN2ENCODER1);
	encoder0.inicializar(PIN1ENCODER0, PIN2ENCODER0);
	motor1.inicializar(R_EN, L_EN, R_PWM, L_PWM);
	motor0.inicializar(IN1, IN2, ENA);
	pid1.inicializar(KP1, KI1, KD1, TIEMPO_PID, ZONA_MUERTA1);
	pid0.inicializar(KP0, KI0, KD0, TIEMPO_PID, ZONA_MUERTA0);
	Serial.begin(9600);
	Serial.println("Serial encendido a 9600 baudios");
	
}


//////////////////////////////////////////////////////////////////////////////////////////
/******************************************Loop******************************************/
//////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
		switch ((int)datos[0])
		{
		case 1:
			//Parada de emergencia
			mov_motor_pos = false;
			parada_motores();
			break;
		case 2: 
			mov_motor_pos = true;
			pos_obj_grad0 = datos[2];
			pos_obj_grad1 = datos[3];
			pos_obj_grad2 = datos[4];
			break;
		case 3: 
			mostrar_posicion();
			break;
		
		case 4:
			mov_motor_pos = false;
			mover_motores_velocidad();
			break;
			
		case 5://calibrar
			if (!calibrado){
				calibrar();
			}
			break;
		default:
			break; 
		}

		if (mov_motor_pos) {
			mover_motores_posicion();
	} 
		Serial.println(cuenta1);
}

//////////////////////////////////////////////////////////////////////////////////////////
/**************************************serialEvent***************************************/
//////////////////////////////////////////////////////////////////////////////////////////


void serialEvent() {
	int i = 0;
//	Serial.println("¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡");
	for (i = 0; i < 5; i++) {
		datos[i] = 0;
	}

	i = 0;
	while (Serial.read() == '#') {	
		datos[i] = Serial.parseInt();
		i++;
	}
}


//////////////////////////////////////////////////////////////////////////////////////////
/************************************Funciones Switch************************************/
//////////////////////////////////////////////////////////////////////////////////////////

void mover_motores_posicion() {
	if (datos[1] == 1) {
		//Hacer coordenadas cartesianas
	}

	if (datos[1] == 2) {
		cuenta_objetivo1 = (int)(pos_obj_grad1 * GRADOS_A_PULSOS1);
		cuenta_objetivo0 = (int)(pos_obj_grad0 * GRADOS_A_PULSOS0);
		calcular_pid();
		motor1.velocidad(velocidad1);
		motor0.velocidad(velocidad0);
		servo.write(pos_obj_grad2);

	//	Serial.println("ha entrado");
	//	Serial.println(velocidad1);
	}
}

void mostrar_posicion() {
		Serial.print('#');
		Serial.print('3');
		Serial.print('#');
		Serial.print('2');
		Serial.print('#');
		Serial.print(cuenta0 * PULSOS_A_GRADOS0);
		Serial.print('#');
		Serial.print(cuenta1 * PULSOS_A_GRADOS1);
		Serial.print('#');
		Serial.println(servo.read()); 
}

void mover_motores_velocidad() {
	if (datos[1] == 2) {
		motor1.velocidad((int)datos[3]);
		motor0.velocidad((int)datos[2]);

		//Control todo o nada del servo
		if (datos[4] > 0) servo.write(servo.read() + 1);
		else if (datos[4] < 0) servo.write(servo.read() - 1);

		Serial.println(datos[3]);
	}
}

void parada_motores() {
	motor0.velocidad(0);
	motor1.velocidad(0);
}

void calibrar() {
	while (digitalRead(PIN_FINAL_CARRERA0)) {
		motor0.velocidad(-100);
	
	}	
	cuenta0 = 0;
	motor0.velocidad(0);
	while (digitalRead(PIN_FINAL_CARRERA1)) {
		motor1.velocidad(-100);
		Serial.println("Calibrando");
	}
	Serial.println("Hecho");
	cuenta1 = 0;
	motor1.velocidad(0);
	calibrado = true;
}

void calcular_pid() {
	velocidad1 = pid1.calcular(cuenta1, cuenta_objetivo1);
	velocidad0 = pid0.calcular(cuenta0, cuenta_objetivo0);
}


//////////////////////////////////////////////////////////////////////////////////////////
/******************************Funciones con interrupciones******************************/
//////////////////////////////////////////////////////////////////////////////////////////

void comprobar_encoder1() {
	cuenta1 = encoder1.actualizar_cuenta();
}

void comprobar_encoder0() {
	cuenta0 = encoder0.actualizar_cuenta();
}




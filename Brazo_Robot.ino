
#include "Motor_base.h"
#include "PID.h"
#include "Motor_brazo.h"
#include "Encoder.h"
#include <math.h>

//Geometria robot
#define L2 25
#define L3 18

//Pines con interrupci�n en arduino mega 2, 3, 18, 19, 20, 21

//Encoder 1
#define PIN1ENCODER1 2 //con interrupci�n
#define PIN2ENCODER1 4
#define PULSOS_REV_ENCODER1 4320 // 16*270
#define GRADOS_A_PULSOS1 12 // 4320/360 = 12
#define PULSOS_A_GRADOS1 0.08333 // 360/4320
//Encoder 0
#define PIN1ENCODER0 18 //con interrupci�n
#define PIN2ENCODER0 19
#define PULSOS_REV_ENCODER0 1700 //
#define GRADOS_A_PULSOS0 4.72 //1700/360
#define PULSOS_A_GRADOS0 0.21765 //360/1700
//Potenciometro
/*
#define GRADOS_A_PULSOSSERVO(x) (((x) * (-40) / 9) + 600) //0 grados son 600 y 90 grados son 200 en valor del potenciometro
#define PULSOS_A_GRADOSSERVO(x) ((-0.225 * (x)) + 135) //inverso
*/

#define GRADOS_A_PULSOSSERVO(x) (517 - 4.1667 * (x) + 0.0109 * (x) * (x)) //0 grados son 517, 90 grados son 230  y -90 son 980 en valor del potenciometro
#define PULSOS_A_GRADOSSERVO(x) (181.025 - 0.4323 * (x) + (0.00016 * (x)) * (x) ) //inverso

#define RADIANES_A_GRADOS(x) ((x) * 57.2958) 
#define GRADOS_A_RADIANES(x) ((x) / 57.2958) 
//Pines motor brazo (motor 1)
#define R_EN 30
#define L_EN 32
#define R_PWM 12 
#define L_PWM 10

//motor prueba
#define IN1 6
#define IN2 7
#define ENB 8

//pines motor base (motor 0)
#define IN3 3
#define IN4 5 
#define ENA 11

//pines servo motor
#define IN1SERVO 52
#define IN2SERVO 53
#define ENASERVO 10

#define POTENCIOMETRO 0 //(A0)

//Constantes para el control pid
#define KP1 1
#define KI1 0.002
#define KD1 1.5
#define KP0 10
#define KI0 0.01
#define KD0 6
#define KPSERVO 5
#define KISERVO 0.01
#define KDSERVO 4
#define TIEMPO_PID 100 //cada cuanto tiempo se calcula el pid en ms
#define ZONA_MUERTA1 12 //zona muerta del motor 1 (en pwm 0-255)
#define ZONA_MUERTA0 80 //zona muerta del motor 0 (en pwm 0-255)
#define ZONA_MUERTASERVO 0 //zona muerta del motor servo (en pwm 0-255)

//Pines finales de carrera
#define PIN_FINAL_CARRERA0 26
#define PIN_FINAL_CARRERA1 27

#define PUNTOS 100

int operacion = 0;
float datos[5] = { 0,0,0,0,0 }; //guarda los datos de la lecura del serial

int pos_objetivo = 0; //posicion en grados positivo o negativo
long int cuenta_objetivo1 = 0; //posicion segun los pulsos del encoder por vuelta
long int cuenta_objetivo0 = 0;
long int cuenta_objetivoservo = 0;

EncoderClass encoder1; //El pin 1 necesita interrupci�n
EncoderClass encoder0;


//Motor_brazoClass motor1;

Motor_baseClass motor1;
/////////////////////////////////////////////
Motor_baseClass motor0;
/////////////////////////////////////////////
Motor_baseClass servo;


PIDClass pid1;
PIDClass pid0;
PIDClass pidservo;

volatile long int cuenta1 = 0; //Es volatile para que las interrupciones puedan cambiarlo sin problemas
volatile long int cuenta0 = 0;
int cuentaservo = 0;
int potenciometro = 0;

int velocidad1 = 0;
int velocidad0 = 0;
int velocidadservo = 0;

float puntos_recta[PUNTOS][3];
int contador = 0;

//Banderas para hacer varias cosas a la vez
bool mov_motor_pos = false;
bool mov_motor_vel = false;
bool calibrado = false;
bool mov_motor_recta = false;
int ant_datos = 0;
//Posiciones objetivo de los motores en pulsos
float pos_obj_grad0 = 0, pos_obj_grad1 = 0, pos_obj_gradservo = 0;
int vel_obj0 = 0, vel_obj1 = 0, vel_objservo = 0;
float pos_fin_x = 0, pos_fin_y = 0, pos_fin_z = 0;


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

	encoder1.inicializar(PIN1ENCODER1, PIN2ENCODER1);
	encoder0.inicializar(PIN1ENCODER0, PIN2ENCODER0);
	//////////////////////CAMBIAR CUANDO FUNCIONE///////////////////
	//motor1.inicializar(R_EN, L_EN, R_PWM, L_PWM);

	motor1.inicializar(IN1, IN2, ENB);
	motor0.inicializar(IN3, IN4, ENA);
	servo.inicializar(IN1SERVO, IN2SERVO, ENASERVO);
	pid1.inicializar(KP1, KI1, KD1, TIEMPO_PID, ZONA_MUERTA1);
	pid0.inicializar(KP0, KI0, KD0, TIEMPO_PID, ZONA_MUERTA0);
	pidservo.inicializar(KPSERVO, KISERVO, KDSERVO, TIEMPO_PID, ZONA_MUERTASERVO);
	Serial.begin(9600);
	//Serial.println("Serial encendido a 9600 baudios");
	
}


//////////////////////////////////////////////////////////////////////////////////////////
/******************************************Loop******************************************/
//////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
	cuentaservo = analogRead(A0); //POTENCIOMETRO
		switch ((int)datos[0])
		{
		case 1:
			//Parada de emergencia
			mov_motor_pos = false;
			mov_motor_vel = false;
			mov_motor_recta = false;
			parada_motores();
			break;
		case 2: 
			mov_motor_pos = true;
			mov_motor_vel = false;
			mov_motor_recta = false;
			if (datos[1] == 2) {
				pos_obj_grad0 = datos[2];
				pos_obj_grad1 = datos[3];
				pos_obj_gradservo = datos[4];
			}
			else{
				float posiciones[3];
				cinem_inverso(datos[2], datos[3], datos[4], posiciones);
				pos_obj_grad0 = posiciones[0];
				pos_obj_grad1 = posiciones[1];
				pos_obj_gradservo = posiciones[2];			
			}
			break;
		case 3: 
			if ((int)datos[0] != ant_datos) {
				mostrar_posicion();
			}
			break;
		
		case 4:
			mov_motor_pos = false;
			mov_motor_vel = true;
			mov_motor_recta = false;
			vel_obj0 = (int)datos[2];
			vel_obj1 = (int)datos[3];
			vel_objservo = (int)datos[4];
			mover_motores_velocidad();
			break;
			
		case 5://calibrar
			mov_motor_vel = false;
			mov_motor_pos = false;
			mov_motor_recta = false;
			if (!calibrado){
				calibrar();
			}
			break;
		case 6://Linea recta
			mov_motor_pos = false;
			mov_motor_vel = false;
			mov_motor_recta = true;
		
			/*	if (datos[1] == 2) {
				
			}
			else {
				pos_fin_x = datos[2];
				pos_fin_y = datos[3];
				pos_fin_z = datos[4];
			}
			*/
			if ((int)datos[0] != ant_datos) {
				generar_puntos(datos[2], datos[3], datos[4]);
				contador = 0;
			}
			break;
		case 99:
			Serial.println("cambio");
			break;
		default:
			break; 
		}
		ant_datos = datos[0];
		//	comprobar_limites();
		if (mov_motor_pos) {
			mover_motores_posicion();
	} 
		if (mov_motor_vel) {
			mover_motores_velocidad();
		}
		if (mov_motor_recta) {
		//	mover_recta(pos_fin_x, pos_fin_y, pos_fin_z);	
			if (mover_recta_puntos(contador)) contador++;
			if (contador >= PUNTOS - 1) contador = PUNTOS - 1;
		}
}

//////////////////////////////////////////////////////////////////////////////////////////
/**************************************serialEvent***************************************/
//////////////////////////////////////////////////////////////////////////////////////////


void serialEvent() {
	int i = 0;
//	Serial.println("���������������������������������������");
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

		cuenta_objetivo1 = (int)(pos_obj_grad1 * GRADOS_A_PULSOS1);
		cuenta_objetivo0 = (int)(pos_obj_grad0 * GRADOS_A_PULSOS0);
		cuenta_objetivoservo = (int)(GRADOS_A_PULSOSSERVO(pos_obj_gradservo));
		calcular_pid();
		motor1.velocidad(velocidad1);
		motor0.velocidad(velocidad0);
		servo.velocidad(velocidadservo);

	//	Serial.println("ha entrado");
	//	Serial.println(velocidad1);	
}

void mostrar_posicion() {

	int art0 = cuenta0 * PULSOS_A_GRADOS0; //q0
	int art1 = cuenta1 * PULSOS_A_GRADOS1; //q1
	int art2 = PULSOS_A_GRADOSSERVO((float)cuentaservo); //q2
    String responseMsg;
    responseMsg = "3#" + String(2) + "#" + art0 + "#" + art1 + "#" + art2;
	Serial.println(responseMsg);
        Serial.flush();
    
/*	float cart[3];
	cinem_directo(art0, art1, art2, cart);
	responseMsg = "3#" + String(1) + "#" + String(cart[0]) + "#" + String(cart[1]) + "#" + String(cart[2]);
	Serial.println(responseMsg);
	Serial.flush();
*/
}

void mover_motores_velocidad() {
	if (datos[1] == 2) {
		motor1.velocidad(vel_obj1);
		motor0.velocidad(vel_obj0);
		servo.velocidad(vel_objservo);
		//Serial.println(datos[3]);
	}
}

void parada_motores() {
	motor0.velocidad(0);
	motor1.velocidad(0);
	servo.velocidad(0);
}

void calibrar() {
	while (digitalRead(PIN_FINAL_CARRERA0)) {
		motor0.velocidad(-80);
	}	
	motor0.velocidad(0);
  encoder0.setCuenta(0);
	while (digitalRead(PIN_FINAL_CARRERA1)) {
		motor1.velocidad(-120);
	}	
 
	encoder1.setCuenta(0);
	motor1.velocidad(0);
/*	pos_obj_grad0 = 0;
	pos_obj_grad1 = 0;
	pos_obj_gradservo = 0;
	*/
	calibrado = true;
//	mov_motor_pos = true;
}

void calcular_pid() {
	velocidad1 = pid1.calcular(cuenta1, cuenta_objetivo1);
	velocidad0 = pid0.calcular(cuenta0, cuenta_objetivo0);
	velocidadservo = pidservo.calcular(cuentaservo, cuenta_objetivoservo);
}

void comprobar_limites() {
	if ((cuenta0 * PULSOS_A_GRADOS0) < 1) {
		mov_motor_pos = true;
		mov_motor_vel = false;
		pos_obj_grad0 = 5;
	}
	if ((cuenta0 * PULSOS_A_GRADOS0) >270) {
		mov_motor_pos = true;
		mov_motor_vel = false;
		pos_obj_grad0 = 270;
	}
	if ((cuenta1 * PULSOS_A_GRADOS0) < 1) {
		mov_motor_pos = true;
		mov_motor_vel = false;
		pos_obj_grad1 = 5;
	}
	if ((cuenta0 * PULSOS_A_GRADOS0) > 170) {
		mov_motor_pos = true;
		mov_motor_vel = false;
		pos_obj_grad1 = 170;
	}
	if (PULSOS_A_GRADOSSERVO((float)cuentaservo) > 92) {
		mov_motor_pos = true;
		mov_motor_vel = false;
		pos_obj_gradservo = 90;
	}
	if (PULSOS_A_GRADOSSERVO((float)cuentaservo) < -92) {
		mov_motor_pos = true;
		mov_motor_vel = false;
		pos_obj_gradservo = -90;
	}
}

void mover_recta(float xfin, float yfin, float zfin) {
	#define INCREMENTO 20

	float posicion_actual[3];
	float vector_director[3];
	float posicion_objetivo[3];
	float posicion_objetivo_art[3];
	cinem_directo(cuenta0 * PULSOS_A_GRADOS0, cuenta1 * PULSOS_A_GRADOS1, PULSOS_A_GRADOSSERVO((float)cuentaservo), posicion_actual);
	vector_director[0] = xfin - posicion_actual[0];
	vector_director[1] = yfin - posicion_actual[1];
	vector_director[2] = zfin - posicion_actual[2];
	normalizar(vector_director);
	for (int i = 0; i < 3; i++) {
		posicion_objetivo[i] = posicion_actual[i] + vector_director[i] * INCREMENTO;
	}
	cinem_inverso(posicion_objetivo[0], posicion_objetivo[1], posicion_objetivo[2], posicion_objetivo_art);
	pos_obj_grad0 = posicion_objetivo_art[0];
	pos_obj_grad1 = posicion_objetivo_art[1];
	pos_obj_gradservo = posicion_objetivo_art[2];
	mover_motores_posicion();
}

bool mover_recta_puntos(int i) {
#define ERROR_CUADRADO 25

	float posicion_actual[3];
	float vector_error[3];
	float error_cuadrado;
	pos_obj_grad0 = puntos_recta[i][0];
	pos_obj_grad1 = puntos_recta[i][1];
	pos_obj_gradservo = puntos_recta[i][2];
	//moverse al punto
	mover_motores_posicion();
	//comprobar si ha llegado al punto con un margen de error y hay que pasar al siguiente
	vector_error[0] = puntos_recta[i][0] - cuenta0 * PULSOS_A_GRADOS0;
	vector_error[1] = puntos_recta[i][1] - cuenta1 * PULSOS_A_GRADOS1;
	vector_error[2] = puntos_recta[i][2] - PULSOS_A_GRADOSSERVO((float)cuentaservo);
	error_cuadrado = (pow(vector_error[0], 2) + pow(vector_error[1], 2) + pow(vector_error[2], 2));
	if (error_cuadrado < ERROR_CUADRADO) return true;
	else return false;
}

//genera los puntos por los que tiene que ir en articulares
void generar_puntos(float xfin, float yfin, float zfin) {

	float posicion_actual[3];
	float vector_director[3];
	float posicion_objetivo[3];
	float posicion_objetivo_art[3];
	float distancia;
	float puntos_aux[PUNTOS][3];
	cinem_directo(cuenta0 * PULSOS_A_GRADOS0, cuenta1 * PULSOS_A_GRADOS1, PULSOS_A_GRADOSSERVO((float)cuentaservo), posicion_actual);
	vector_director[0] = xfin - posicion_actual[0];
	vector_director[1] = yfin - posicion_actual[1];
	vector_director[2] = zfin - posicion_actual[2];
	distancia = sqrt(pow(vector_director[0], 2) + pow(vector_director[1], 2) + pow(vector_director[2], 2));
	distancia = distancia / PUNTOS;
	normalizar(vector_director);
	for (int i = 0; i < PUNTOS; i++) {
		for (int j = 0; j < 3; j++) {
			puntos_aux[i][j] = posicion_actual[j] + vector_director[j] * distancia * i;
		}	
	}
	//se pasa a articulares
	for (int i = 0; i < PUNTOS; i++) {
		cinem_inverso(puntos_aux[i][0], puntos_aux[i][1], puntos_aux[i][2], puntos_recta[i]);
	}
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

//////////////////////////////////////////////////////////////////////////////////////////
/************************************Modelo cinemático***********************************/
//////////////////////////////////////////////////////////////////////////////////////////

void cinem_inverso(float x, float y, float z, float* pos) {
	pos[0] = RADIANES_A_GRADOS(atan2(y, x)); //q1
	pos[2] = RADIANES_A_GRADOS(acos((pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(L2, 2) - pow(L3, 2)) / (2 * L2*L3))); //q3
	pos[1] = RADIANES_A_GRADOS(atan2(z, sqrt(pow(x, 2) + pow(y, 2))) - atan2(L3*sin(pos[2]), L2 + L3*cos(pos[2])));//q2
}

void cinem_directo(float q1grad, float q2grad, float q3grad, float* carte) {
	float q1 = GRADOS_A_RADIANES(q1grad);
	float q2 = GRADOS_A_RADIANES(q2grad);
	float q3 = GRADOS_A_RADIANES(q3grad);
	carte[0] = L3*cos(q1)*cos(q2 + q3) + L2*cos(q1)*cos(q2);
	carte[1] = L3*sin(q1)*cos(q2 + q3) + L2*sin(q1)*cos(q2);
	carte[2] = L3*sin(q2 + q3) + L2*sin(q2);
}

//////////////////////////////////////////////////////////////////////////////////////////
/***********************************Funciones de apoyo***********************************/
//////////////////////////////////////////////////////////////////////////////////////////

void normalizar(float* vector3) {
	float aux;
	aux = sqrt(pow(vector3[0], 2) + pow(vector3[1], 2) + pow(vector3[2], 2));
	vector3[0] /= aux;
	vector3[1] /= aux;
	vector3[2] /= aux;
}

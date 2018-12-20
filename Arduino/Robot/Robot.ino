/// Variables //////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <PID_v1.h>
#include "TimerOne.h"
#include "MPU6050_tockn.h"
#include <Wire.h>

#define RIGHT_PWM 5
#define LEFT_PWM 6
#define   Kd 0.0
#define   Ki 0.3
#define   Kp 2.8
// PWM_SPEED_RELATION    rpm= PWM*m -b
#define   m  0.26
#define   b  2
#define LEFT_ECHO 8
#define LEFT_TRIG 9
#define RIGHT_ECHO 10
#define RIGHT_TRIG 11
#define FORWARD_TRIG 12
#define FORWARD_ECHO 13
String path = "ffrlrfffflfffrff";
int old_angle, new_angle;
volatile float left, right, forward;
double Setpoint = 19;
double right_pwm = 90;
double left_pwm = 94;
int encoderR_pin = 2;             //Pin 2, donde se conecta el encoder
int encoderL_pin = 3;
volatile unsigned int stepR = 0;           // Revoluciones por minuto calculadas.
volatile unsigned int stepL = 0;
float velocityR = 0;                  //Velocidad en [Km/h]
float velocityL = 0;
double pulsesR = 0;       // Número de pulsos leidos por el Arduino en un segundo
double pulsesL = 0;
unsigned long timeoldR = 0;  // Tiempo
unsigned long timeoldL = 0;
unsigned int pulsesperturnR = 8; // Número de muescas que tiene el disco del encoder.
unsigned int pulsesperturnL = 8;
const int wheel_diameterR = 64;   // Diámetro de la rueda pequeña[mm]
const int wheel_diameterL = 64;
static volatile unsigned long debounceR = 0; // Tiempo del rebote.
static volatile unsigned long debounceL = 0;
PID RightPID(&pulsesR, &right_pwm, &Setpoint, Kp, Ki, Kd, DIRECT);
PID LeftPID(&pulsesL, &left_pwm, &Setpoint, Kp, Ki, Kd, DIRECT);
void drive(byte to, int steps);
void read_sonar(void);
////  Configuración del Arduino /////////////////////////////////////////////////////////
void counterR() {
  if (  digitalRead (encoderR_pin) && (micros() - debounceR > 500) && digitalRead (encoderR_pin) ) {
    // Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
    debounceR = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
    pulsesR++;
    stepR ++;
  }  // Suma el pulso bueno que entra.
  else ;
}
int flag = 0;
void counterL() {
  if (  digitalRead (encoderL_pin) && (micros() - debounceL > 500) && digitalRead (encoderL_pin) ) {
    // Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
    debounceL = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
    pulsesL++;
    stepL ++;
    // Serial.println(stepL);

  }  // Suma el pulso bueno que entra.
  else ;


  if (pulsesL == 7) flag = 1;
}
void timerIsr()
{
  RightPID.Compute();
  LeftPID.Compute();
  noInterrupts(); //Don't process interrupts during calculations // Desconectamos la interrupción para que no actué en esta parte del programa.
  //    rpmR = (60 * 1000 / pulsesperturnR ) / (millis() - timeoldR) * pulsesR; // Calculamos las revoluciones por minuto
  //    velocityR = rpmR * 3.1416 * wheel_diameterR * 60 / 1000000; // Cálculo de la velocidad en [Km/h]
  timeoldR = millis(); // Almacenamos el tiempo actual.
  //    Serial.print(pulsesR, DEC); Serial.print("     ");
  //    Serial.print("     ");
  //    Serial.print(right_pwm);
  //    Serial.print("     ");
  pulsesR = 0;  // Inicializamos los pulsos.

  //   rpmL = (60 * 1000 / pulsesperturnL ) / (millis() - timeoldL) * pulsesL; // Calculamos las revoluciones por minuto
  //  velocityL = rpmL * 3.1416 * wheel_diameterL * 60 / 1000000; // Cálculo de la velocidad en [Km/h]
  timeoldL = millis(); // Almacenamos el tiempo actual.
  //    Serial.print(pulsesL, DEC);
  //    Serial.print("     ");
  //    Serial.println(left_pwm);
  pulsesL = 0;  // Inicializamos los pulsos.
  interrupts(); // Restart the interrupt processing // Reiniciamos la interrupción
}
void setup() {
  Serial.begin(9600); // Configuración del puerto serie
  while (!Serial.available());
  path = Serial.readStringUntil('\n');

  //timer init
  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);
  pinMode(FORWARD_TRIG, OUTPUT);
  pinMode(FORWARD_ECHO, INPUT);

  Timer1.initialize(1000000); // set timer for 1sec
  Timer1.attachInterrupt( timerIsr ); // enable the timer

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);

  pinMode(encoderR_pin, INPUT); // Configuración del pin nº2
  pinMode(encoderL_pin, INPUT);
  attachInterrupt(0, counterR, RISING); // Configuración de la interrupción 0, donde esta conectado.
  attachInterrupt(1, counterL, RISING);
  pulsesR = 0;
  pulsesL = 0;
  //  rpmR = 0;
  //  rpmL = 0;
  timeoldR = 0;
  timeoldL = 0;

  //Turn the PID on
  RightPID.SetMode(AUTOMATIC);
  LeftPID.SetMode(AUTOMATIC);
  //Adjust PID values
  RightPID.SetTunings(Kp, Ki, Kd);
  LeftPID.SetTunings(Kp, Ki, Kd);

  //  Serial.print("Seconds ");
  //  Serial.print("RPMR ");
  //  Serial.print("PulsesR ");
  //  Serial.print("VelocityR[Km/h]  ");
  //  Serial.print("RPML ");
  //  Serial.print("PulsesL ");
  //  Serial.println("VelocityL[Km/h]");

}
int x = 1;
////  Programa principal ///////////////////////////////////////////////////////////////////////



void loop() {


  for (int i = 0; i < path.length(); i++) {

    if ((path[i] == 'r') || (path[i] == 'l')) {
      drive('s', 5);
      drive(path[i], 5);
      drive('s', 5);
      if (i < path.length() - 1 && path[i + 1] == 'r') {
        read_sonar();
        while (right < 12) {
          drive('f', 1);
          read_sonar();
        }

        drive('f', 3);
      }
      else if (i < path.length() - 1 && path[i + 1] == 'l') {
        read_sonar();
        while (left < 12) {
          drive('f', 1);
          read_sonar();
        }
        drive('f', 3);

      }
    }

    else if (path[i] == 'f') {

      drive(path[i], 8);

    } else if (path[i] == 'x') {

      drive(path[i], 7);

    }


  }
  while (1) {
    drive('s', 5);

  }

}



// if(x==1) {
//   delay(2000);
//   drive('r');
// delay(430);
// x=0;
// }


void drive(byte to, int steps) {
  switch (to) {
    case 'f':
      read_sonar();

      while ( steps > stepL && steps > stepR) {
        read_sonar();
        if (forward < 15) {
          analogWrite(LEFT_PWM, 0);
          analogWrite(RIGHT_PWM, 0);
          //delay(500);
          break;

        }
        analogWrite(LEFT_PWM, left_pwm);
        analogWrite(RIGHT_PWM, right_pwm);
        //        Serial.println(forward);

      }


      break;
    case 'r':
      analogWrite(LEFT_PWM, left_pwm);
      analogWrite(RIGHT_PWM, 0);
      while (steps > stepL) {}
      right_pwm = left_pwm;

      break;
    case 'l':
      analogWrite(LEFT_PWM, 0);
      analogWrite(RIGHT_PWM, right_pwm);
      while ( steps > stepR) ;
      left_pwm = right_pwm;

      break;
    case 's':
      analogWrite(LEFT_PWM, 0);
      analogWrite(RIGHT_PWM, 0);
      delay(500);
      break;


  }


  stepL = 0;
  stepR = 0;
  right_pwm = 90;
  left_pwm = 94;
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
}


void read_sonar() {
  // do{
  digitalWrite(LEFT_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(LEFT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(LEFT_TRIG, LOW);

  left = pulseIn(LEFT_ECHO, HIGH) * 0.034 / 2;
  // }while(left>100);

  //do{
  digitalWrite(RIGHT_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(RIGHT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(RIGHT_TRIG, LOW);

  right = pulseIn(RIGHT_ECHO, HIGH) * 0.034 / 2;
  //}while(right>100);

  //do{
  digitalWrite(FORWARD_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(FORWARD_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(FORWARD_TRIG, LOW);

  forward = pulseIn(FORWARD_ECHO, HIGH) * 0.034 / 2;
  //}while(forward>100);
  //    Serial.println(forward);
}

////Fin de programa principal //////////////////////////////////////////////////////////////////////////////////
///////////////////////////Función que cuenta los pulsos buenos ///////////////////////////////////////////

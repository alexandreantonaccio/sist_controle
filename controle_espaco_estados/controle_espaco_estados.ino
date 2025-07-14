#include <Stepper.h>
#include <BasicLinearAlgebra.h>

const int STEPS_PER_REVOLUTION = 200;
Stepper myStepper(STEPS_PER_REVOLUTION, 4, 5, 6, 7);
const int PIN_TRIG = 9;
const int PIN_ECHO = 10;

const float m1 = 0.05;
const float m2 = 0.05;
const float k2 = 4.16;
const float b  = 0.5;
const float c1 = 0.1;
const float c2 = 0.1; 

BLA::Matrix<4, 4> A = {
    0, 0, 1, 0,
    0, 0, 0, 1,
    -k2/m1, k2/m1, -(b+c1)/m1, b/m1,
    k2/m2, -k2/m2, b/m2, -(b+c2)/m2
};

BLA::Matrix<4, 1> B = {0, 0, 1.0/m1, 0};
BLA::Matrix<1, 4> C = {0, 1, 0, 0};
BLA::Matrix<1, 4> K = {-1.6566, 3.7, 0.1, 0.3837};
BLA::Matrix<4, 1> L = {0.85, 20.5, 5.12, 110.2};

// Vetores de estado
BLA::Matrix<4, 1> x_hat = {0, 0, 0, 0};
BLA::Matrix<4, 1> x_des = {0, 0, 0, 0};

const float dt = 0.02; 
const int maxRpm = 100;
const float maxControlForce = 10.0;

void setup() {
  Serial.begin(9600);
  Serial.println("Sistema de Controlo por Espaco de Estados Iniciado");

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  x_des(0) = 23.5; 
  x_des(1) = 14.0; 
  x_des(2) = 0.0; 
  x_des(3) = 0.0; 
}

void loop() {
  static unsigned long lastTime = 0;
  if (micros() - lastTime >= dt * 1000000) {
    lastTime = micros();
    float y_medido = lerDistanciaCm();
    float u = -(K * (x_hat - x_des))(0, 0);

    if (u > maxControlForce) u = maxControlForce;
    if (u < -maxControlForce) u = -maxControlForce;

    float rpm = map(u, -maxControlForce, maxControlForce, -maxRpm, maxRpm);
    moverMotorRpm(rpm);
    
    float y_estimado = (C * x_hat)(0, 0);
    float erro_medicao = y_medido - y_estimado;

    BLA::Matrix<4, 1> x_hat_dot = (A * x_hat) + (B * u) + (L * erro_medicao);
    x_hat = x_hat + x_hat_dot * dt;
    imprimirDados();
  }
}

/**
 * @brief 
 */
void moverMotorRpm(float rpm) {
  if (abs(rpm) < 1.0) {
    return;
  }
  long intervaloMicros = (60L * 1000000L) / (STEPS_PER_REVOLUTION * abs(rpm));
  static unsigned long ultimoPassoMicros = 0;

  if (micros() - ultimoPassoMicros >= intervaloMicros) {
    ultimoPassoMicros = micros();
    if (rpm > 0) {
      myStepper.step(1);
    } else {
      myStepper.step(-1);
    }
  }
}

/**
 * @brief 
 */
float lerDistanciaCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duracao = pulseIn(PIN_ECHO, HIGH, 25000);
  float distancia = (duracao * 0.0343) / 2.0;
  if (distancia == 0 || distancia > 400) {
    return x_hat(1);
  }
  return distancia;
}

/**
 * @brief 
 */
void imprimirDados() {
  static unsigned long ultimoPrintMs = 0;
  if (millis() - ultimoPrintMs > 250) {
    ultimoPrintMs = millis();
    Serial.print("Setpoint(x2): \n");
    Serial.print(x_des(1));
    Serial.print(" | Atual(x2): ");
    Serial.print(x_hat(1));
    Serial.print(" | Vel(x2): ");
    Serial.print(x_hat(3));
  }
}

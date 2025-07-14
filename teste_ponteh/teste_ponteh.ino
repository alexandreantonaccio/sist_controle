#include <Stepper.h>

const int PASSOS_POR_VOLTA = 200;
Stepper meuMotor(PASSOS_POR_VOLTA, 4, 5, 6, 7);

void setup() {
  // Define a velocidade do motor em RPM (rotações por minuto)
  // Experimente com este valor para encontrar uma velocidade adequada
  meuMotor.setSpeed(60);
}

void loop() {
  // Gira o motor uma volta completa (200 passos) no sentido horário
  // Para girar no sentido horário, o número de passos é positivo.
  meuMotor.step(200);

  // Aguarda um segundo antes de girar novamente
  delay(1000);
  meuMotor.step(-200);
  delay(1000);
}
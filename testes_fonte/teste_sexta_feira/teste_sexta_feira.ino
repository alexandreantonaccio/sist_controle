#include <Stepper.h>
#include <PID_v1.h>
#include <SimpleKalmanFilter.h> // Biblioteca para o Filtro de Kalman

// --- Configurações do Motor de Passo ---
const int PASSOS_MOTOR = 200; // Passos por revolução para o seu motor
Stepper myStepper(PASSOS_MOTOR, 4, 5, 6, 7); // Pinos do driver do motor (IN1, IN2, IN3, IN4)
const int maxRpm = 100; // RPM máximo que o motor pode atingir

// --- Configurações do Sensor Ultrassônico ---
const int PIN_TRIG = 9;
const int PIN_ECHO = 10;

// --- Configurações do Filtro de Kalman ---
SimpleKalmanFilter kalmanFilter(2.0, 2.0, 0.1);

// --- Variáveis do Controlador PID ---
double setpoint;       // O valor desejado (distância em cm)
double distanciaAtual; // O valor medido (filtrado)
double rpmSaida;       // A saída do PID (velocidade do motor em RPM)

// --- Constantes de Sintonia do PID (Ganhos) ---
double Kp = 0.610;
double Ki = 12.26;
double Kd = 0.103;

// --- Parâmetros de Controle Adicionais ---
const float ZONA_MORTA_CM = 0.4; // Se o erro for menor que isso, o motor para.
const float RPM_MINIMO = 1.0;    // RPM mínimo para acionar o motor

// --- Objeto PID ---
PID myPID(&distanciaAtual, &rpmSaida, &setpoint, Kp, Ki, Kd, DIRECT);

// --- Variáveis de Controle de Teste ---
const long INTERVALO_TROCA_MS = 30000; // Alterna o setpoint a cada 30 segundos
unsigned long ultimoTempoTroca = 0;
bool setpointAtualE15 = true;

// --- Declarações de Funções Auxiliares ---
float lerDistanciaBruta();
float lerDistanciaCm();
void imprimirDados();
void desligarMotor();

void setup() {
  Serial.begin(9600);
  
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  
  // --- Inicialização do sensor e do Filtro de Kalman ---
  Serial.println("Inicializando sensor e Filtro de Kalman...");
  float primeiraLeitura = 0;
  while (primeiraLeitura <= 0 || primeiraLeitura > 400) {
    primeiraLeitura = lerDistanciaBruta();
    delay(100);
  }
  
  distanciaAtual = kalmanFilter.updateEstimate(primeiraLeitura);
  
  Serial.print("Filtro de Kalman inicializado com distancia: ");
  Serial.println(distanciaAtual);
  
  Serial.println("Sistema de Controle PID Iniciado.");
  
  setpoint = 15.0;

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20);
  myPID.SetOutputLimits(-maxRpm, maxRpm);
}

void loop() {
  // --- Lógica para alternar o setpoint automaticamente (para fins de teste) ---
  if (millis() - ultimoTempoTroca >= INTERVALO_TROCA_MS) {
    ultimoTempoTroca = millis();
    if (setpointAtualE15) {
      setpoint = 10.0;
      setpointAtualE15 = false;
    } else {
      setpoint = 15.0;
      setpointAtualE15 = true;
    }
    Serial.print("Novo setpoint definido: ");
    Serial.println(setpoint);
  }
  
  distanciaAtual = lerDistanciaCm();
  double erro = setpoint - distanciaAtual;

  // --- LÓGICA DE CONTROLE COM ANTI-WINDUP ---
  bool pidSaturado = (rpmSaida >= maxRpm && erro > 0) || (rpmSaida <= -maxRpm && erro < 0);

  if (abs(erro) < ZONA_MORTA_CM) {
    rpmSaida = 0;
    myPID.SetMode(MANUAL); 
    myPID.SetMode(AUTOMATIC);
  } else if (pidSaturado) {
    // ANTI-WINDUP: Mantém a saída no limite sem calcular o PID.
  } else {
    myPID.Compute();
  }
  
  // --- NOVA LÓGICA DE CONTROLE DO MOTOR ---
  if (abs(rpmSaida) > RPM_MINIMO) {
    // Define a velocidade do motor (o valor deve ser sempre positivo)
    myStepper.setSpeed(abs(rpmSaida));
    // Executa um passo na direção correta (positiva ou negativa)
    myStepper.step(rpmSaida > 0 ? 1 : -1);
  } else {
    // Se a velocidade for muito baixa, mantém o motor desligado
    desligarMotor();
  }
  
  imprimirDados();

  // --- DELAY PARA ESTABILIZAR LEITURAS DO SENSOR ---
  delay(30);
}

// Função que lê o sensor e aplica o filtro de Kalman
float lerDistanciaCm() {
  float novaLeitura = lerDistanciaBruta();

  if (novaLeitura <= 0 || novaLeitura > 400) {
    return distanciaAtual;
  }

  float distanciaFiltrada = kalmanFilter.updateEstimate(novaLeitura);
  return distanciaFiltrada;
}

// Função para obter uma única leitura do sensor ultrassônico
float lerDistanciaBruta() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duracao = pulseIn(PIN_ECHO, HIGH, 25000);
  return (duracao * 0.0343) / 2.0;
}

// Função para desligar as bobinas do motor e economizar energia
void desligarMotor() {
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}

// Função para imprimir os dados na Serial para monitoramento e plotagem
void imprimirDados() {
  static unsigned long ultimoPrintMs = 0;
  if (millis() - ultimoPrintMs > 100) {
    ultimoPrintMs = millis();
    Serial.print("Tempo:");
    Serial.print(millis() / 1000.0, 3);
    Serial.print(", Setpoint:");
    Serial.print(setpoint, 2);
    Serial.print(", Distancia:");
    Serial.print(distanciaAtual, 2);
    Serial.print(", RPM:");
    Serial.println(rpmSaida, 2);
  }
}

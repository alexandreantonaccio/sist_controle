#include <Stepper.h>
#include <PID_v1.h>
#include <SimpleKalmanFilter.h> // Biblioteca para o Filtro de Kalman

// --- Configurações do Motor de Passo ---
const int PASSOS_MOTOR = 200; // Passos por revolução para o seu motor
Stepper myStepper(PASSOS_MOTOR, 4, 5, 6, 7); // Pinos do driver do motor (IN1, IN2, IN3, IN4)
const int maxRpm = 60; // RPM máximo que o motor pode atingir

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
double Kp = 1.129;
double Ki = 0.414;
double Kd = 0.340;

// --- Parâmetros de Controle Adicionais ---
const float ZONA_MORTA_CM = 0.1; // Se o erro for menor que isso, o motor para.
const float RPM_MINIMO = 0.5;    // RPM mínimo para acionar o motor

// --- Objeto PID ---
PID myPID(&distanciaAtual, &rpmSaida, &setpoint, Kp, Ki, Kd, DIRECT);

// --- Variáveis de Controle de Teste ---
const long INTERVALO_TROCA_MS = 70000; // Alterna o setpoint a cada 60 segundos
unsigned long ultimoTempoTroca = 0;
bool setpointAtualE15 = true;

// --- Variáveis para Controle de Passo Não-Bloqueante ---
unsigned long ultimoPassoMicros = 0;
long passoIntervaloMicros = 0;

// --- NOVAS Variáveis para Desacoplamento de Tarefas ---
unsigned long ultimoTempoLeitura = 0;
// Intervalo de 50ms para a leitura do sensor. É um valor seguro para evitar eco.
const long INTERVALO_LEITURA_MS = 50; 

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
  // Ajusta o tempo de amostragem do PID para corresponder ao intervalo de leitura do sensor
  myPID.SetSampleTime(INTERVALO_LEITURA_MS);
  myPID.SetOutputLimits(-maxRpm, maxRpm);
}

void loop() {
  // --- TAREFA 1: CONTROLE DO MOTOR (Executa o mais rápido possível) ---
  // Esta lógica agora roda livremente, sem ser bloqueada pela leitura do sensor.
  if (abs(rpmSaida) > RPM_MINIMO) {
    // Calcula o intervalo de tempo necessário entre os passos para atingir o RPM desejado
    passoIntervaloMicros = 60000000L / (PASSOS_MOTOR * abs(rpmSaida));
    
    unsigned long agoraMicros = micros();
    
    // Verifica se já passou tempo suficiente desde o último passo
    if (agoraMicros - ultimoPassoMicros >= passoIntervaloMicros) {
      ultimoPassoMicros = agoraMicros; // Atualiza o tempo do último passo
      myStepper.step(rpmSaida > 0 ? 1 : -1);
    }
  } else {
    // Se o RPM for muito baixo, desliga o motor para economizar energia
    desligarMotor();
  }

  // --- TAREFA 2: LEITURA DO SENSOR E CÁLCULO PID (Executa em intervalos controlados) ---
  unsigned long agoraMillis = millis();
  if (agoraMillis - ultimoTempoLeitura >= INTERVALO_LEITURA_MS) {
    ultimoTempoLeitura = agoraMillis; // Atualiza o tempo da última execução

    // 1. Lê o sensor
    distanciaAtual = lerDistanciaCm();
    
    // 2. Calcula a saída do PID com base na nova leitura
    double erro = setpoint - distanciaAtual;
    bool pidSaturado = (rpmSaida >= maxRpm && erro > 0) || (rpmSaida <= -maxRpm && erro < 0);

    if (abs(erro) < ZONA_MORTA_CM) {
      rpmSaida = 0;
      myPID.SetMode(MANUAL); 
      myPID.SetMode(AUTOMATIC);
    } else if (pidSaturado) {
      // Anti-windup: não faz nada, mantém a saída no limite.
    } else {
      myPID.Compute(); // O PID só é calculado aqui, a cada intervalo definido.
    }
  }

  // --- Lógica para alternar o setpoint (já era não-bloqueante) ---
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
  
  // --- Impressão de dados (já era não-bloqueante) ---
  imprimirDados();
}

// Função que lê o sensor e aplica o filtro de Kalman
float lerDistanciaCm() {
  float novaLeitura = lerDistanciaBruta();

  // Ignora leituras inválidas e retorna o último valor válido
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
  
  // Timeout de 25000 microssegundos (equivale a ~4.3 metros)
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
  // Imprime os dados a cada 100ms para não sobrecarregar a Serial
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

#include <Stepper.h>
#include <SimpleKalmanFilter.h> // Biblioteca para o Filtro de Kalman
#include <math.h>              // Incluído para a função abs()

// --- Configurações do Motor de Passo ---
const int PASSOS_MOTOR = 200; // Passos por revolução para o seu motor
Stepper myStepper(PASSOS_MOTOR, 4, 5, 6, 7); // Pinos do driver do motor (IN1, IN2, IN3, IN4)

// --- Constante de Conversão ---
const float PASSOS_POR_CM = (float)PASSOS_MOTOR / 4.0; // 50.0 passos/cm

// --- Configurações do Sensor Ultrassônico ---
const int PIN_TRIG = 9;
const int PIN_ECHO = 10;

// --- Configurações do Filtro de Kalman ---
SimpleKalmanFilter kalmanFilter(2.0, 2.0, 0.1); 

// --- Variáveis de Controle ---
double setpointCm = 15.0;
double distanciaAtualCm;
int direcaoMotor = 0; // 0=Parado, 1=Avançar, -1=Recuar
int velocidadeAtualRpm = 0; // Para monitoramento no Serial

// --- Parâmetros de Controle Avançado ---
const float ZONA_MORTA_CM = 0.1;      // Zona morta de 0.1cm. Se o erro for menor, o motor para.
const float ZONA_CALMA_CM = 1.0;      // Zona de transição para velocidade lenta (1.0 cm de margem)

// --- !! MUDANÇA PRINCIPAL: DUAS VELOCIDADES !! ---
const int VELOCIDADE_RAPIDA_RPM = 60; // Velocidade para grandes distâncias
const int VELOCidade_AJUSTE_RPM = 15; // Velocidade lenta para ajuste fino

// --- Variáveis de Controle de Teste ---
const long INTERVALO_TROCA_MS = 45000;
unsigned long ultimoTempoTroca = 0;
bool setpointAtualE15 = true;

// --- Variáveis para Desacoplamento de Tarefas ---
unsigned long ultimoTempoLeitura = 0;
const long INTERVALO_LEITURA_MS = 40; // Intervalo para a lógica de decisão

// --- Declarações de Funções Auxiliares ---
float lerDistanciaBruta();
float lerDistanciaCm();
void imprimirDados();
void desligarMotor();

void setup() {
  Serial.begin(9600);
  
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  
  Serial.println("Inicializando sensor e Filtro de Kalman...");
  float primeiraLeitura = 0;
  while (primeiraLeitura <= 0 || primeiraLeitura > 400) {
    primeiraLeitura = lerDistanciaBruta();
    delay(100);
  }
  
  distanciaAtualCm = kalmanFilter.updateEstimate(primeiraLeitura);
  
  Serial.print("Filtro de Kalman inicializado com distancia: ");
  Serial.println(distanciaAtualCm);
  
  Serial.println("Sistema de Controle com Duas Velocidades Iniciado.");
}

void loop() {
  unsigned long agora = millis();
  
  // --- TAREFA 1: LÓGICA DE DECISÃO (executa a cada 40ms) ---
  if (agora - ultimoTempoLeitura >= INTERVALO_LEITURA_MS) {
    ultimoTempoLeitura = agora;

    distanciaAtualCm = lerDistanciaCm();
    double erroCm = setpointCm - distanciaAtualCm;

    // LÓGICA 1: Erro desprezível (dentro da zona morta). Objetivo alcançado.
    if (abs(erroCm) < ZONA_MORTA_CM) {
      direcaoMotor = 0;
      velocidadeAtualRpm = 0;
    }
    // LÓGICA 2: Erro grande (fora da zona calma). Movimento em velocidade RÁPIDA.
    else if (abs(erroCm) >= ZONA_CALMA_CM) {
      myStepper.setSpeed(VELOCIDADE_RAPIDA_RPM);
      direcaoMotor = (erroCm > 0) ? 1 : -1;
      velocidadeAtualRpm = VELOCIDADE_RAPIDA_RPM;
    }
    // LÓGICA 3: Erro pequeno (dentro da zona calma). Movimento em velocidade LENTA.
    else {
      myStepper.setSpeed(VELOCidade_AJUSTE_RPM);
      direcaoMotor = (erroCm > 0) ? 1 : -1;
      velocidadeAtualRpm = VELOCidade_AJUSTE_RPM;
    }
  }

  // --- TAREFA 2: ATUAÇÃO DO MOTOR (executa o mais rápido possível) ---
  if (direcaoMotor != 0) {
    // Ação: Se a decisão for mover, dê um passo na direção definida.
    // Como esta linha está fora do timer de 40ms, ela será chamada continuamente,
    // fazendo o motor girar na velocidade definida em setSpeed().
    myStepper.step(direcaoMotor);
  } else {
    // Ação: Se a decisão for parar, desliga as bobinas.
    desligarMotor();
  }

  // --- TAREFA 3: Lógica para alternar o setpoint ---
  if (agora - ultimoTempoTroca >= INTERVALO_TROCA_MS) {
    ultimoTempoTroca = agora;
    setpointAtualE15 = !setpointAtualE15;
    setpointCm = setpointAtualE15 ? 15.0 : 10.0;
    Serial.print("Novo setpoint definido: ");
    Serial.println(setpointCm);
  }
  
  // --- TAREFA 4: Impressão de dados ---
  imprimirDados();
}

// Função que lê o sensor e aplica o filtro de Kalman
float lerDistanciaCm() {
  float novaLeitura = lerDistanciaBruta();
  if (novaLeitura <= 0 || novaLeitura > 400) {
    return distanciaAtualCm; 
  }
  return kalmanFilter.updateEstimate(novaLeitura);
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

// Função para imprimir os dados na Serial para monitoramento
void imprimirDados() {
  static unsigned long ultimoPrintMs = 0;
  if (millis() - ultimoPrintMs > 100) { 
    ultimoPrintMs = millis();
    Serial.print("Tempo:");
    Serial.print(millis() / 1000.0, 3);
    Serial.print(", Setpoint:");
    Serial.print(setpointCm, 2);
    Serial.print(", Distancia:");
    Serial.print(distanciaAtualCm, 2);
    Serial.print(", Velocidade_RPM:");
    Serial.print(velocidadeAtualRpm);
    Serial.print(", Direcao:");
    Serial.println(direcaoMotor);
  }
}

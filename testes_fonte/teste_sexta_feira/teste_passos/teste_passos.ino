#include <Stepper.h>
#include <SimpleKalmanFilter.h>

// ==============================================================================
// --- CONFIGURAÇÕES FÍSICAS E DO ATUADOR ---
// ==============================================================================
// --- Motor de Passo ---
const int PASSOS_MOTOR = 200; // Passos por revolução do seu motor
Stepper myStepper(PASSOS_MOTOR, 4, 5, 6, 7); // Pinos do driver (IN1, IN2, IN3, IN4)

// --- Calibração Mecânica (CRÍTICO) ---
// Com base na sua informação: 200 passos = 4 cm de deslocamento.
const float PASSOS_POR_CM = 50.0; // (200 passos / 4 cm)

// --- Sensor Ultrassônico ---
const int PIN_TRIG = 9;
const int PIN_ECHO = 10;

// ==============================================================================
// --- CONFIGURAÇÕES DO CONTROLADOR ---
// ==============================================================================
// --- Filtro de Kalman ---
SimpleKalmanFilter kalmanFilter(2.0, 2.0, 0.1); // Parâmetros (measurement_uncertainty, estimation_uncertainty, process_noise)

// --- Variáveis de Controle ---
double setpoint_cm;       // O valor desejado (distância em cm)
double distanciaAtual_cm; // O valor medido e filtrado (em cm)

// --- Parâmetros de Operação do Controle ---
const int VELOCIDADE_AJUSTE_RPM = 60;   // Velocidade constante em que o motor executará os passos
const float TOLERANCIA_CM = 0.1;        // Faixa de erro aceitável (em cm). Se o erro for menor, o motor não se move.
const int DELAY_ESTABILIZACAO_MS = 250; // Tempo de espera após um movimento para as vibrações mecânicas cessarem.

// ==============================================================================
// --- VARIÁVEIS DE TESTE E CONTROLE DE TEMPO ---
// ==============================================================================
const long INTERVALO_TROCA_MS = 30000; // Alterna o setpoint a cada 30 segundos para teste
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

  // Define a velocidade constante do motor que será usada nos movimentos
  myStepper.setSpeed(VELOCIDADE_AJUSTE_RPM);
  
  // --- Inicialização do sensor ---
  Serial.println("Inicializando sensor...");
  distanciaAtual_cm = lerDistanciaCm();
  // Garante que a primeira leitura seja válida antes de continuar
  while (distanciaAtual_cm <= 0 || distanciaAtual_cm > 400) {
    delay(100);
    distanciaAtual_cm = lerDistanciaCm();
  }
  Serial.print("Distancia inicial: ");
  Serial.println(distanciaAtual_cm);
  
  // Define o setpoint inicial
  setpoint_cm = 15.0;
  
  Serial.println("Sistema de Controle Posicional Direto Iniciado.");
}

void loop() {
  // --- LÓGICA DE CONTROLE POSICIONAL DIRETO ---

  // 1. LÊ a posição atual do sistema.
  distanciaAtual_cm = lerDistanciaCm();
  imprimirDados(); // Imprime o estado atual

  // 2. CALCULA o erro de posição.
  double erro_cm = setpoint_cm - distanciaAtual_cm;

  // 3. VERIFICA se a correção é necessária (se o erro está fora da tolerância).
  if (abs(erro_cm) > TOLERANCIA_CM) {
    // 4. CALCULA o número total de passos necessários para corrigir o erro.
    long passosParaMover = round(erro_cm * PASSOS_POR_CM);
    
    Serial.print(">>> Correcao necessaria! Erro: ");
    Serial.print(erro_cm, 2);
    Serial.print(" cm. Movendo ");
    Serial.print(passosParaMover);
    Serial.println(" passos.");

    // 5. EXECUTA o movimento completo. A função step() é BLOQUEANTE.
    // O código vai pausar aqui até que todos os passos sejam concluídos.
    myStepper.step(passosParaMover);

    // 6. ESPERA um momento para o sistema estabilizar mecanicamente.
    delay(DELAY_ESTABILIZACAO_MS);
  }

  // Após a verificação (e possível correção), o motor é desligado e o ciclo recomeça.
  desligarMotor();

  // --- Lógica para alternar o setpoint (para fins de teste) ---
  if (millis() - ultimoTempoTroca >= INTERVALO_TROCA_MS) {
    ultimoTempoTroca = millis();
    setpointAtualE15 = !setpointAtualE15;
    setpoint_cm = setpointAtualE15 ? 15.0 : 10.0;
    Serial.print("\n>>> NOVO SETPOINT DEFINIDO: ");
    Serial.println(setpoint_cm);
    Serial.println("-----------------------------------\n");
  }
}

// Função que lê o sensor e aplica o filtro de Kalman
float lerDistanciaCm() {
  float novaLeitura = lerDistanciaBruta();
  if (novaLeitura <= 0 || novaLeitura > 400) {
    return distanciaAtual_cm; 
  }
  return kalmanFilter.updateEstimate(novaLeitura);
}

// Função para obter uma única leitura do sensor ultrassônico
float lerDistanciaBruta() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  // CORREÇÃO: trocado PIN_G por PIN_TRIG para enviar o pulso ultrassônico.
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duracao = pulseIn(PIN_ECHO, HIGH, 25000); 
  return (duracao * 0.0343) / 2.0;
}

// Função para desligar as bobinas do motor, economizando energia e reduzindo o aquecimento
void desligarMotor() {
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}

// Função para imprimir os dados na Serial para monitoramento e plotagem
void imprimirDados() {
  static unsigned long ultimoPrintMs = 0;
  if (millis() - ultimoPrintMs > 200) { 
    ultimoPrintMs = millis();
    Serial.print("Setpoint: ");
    Serial.print(setpoint_cm, 2);
    Serial.print(" cm, Distancia Atual: ");
    Serial.println(distanciaAtual_cm, 2);
  }
}

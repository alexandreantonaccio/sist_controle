#include <Stepper.h>
#include <SimpleKalmanFilter.h> // Usado para suavizar a medição de posição

// =========================================================================
// CONTROLE POR ALOCAÇÃO DE POLOS (REALIMENTAÇÃO DE ESTADOS) NO ARDUINO
// =========================================================================
// Este código implementa o controlador projetado pelo script MATLAB de
// alocação de polos. Ele NÃO é um PID padrão.
//
// A lei de controle é:
// Saida = - (K1*x1 + K2*x2 + K3*x3) + Ki_sf * erro_integral
// Onde:
// x1 = Posição (medida)
// x2 = Velocidade (estimada)
// x3 = Aceleração (estimada)
// =========================================================================


// --- Configurações do Motor de Passo ---
const int PASSOS_MOTOR = 200;
Stepper myStepper(PASSOS_MOTOR, 4, 5, 6, 7);
const int maxRpm = 60;

// --- Configurações do Sensor Ultrassônico ---
const int PIN_TRIG = 9;
const int PIN_ECHO = 10;

// --- Filtro de Kalman para suavizar a medição de posição (x1) ---
SimpleKalmanFilter kalmanFilter(2.0, 2.0, 0.1);

// --- Variáveis de Controle ---
double setpoint;
double saidaControle; // Saída do controlador (RPM)

// --- GANHOS CALCULADOS PELO MATLAB (ALOCAÇÃO DE POLOS) ---
// Estes ganhos vêm diretamente do seu script MATLAB.
// K = [K1, K2, K3]
const double K[] = {0.2361, -3.2692, 1.4293};
const double Ki_sf = 0.0963; // Ganho integral (State Feedback)

// --- ESTADOS DO SISTEMA ---
// Precisamos estimar a velocidade e a aceleração a partir da posição.
double x1 = 0.0; // Estado 1: Posição (cm) - Medido e filtrado
double x2 = 0.0; // Estado 2: Velocidade (cm/s) - Estimado
double x3 = 0.0; // Estado 3: Aceleração (cm/s^2) - Estimado

// Variáveis auxiliares para a estimação
double x1_anterior = 0.0;
double x2_anterior = 0.0;
double erro_integral = 0.0;

// --- Parâmetros de Controle Adicionais ---
const float RPM_MINIMO = 0.5;

// --- Variáveis de Controle de Tempo ---
unsigned long ultimoTempoLeitura = 0;
const long INTERVALO_LEITURA_MS = 50; // 50ms = 20Hz (deve ser o mesmo do MATLAB)
const double INTERVALO_LEITURA_S = INTERVALO_LEITURA_MS / 1000.0;

// --- Variáveis para Controle de Passo Não-Bloqueante ---
unsigned long ultimoPassoMicros = 0;
long passoIntervaloMicros = 0;

// --- Declarações de Funções Auxiliares ---
float lerDistanciaBruta();
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
  
  x1 = kalmanFilter.updateEstimate(primeiraLeitura);
  x1_anterior = x1;
  
  Serial.print("Controlador por Alocacao de Polos iniciado. Posicao inicial: ");
  Serial.println(x1);
  
  setpoint = 15.0;
}

void loop() {
  // --- TAREFA 1: CONTROLE DO MOTOR (Executa o mais rápido possível) ---
  if (abs(saidaControle) > RPM_MINIMO) {
    passoIntervaloMicros = 60000000L / (PASSOS_MOTOR * abs(saidaControle));
    unsigned long agoraMicros = micros();
    if (agoraMicros - ultimoPassoMicros >= passoIntervaloMicros) {
      ultimoPassoMicros = agoraMicros;
      myStepper.step(saidaControle > 0 ? 1 : -1);
    }
  } else {
    desligarMotor();
  }

  // --- TAREFA 2: LEITURA, ESTIMAÇÃO E CÁLCULO (Executa em intervalos controlados) ---
  unsigned long agoraMillis = millis();
  if (agoraMillis - ultimoTempoLeitura >= INTERVALO_LEITURA_MS) {
    ultimoTempoLeitura = agoraMillis;

    // --- 1. Medir Posição (x1) ---
    float leituraBruta = lerDistanciaBruta();
    if (leituraBruta > 0 && leituraBruta < 400) {
      x1 = kalmanFilter.updateEstimate(leituraBruta);
    }
    // Se a leitura for inválida, x1 mantém seu valor anterior.

    // --- 2. Estimar Velocidade (x2) e Aceleração (x3) ---
    // Derivada numérica. Pode ser ruidosa, mas é a abordagem mais direta.
    x2 = (x1 - x1_anterior) / INTERVALO_LEITURA_S;
    x3 = (x2 - x2_anterior) / INTERVALO_LEITURA_S;

    // --- 3. Calcular Erro e sua Integral ---
    double erro = setpoint - x1;
    erro_integral += erro * INTERVALO_LEITURA_S;
    
    // Anti-windup simples: limita a integral para evitar que cresça indefinidamente
    erro_integral = constrain(erro_integral, -100.0, 100.0);

    // --- 4. APLICAR A LEI DE CONTROLE POR ALOCAÇÃO DE POLOS ---
    double u_feedback = -(K[0] * x1 + K[1] * x2 + K[2] * x3);
    double u_integral = Ki_sf * erro_integral;
    
    saidaControle = u_feedback + u_integral;

    // --- 5. Saturar a saída nos limites do motor ---
    saidaControle = constrain(saidaControle, -maxRpm, maxRpm);

    // --- 6. Atualizar variáveis para a próxima iteração ---
    x1_anterior = x1;
    x2_anterior = x2;

    // --- Impressão de dados ---
    imprimirDados();
  }
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

// Função para desligar as bobinas do motor
void desligarMotor() {
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}

// Função para imprimir os dados na Serial
void imprimirDados() {
  Serial.print("SP:");
  Serial.print(setpoint, 2);
  Serial.print(", x1(pos):");
  Serial.print(x1, 2);
  Serial.print(", x2(vel):");
  Serial.print(x2, 2);
  Serial.print(", x3(acel):");
  Serial.print(x3, 2);
  Serial.print(", RPM:");
  Serial.println(saidaControle, 2);
}
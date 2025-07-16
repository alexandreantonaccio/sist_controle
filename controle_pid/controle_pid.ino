#include <Stepper.h>
#include <PID_v1.h>

// --- Configurações do Motor ---
const int PASSOS_MOTOR = 200;
Stepper myStepper(PASSOS_MOTOR, 4, 5, 6, 7);
const int maxRpm = 100; // RPM máximo do motor

// --- Configurações do Sensor Ultrassônico ---
const int PIN_TRIG = 9;
const int PIN_ECHO = 10;
const int TAMANHO_MEDIA_MOVEL = 5; // Número de amostras para a média móvel
float leiturasDistancia[TAMANHO_MEDIA_MOVEL]; // Array para armazenar as leituras
int indiceLeitura = 0; // Índice atual do array

// --- Variáveis do PID ---
double setpoint;
double distanciaAtual;
double rpmSaida; 

// --- Constantes de Sintonia do PID ---
double Kp = 1.129;
double Ki = 0.414;
double Kd = 0.340;

const float ZONA_MORTA_CM = 0.5; // Zona morta para evitar oscilação

// --- Objeto PID ---
PID myPID(&distanciaAtual, &rpmSaida, &setpoint, Kp, Ki, Kd, DIRECT);

// --- VARIÁVEIS PARA ALTERNAR O SETPOINT ---
// ***** ALTERAÇÃO AQUI *****
const long INTERVALO_TROCA_MS = 30000; // Alterna a cada 30 segundos 
unsigned long ultimoTempoTroca = 0;     // Armazena o tempo da última troca
bool setpointAtualE15 = true;       


float lerDistanciaBruta(); // Declaração da função auxiliar

void setup() {
  Serial.begin(9600);
  
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  
  // --- Pré-aquecimento do filtro da média móvel ---
  Serial.println("Aquecendo sensor e filtro de media movel...");
  float primeiraLeitura = 0;
  while (primeiraLeitura <= 6 || primeiraLeitura > 20) {
    primeiraLeitura = lerDistanciaBruta(); // Lê um valor bruto inicial
    delay(50); // Espera um pouco entre tentativas
  }
  for (int i = 0; i < TAMANHO_MEDIA_MOVEL; i++) {
    leiturasDistancia[i] = primeiraLeitura;
  }
  distanciaAtual = primeiraLeitura; // Define a distância atual inicial
  Serial.println("Filtro inicializado.");
  
  Serial.println("Sistema de Controle PID Iniciado.");
  Serial.println("O setpoint alternara automaticamente entre 10cm e 15cm a cada 15 segundos.");

  // Define o setpoint inicial para 15cm
  setpoint = 15.0;

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20);
  myPID.SetOutputLimits(-maxRpm, maxRpm);
}

void loop() {
  // --- Lógica para alternar o setpoint automaticamente ---
  // Verifica se já se passaram 'INTERVALO_TROCA_MS' milissegundos desde a última troca
  if (millis() - ultimoTempoTroca >= INTERVALO_TROCA_MS) {
    ultimoTempoTroca = millis(); // Reseta o cronômetro

    if (setpointAtualE15) {
      // Se o setpoint era 15, muda para 10
      setpoint = 10.0;
      setpointAtualE15 = false;
    } else {
      // Se o setpoint era 10, muda para 15
      setpoint = 15.0;
      setpointAtualE15 = true;
    }
    Serial.print(">>> Setpoint alternado para: ");
    Serial.println(setpoint);
  }
  
  // A chamada da função permanece a mesma, mas agora ela retorna um valor filtrado
  distanciaAtual = lerDistanciaCm();
  double erro = setpoint - distanciaAtual;

  if (abs(erro) < ZONA_MORTA_CM) {
    rpmSaida = 0;
  } else {
    myPID.Compute();
  }
  
  moverMotorRpm(rpmSaida);
  imprimirDados();
}

void moverMotorRpm(float rpm) {
  if (abs(rpm) < 1.0) {
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    return;
  }
  
  long intervaloMicros = (60L * 1000000L) / (PASSOS_MOTOR * abs(rpm));
  static unsigned long ultimoPassoMicros = 0;

  if (micros() - ultimoPassoMicros >= intervaloMicros) {
    ultimoPassoMicros = micros();
    if (rpmSaida > 0) {
      myStepper.step(1);
    } else {
      myStepper.step(-1);
    }
  }
}

float lerDistanciaCm() {
  float novaLeitura = lerDistanciaBruta();

  if (novaLeitura <= 0 || novaLeitura > 400) {
    return distanciaAtual;
  }

  leiturasDistancia[indiceLeitura] = novaLeitura;
  indiceLeitura = (indiceLeitura + 1) % TAMANHO_MEDIA_MOVEL;

  float soma = 0;
  for (int i = 0; i < TAMANHO_MEDIA_MOVEL; i++) {
    soma += leiturasDistancia[i];
  }

  return soma / TAMANHO_MEDIA_MOVEL;
}

float lerDistanciaBruta() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duracao = pulseIn(PIN_ECHO, HIGH, 25000);
  return (duracao * 0.0343) / 2.0;
}

void imprimirDados() {
  static unsigned long ultimoPrintMs = 0;
  if (millis() - ultimoPrintMs > 250) {
    ultimoPrintMs = millis();
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" cm | Atual (filtrado): ");
    Serial.print(distanciaAtual, 2);
    Serial.print(" cm | Saida: ");
    Serial.print(rpmSaida);
    Serial.println(" RPM");
  }
}
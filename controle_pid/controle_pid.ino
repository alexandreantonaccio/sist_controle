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
double Kp = 10.0;
double Ki = 5.0;
double Kd = 10.0;

const float ZONA_MORTA_CM = 0.5; // Zona morta para evitar oscilação

// --- Objeto PID ---
PID myPID(&distanciaAtual, &rpmSaida, &setpoint, Kp, Ki, Kd, DIRECT);

float lerDistanciaBruta(); // Declaração da função auxiliar

void setup() {
  Serial.begin(9600);
  
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  
  // --- Pré-aquecimento do filtro da média móvel ---
  Serial.println("Aquecendo sensor e filtro de media movel...");
  float primeiraLeitura = 0;
  while (primeiraLeitura <= 0 || primeiraLeitura > 400) {
    primeiraLeitura = lerDistanciaBruta(); // Lê um valor bruto inicial
    delay(50); // Espera um pouco entre tentativas
  }
  for (int i = 0; i < TAMANHO_MEDIA_MOVEL; i++) {
    leiturasDistancia[i] = primeiraLeitura;
  }
  distanciaAtual = primeiraLeitura; // Define a distância atual inicial
  Serial.println("Filtro inicializado.");
  
  Serial.println("Sistema de Controle PID Iniciado.");
  Serial.println("Digite a distancia desejada (cm) no monitor serial e pressione Enter.");

  // Define um setpoint inicial
  setpoint = 14.0;

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20);
  myPID.SetOutputLimits(-maxRpm, maxRpm);
}

void loop() {
  if (Serial.available() > 0) {
    float novoSetpoint = Serial.parseFloat();
    if (novoSetpoint > 0 && novoSetpoint < 400) {
      setpoint = novoSetpoint;
      Serial.print(">>> Novo setpoint definido: ");
      Serial.println(setpoint);
    }
    while(Serial.available() > 0) { Serial.read(); }
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

/**
 * @brief Lê a distância do sensor e retorna uma média móvel para suavizar os valores.
 * @return A distância média móvel em centímetros.
 */
float lerDistanciaCm() {
  // 1. Obtém a leitura bruta do sensor
  float novaLeitura = lerDistanciaBruta();

  // 2. Valida a nova leitura. Se for inválida, ignora e retorna a última média.
  if (novaLeitura <= 0 || novaLeitura > 400) {
    return distanciaAtual; // Retorna a última média válida
  }

  // 3. Adiciona a nova leitura ao array
  leiturasDistancia[indiceLeitura] = novaLeitura;

  // 4. Atualiza o índice para a próxima leitura, de forma circular
  indiceLeitura = (indiceLeitura + 1) % TAMANHO_MEDIA_MOVEL;

  // 5. Calcula a soma de todas as leituras no array
  float soma = 0;
  for (int i = 0; i < TAMANHO_MEDIA_MOVEL; i++) {
    soma += leiturasDistancia[i];
  }

  // 6. Retorna a média
  return soma / TAMANHO_MEDIA_MOVEL;
}

/**
 * @brief Realiza a medição física da distância com o sensor HC-SR04.
 * @return A distância "bruta" medida em cm, sem filtro.
 */
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
    Serial.print(distanciaAtual, 2); // Imprime com 2 casas decimais
    Serial.print(" cm | Saida: ");
    Serial.print(rpmSaida);
    Serial.println(" RPM");
  }
}
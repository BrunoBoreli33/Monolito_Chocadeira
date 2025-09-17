#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ESP32Servo.h>

#include <Arduino.h>
#include <PID_v1.h>

#define endereco 0x27  // Verify address with I2C Scanner if needed
#define colunas 20
#define linhas 4

#define DHT22_1_PIN 5
#define DHT22_2_PIN 15
#define DHT_TYPE DHT22

#define BOMBA 32
#define FAN_EVAPORADOR 27
#define FAN_TROCA_DE_AR 26
#define ULTRASSONICO 33

#define Pino_sensorNivel 18

#define buzzer_signal 2

// Numero de opcoes de menu -> opcoes
#define NUM_OPCOES 2 

// Definições para o dimmer
#define TRIAC_PIN 4          // Pino para disparo do triac
#define ZERO_CROSS_PIN 13    // Pino para detecção do zero crossing
#define PULSE_WIDTH 30       // Duração do pulso em microssegundos

// Limites de potência do dimmer
#define DIMMER_MIN_VALUE 0   // Valor mínimo para o dimmer
#define DIMMER_MAX_VALUE 249   // Valor máximo para o dimmer

// Estrutura completa para o dimmer
typedef struct {
  // Parâmetros de configuração
  uint32_t halfCyclePeriod;  // Tempo do meio ciclo em microssegundos
  uint8_t filterSamples;     // Número de amostras para filtrar
  uint8_t smoothSteps;       // Passos para transição suave
  
  // Estado do sistema
  hw_timer_t *dimmerTimer;   // Timer para controle do disparo
  volatile bool timerRunning; // Flag para controlar o estado do timer
  volatile unsigned long lastZeroCrossTime;
  volatile int zeroCrossingCounter;
  
  // Valores de brilho
  uint8_t targetBrightness;  // Valor alvo (0-255)
  uint8_t currentBrightness; // Valor atual após suavização
  uint8_t *brightnessHistory; // Buffer para o histórico
  uint8_t historyIndex;
} DimmerControl;

// Instância global do controle de dimmer
DimmerControl dimmer;

unsigned long lastCenteringTime = 0;

//Sensor de Nível
bool sensor_nivel = false;
// 0 -> Sem Água
// 1 -> Tem Água

Servo myServo;

DHT dht22_1(DHT22_1_PIN, DHT_TYPE);
DHT dht22_2(DHT22_2_PIN, DHT_TYPE);

LiquidCrystal_I2C lcd(endereco, colunas, linhas);

// Buttons and sensors
int esq = 34, retroceder = 19, avancar = 25, okmenu = 23;
int tela = 0, section_menu = 0, buzzer_mode = 1;

// Variáveis para o controle PID
double setPoint = 37.7;      // Temperatura alvo (°C)

double output_power = 0.0;   // Saída do PID (0-255 para o dimmer)

// Parâmetros de ajuste do PID
double Kp = 260.0;  // Ganho proporcional
double Ki = 0.41;    // Ganho integral
double Kd = 19.0;   // Ganho derivativo

//SENSORES
float humidity1 = 0.0, humidity2 = 0.0, temperature1 = 0.0, temperature2 = 0.0;

double humidity_average = 0.0, temperature_average = 5.0;

// Instância do controlador PID
PID temperaturePID(&temperature_average, &output_power, &setPoint, Kp, Ki, Kd, DIRECT);

//pino do servo
const int servoPin = 14;

//versão assíncrona
int currentAngle = 0;
int targetAngle = 0;
int step = 0;
bool servoMoving = false;
unsigned long previousMillis = 0;
const int interval = 110; // Tempo entre cada incremento/decremento

// Configurações do servo (valores originais mantidos)
int pos_esquerda = 62; //meio + 17
int pos_direita = 36; //meio - 22
int pos_meio = 48;
const int stepDelay = 15;             // 6ms entre atualizações
const float smoothingFactor = 0.025;// Fator original

// Variáveis de estado
enum MotorState { STOPPED, MOVING };
MotorState motorState = STOPPED;
unsigned long lastCompletedMove = 0;
float currentPos = pos_meio;
int targetPos = pos_meio;

// Adicione após as outras variáveis globais
#define CALIB_TELA 6
int calib_pos_meio = pos_meio;
int calib_pos_direita = pos_direita;
int calib_pos_esquerda = pos_esquerda;
int calib_selected_param = 0;
bool calibrating = false;

static unsigned long falseStartTime = 0;   // Marca o início do período com condições falsas
static bool firstUpdateDone = false;         // Indica se já foi realizada a primeira atualização

unsigned long fanTimer = 0;
bool fan_troca_ar_mode = false;

//variáveis para controle do aviso sem água
int tela_anterior = 0;
bool alertaSemAguaAtivo = false;
unsigned long buzzerInterval = 0;
bool buzzerState = false;

int debug_set = 0;

bool last_control = false;

int TimerAtt_zero = 0;

int bomba_count = 10;

int esq_tela1_edMemoria = 0;

int controleT1T2_hora = 0;

int beep_menu = 1;

bool choca_ativa = false;

// Profile Control Variables
bool perf1_created = false;
bool perf2_created = false;
int current_editing_profile = 0; // 0: none, 1: PERF1, 2: PERF2
int current_selected_profile = 0; // Selected profile in menu
int submenu_option = 0; // Submenu option (Start, Edit, Delete)


// Variáveis globais para controle do timer
unsigned long timer_inicio_fase_inicial = 0;
unsigned long timer_inicio_fase_final = 0;
bool timer_fase_inicial_concluida = false;
bool timer_fase_final_concluida = false;

// Variáveis adicionais para controle do tempo
unsigned long tempo_restante_bloco1;
unsigned long tempo_restante_bloco2;
int dias, horas, minutos, segundos;
unsigned long ultimo_decremento = 0;


// Parameters for new incubation
float temp_target_I = 37.7, temp_target_I_last = 0, temp_target_I_perf1 = 0, temp_target_I_perf2 = 0;
float umid_target_I = 54.5, umid_target_I_last = 0, umid_target_I_perf1 = 0, umid_target_I_perf2 = 0;
float temp_target_F = 37.4, temp_target_F_last = 0, temp_target_F_perf1 = 0, temp_target_F_perf2 = 0;
float umid_target_F = 59.5, umid_target_F_last = 0, umid_target_F_perf1 = 0, umid_target_F_perf2 = 0;
int tempo_choca_I = 18, tempo_choca_I_last = 0, tempo_choca_I_perf1 = 0, tempo_choca_I_perf2 = 0;
int tempo_choca_F = 3, tempo_choca_F_last = 0, tempo_choca_F_perf1 = 0, tempo_choca_F_perf2 = 0;
int intervalo_viragem = 120, intervalo_viragem_last = 0, intervalo_viragem_perf1 = 0, intervalo_viragem_perf2 = 0;

int selected_param = 0, select_opcoes = 0;
bool editing_param = false;

// Function prototypes
void handle_buzzer();
void adjustValue(int delta);
void saveSettings();
void debug_ram();
void showStartupAnimation();
void saveMemory_Animation();
void noProfileAnimation();
void dimmerInit(uint8_t frequency, uint8_t filterSamples, uint8_t smoothSteps);
void updateDHTSensors();
void monitoraPID();


void setup() {

  Serial.begin(115200);
  Serial.println("\n\n===== SISTEMA DE CONTROLE DE TEMPERATURA COM PID =====");
  Serial.println("Sistema de Monitoramento Ativo");
  Serial.print("Parâmetros PID: Kp=");
  Serial.print(Kp);
  Serial.print(", Ki=");
  Serial.print(Ki);
  Serial.print(", Kd=");
  Serial.println(Kd);
  Serial.println("Monitoramento via Serial a cada 3 segundos");
  Serial.println("======================================================");

  dht22_1.begin();
  dht22_2.begin();

  temperaturePID.SetMode(AUTOMATIC);
  temperaturePID.SetOutputLimits(0, 255);  
  temperaturePID.SetSampleTime(501);

  // Inicializa o sistema de dimmer (60Hz, 3 amostras, 3 passos de suavização)
  dimmerInit(60, 1, 1);

  dimmerSetBrightness((DIMMER_MIN_VALUE + DIMMER_MAX_VALUE) / 2);

  myServo.attach(servoPin);
  myServo.write(pos_meio);

  //myServo.write(position1);

  Wire.begin(21, 22);  // SDA=GPIO21, SCL=GPIO22
  delay(100);

  lcd.begin(colunas, linhas);
  lcd.backlight();
  lcd.clear();

  pinMode(esq, INPUT);
  pinMode(retroceder, INPUT_PULLUP);
  pinMode(avancar, INPUT_PULLUP);
  pinMode(okmenu, INPUT_PULLUP);
  pinMode(buzzer_signal, OUTPUT);

  pinMode(BOMBA, OUTPUT);
  pinMode(FAN_EVAPORADOR, OUTPUT);
  pinMode(FAN_TROCA_DE_AR, OUTPUT);
  pinMode(ULTRASSONICO, OUTPUT);

  digitalWrite(BOMBA, HIGH);
  digitalWrite(FAN_EVAPORADOR, HIGH);
  digitalWrite(FAN_TROCA_DE_AR, HIGH);
  digitalWrite(ULTRASSONICO, HIGH);

  pinMode(Pino_sensorNivel, INPUT_PULLDOWN);

  // Initialization animation
  int linhaTexto = 1;
  int linhaBarra = 3;
  String mensagem = "INICIANDO";
  int colunaInicio = (colunas - mensagem.length()) / 2;
  
  byte progresso[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  lcd.createChar(0, progresso);

  for (int i = 0; i <= 100; i += 5) {
    lcd.setCursor(colunaInicio, linhaTexto);
    lcd.print(mensagem);
    for (int d = 0; d < (i / 25); d++) {
      lcd.print(".");
    }

    char spinner[] = {'|', '/', '-', '\\'};
    lcd.setCursor(colunaInicio + mensagem.length() + 3, linhaTexto);
    lcd.print(spinner[(i / 10) % 4]);

    lcd.setCursor(0, linhaBarra);
    lcd.print("[");
    int barraProgresso = map(i, 0, 100, 0, colunas - 2);
    for (int p = 0; p < colunas - 2; p++) {
      lcd.print(p <= barraProgresso ? (char)0 : ' ');
    }
    lcd.print("]");
    delay(100);
  }
  lcd.clear();
  Serial.println(choca_ativa);
}

void loop() {

  //digitalWrite(FAN_EVAPORADOR, LOW);
  //digitalWrite(ULTRASSONICO, LOW);

  // Monitoramento dos parâmetros via Serial
  monitoraPID();

  if(choca_ativa){
    timerFunction();
    Interface();
    handle_buttons();
    servo_function();
    updateServoMovement();
    displayDHTData();
    umidity_control();
    func_sensorNivel();
    Aquecimento_PID();

  }else{
    Interface();
    handle_buttons();
    servo_function();
    updateServoMovement();
    displayDHTData();
    umidity_control();
    func_sensorNivel();
    Aquecimento_PID();
  }
}

void Aquecimento_PID(){
  if (choca_ativa) { // Só executa o controle se choca_ativa for true
    if (temperaturePID.Compute()) {
      // Mapeia a saída do PID para a faixa do dimmer
      uint8_t mappedBrightness = map((int)constrain(output_power, 0, 255), 0, 255, DIMMER_MIN_VALUE, DIMMER_MAX_VALUE);
      dimmerSetBrightness(mappedBrightness);
    }
    
    // Aplica as atualizações ao dimmer (com suavização e filtragem)
    dimmerUpdate();
  } else {
    // Desliga o dimmer completamente quando não está ativo
    dimmerSetBrightness(0); // Ajuste para o valor mínimo de segurança
    dimmerUpdate(); // Garante que a mudança seja aplicada imediatamente
  }
}

void displayDHTData() {
  static unsigned long previousUpdate = 0;
  static float lastValidTemp1 = 0.0;
  static float lastValidHum1 = 0.0;
  static float lastValidTemp2 = 0.0;
  static float lastValidHum2 = 0.0;

  if (millis() - previousUpdate >= 3001) { // Intervalo aumentado para 2.2s
    previousUpdate = millis();

    // Lê e valida sensor 1
    float temp1 = dht22_1.readTemperature();
    float hum1  = dht22_1.readHumidity();
    if (!isnan(temp1) && !isnan(hum1)) {
      lastValidTemp1 = temp1;
      lastValidHum1 = hum1;
    } else {
      Serial.println("Erro na leitura do DHT22_1!");
    }

    // Lê e valida sensor 2
    float temp2 = dht22_2.readTemperature();
    float hum2  = dht22_2.readHumidity();
    if (!isnan(temp2) && !isnan(hum2)) {
      lastValidTemp2 = temp2;
      lastValidHum2 = hum2;
    } else {
      Serial.println("Erro na leitura do DHT22_2!");
    }

    // Atualiza variáveis mantendo último valor válido
    temperature1 = lastValidTemp1;
    humidity1 = lastValidHum1;
    temperature2 = lastValidTemp2;
    humidity2 = lastValidHum2;

    humidity_average = ((humidity2 + humidity1) / 2);
    temperature_average = temperature1; // Seu ajuste intencional
    Serial.println(humidity_average);
    Serial.println(humidity1);
    Serial.println(humidity2);
    Serial.println(temperature_average);
  }
}



// Função para o callback do timer
void IRAM_ATTR dimmerTimerISR() {
  digitalWrite(TRIAC_PIN, HIGH);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(TRIAC_PIN, LOW);
  dimmer.timerRunning = false;
  timerStop(dimmer.dimmerTimer);
}

// Função de interrupção para detecção de zero crossing
void IRAM_ATTR dimmerZeroCrossISR() {
  unsigned long currentTime = micros();
  unsigned long timeSinceLastZC = currentTime - dimmer.lastZeroCrossTime;
  
  // Filtra ruídos verificando se o tempo desde o último zero crossing é razoável
  if (timeSinceLastZC > (dimmer.halfCyclePeriod * 0.8)) {
    dimmer.lastZeroCrossTime = currentTime;
    dimmer.zeroCrossingCounter++;
    
    // Configura o timer para disparar no momento correto
    if (dimmer.currentBrightness > 5) { // Evita disparos para valores muito baixos
      int firingDelay = map((int)(255 - dimmer.currentBrightness), 0, 255, 0, (int)(dimmer.halfCyclePeriod - 300));
      
      // Certifique-se de que o timer não está em execução
      if (dimmer.timerRunning) {
        timerStop(dimmer.dimmerTimer);
        delayMicroseconds(5);
      }
      
      // Configurar e iniciar o timer
      timerWrite(dimmer.dimmerTimer, 0);
      timerAlarm(dimmer.dimmerTimer, firingDelay, false, 0);
      dimmer.timerRunning = true;
      timerStart(dimmer.dimmerTimer);
    }
  }
}

// Inicializa o sistema de controle de dimmer
void dimmerInit(uint8_t frequency = 60, uint8_t filterSamples = 5, uint8_t smoothSteps = 5) {
  // Configurar pinos
  pinMode(TRIAC_PIN, OUTPUT);
  digitalWrite(TRIAC_PIN, LOW);
  pinMode(ZERO_CROSS_PIN, INPUT_PULLUP);
  
  // Inicializar a estrutura de controle
  dimmer.halfCyclePeriod = (1000000 / frequency) / 2;
  dimmer.filterSamples = filterSamples;
  dimmer.smoothSteps = smoothSteps;
  dimmer.timerRunning = false;
  dimmer.lastZeroCrossTime = 0;
  dimmer.zeroCrossingCounter = 0;
  dimmer.targetBrightness = DIMMER_MIN_VALUE;
  dimmer.currentBrightness = DIMMER_MIN_VALUE;
  
  // Alocar o buffer de histórico
  dimmer.brightnessHistory = (uint8_t*)malloc(filterSamples * sizeof(uint8_t));
  for (int i = 0; i < filterSamples; i++) {
    dimmer.brightnessHistory[i] = dimmer.currentBrightness;
  }
  dimmer.historyIndex = 0;
  
  // Configurar o timer
  dimmer.dimmerTimer = timerBegin(1000000);
  timerAttachInterrupt(dimmer.dimmerTimer, &dimmerTimerISR);
  timerAlarm(dimmer.dimmerTimer, dimmer.halfCyclePeriod, false, 0);
  timerStop(dimmer.dimmerTimer);
  
  // Configurar a interrupção para zero crossing
  attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), dimmerZeroCrossISR, RISING);
}

// Define o brilho alvo do dimmer
void dimmerSetBrightness(uint8_t brightness) {
  // Limita o valor à faixa do dimmer
  dimmer.targetBrightness = constrain(brightness, DIMMER_MIN_VALUE, DIMMER_MAX_VALUE);
}

// Obtém o valor atual de brilho
uint8_t dimmerGetBrightness() {
  return dimmer.currentBrightness;
}

// Atualiza o estado do dimmer (com suavização e filtragem)
void dimmerUpdate() {
  // Transição suave entre o valor atual e o alvo
  if (dimmer.currentBrightness != dimmer.targetBrightness) {
    // Se a diferença for grande, mude mais rapidamente
    int diff = abs((int)dimmer.targetBrightness - (int)dimmer.currentBrightness);
    int step = max(1, diff / dimmer.smoothSteps);
    
    if (dimmer.currentBrightness < dimmer.targetBrightness) {
      // Conversão explícita para uint8_t
      uint8_t newBrightness = (uint8_t)min(255, (int)dimmer.currentBrightness + step);
      dimmer.currentBrightness = min(dimmer.targetBrightness, newBrightness);
    } else {
      // Prevenimos underflow com casting para int
      int newValue = max(0, (int)dimmer.currentBrightness - step);
      dimmer.currentBrightness = max(dimmer.targetBrightness, (uint8_t)newValue);
    }
  }
  
  // Média móvel para filtrar variações rápidas
  uint16_t sum = dimmer.currentBrightness;
  for (int i = 0; i < dimmer.filterSamples; i++) {
    sum += dimmer.brightnessHistory[i];
  }
  
  // Atualizamos o histórico após o cálculo
  dimmer.brightnessHistory[dimmer.historyIndex] = dimmer.currentBrightness;
  dimmer.historyIndex = (dimmer.historyIndex + 1) % dimmer.filterSamples;
  
  // A média é o resultado final
  dimmer.currentBrightness = sum / (dimmer.filterSamples + 1);
}

// Libera recursos alocados pelo dimmer
void dimmerCleanup() {
  if (dimmer.brightnessHistory) {
    free(dimmer.brightnessHistory);
    dimmer.brightnessHistory = NULL;
  }
  
  // Desativa o timer e interrupções
  timerStop(dimmer.dimmerTimer);
  timerAlarm(dimmer.dimmerTimer, 0, false, 0);
  detachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN));
}

// Monitoramento dos parâmetros PID, temperatura e umidade
void monitoraPID() {
  static unsigned long ultimoMonitoramento = 0;
  
  // Exibe informações a cada 3 segundos
  if (millis() - ultimoMonitoramento >= 3001) {
    ultimoMonitoramento = millis();
    
    // Parâmetros do PID
    Serial.println("---------- MONITORAMENTO ----------");
    Serial.print("PID: Kp=");
    Serial.print(Kp);
    Serial.print(" | Ki=");
    Serial.print(Ki);
    Serial.print(" | Kd=");
    Serial.println(Kd);
    
    // Temperatura dos sensores
    Serial.print("T1: ");
    Serial.print(temperature1, 2);
    Serial.print("°C | T2: ");
    Serial.print(temperature2, 2);
    Serial.print("°C | T-AVG: ");
    Serial.print(temperature_average, 2);
    Serial.print("°C | SetPoint: ");
    Serial.print(setPoint, 2);
    Serial.print("°C | Erro: ");
    Serial.print(setPoint - temperature_average, 2);
    Serial.println("°C");
    
    // Umidade dos sensores
    Serial.print("H1: ");
    Serial.print(humidity1, 2);
    Serial.print("% | H2: ");
    Serial.print(humidity2, 2);
    Serial.print("% | H-AVG: ");
    Serial.print(humidity_average, 2);
    Serial.println("%");
    
    // Saída do PID e valores do dimmer
    Serial.print("Saída PID: ");
    Serial.print(output_power);
    Serial.print(" | Brilho atual: ");
    Serial.print(dimmer.currentBrightness);
    Serial.print(" | Brilho alvo: ");
    Serial.println(dimmer.targetBrightness);
    
    // Estado da chocadeira
    Serial.print("Chocadeira ativa: ");
    Serial.println(choca_ativa ? "SIM" : "NÃO");
    Serial.println("----------------------------------");
  }
}

void Interface() {
  switch (tela) {
    case 0:
      displayMainMenu();
      break;
    case 1:
      displayIniciarChoca();
      break;
    case 2:
      displayOpcoes();
      break;
    case 3:
      displayPredefinicoes();
      break;
    case 4:
      displayErrors();
      break;
    case 5: 
      displayBuzzerSettings();
      break;
    case 6:
      displayCalibracao();
      break;
    case 7:
      displayFanSettings();
      break;
    case 8:
      displayChocaStarted();
      break;
    case 9:
      displayProfileOptions();
      break;
    case 10:
      displayChocaAtivaMenu();
      break;
    case 11:
      displaySemAgua();
      break;
    default:
      lcd.clear();
      lcd.print("Tela invalida!");
      break;
  }
}

void displayMainMenu() {
  if (section_menu == 3) {
    lcd.setCursor(0, 0);
    lcd.print(" <- --- MENU ---  ");
  } else if (section_menu < 3) {
    lcd.setCursor(0, 0);
    lcd.print("    --- MENU --- ->");
  }
  
  if (beep_menu == 1) {
    handle_buzzer();
    beep_menu = 0;
  }
  switch (section_menu) {
    case 0:
      lcd.setCursor(0, 1);
      lcd.print(choca_ativa ? "-> CHOCA ATIVA" : "-> NOVA CHOCAGEM");
      lcd.setCursor(0, 2);
      lcd.print("   OPCOES       ");
      lcd.setCursor(0, 3);
      lcd.print("   CHOCAGENS SALVAS");
      break;
    case 1:
      lcd.setCursor(0, 1);
      lcd.print(choca_ativa ? "   CHOCA ATIVA" : "   NOVA CHOCAGEM");
      lcd.setCursor(0, 2);
      lcd.print("-> OPCOES       ");
      lcd.setCursor(0, 3);
      lcd.print("   CHOCAGENS SALVAS");
      break;
    case 2:
      lcd.setCursor(0, 1);
      lcd.print(choca_ativa ? "   CHOCA ATIVA" : "   NOVA CHOCAGEM");
      lcd.setCursor(0, 2);
      lcd.print("   OPCOES       ");
      lcd.setCursor(0, 3);
      lcd.print("-> CHOCAGENS SALVAS");
      break;
    case 3:
      lcd.setCursor(0, 1);
      lcd.print("-> ERROS");
      break;
  }
}

void displayIniciarChoca() {
  switch (selected_param) {
    case 0: 
    case 1:
    case 2: {
      lcd.setCursor(0, 0);
      lcd.print("    CONFIG_PARAM ->");
      
      lcd.setCursor(0, 1);
      lcd.print(selected_param == 0 ? "->" : "  ");
      lcd.print("TempINI: ");
      lcd.print(temp_target_I, 1);
      lcd.print(" C  ");

      lcd.setCursor(0, 2);
      lcd.print(selected_param == 1 ? "->" : "  ");
      lcd.print("UmidINI: ");
      lcd.print(umid_target_I, 1);
      lcd.print("%  ");

      lcd.setCursor(0, 3);
      lcd.print(selected_param == 2 ? "->" : "  ");
      lcd.print("Viragem: ");
      
      int intervalo_int = (int)intervalo_viragem; 
      int horas = intervalo_int / 60;
      int minutos = intervalo_int % 60;
      lcd.print(horas);
      lcd.print(":");
      if (minutos < 10) {
        lcd.print("0");
      }
      lcd.print(minutos);
      lcd.print("h ");
      break;
    }
    case 3:
    case 4:
    case 5: {
      lcd.setCursor(0, 0);
      lcd.print(" <- CONFIG_PARAM ->");
      
      lcd.setCursor(0, 1);
      lcd.print(selected_param == 3 ? "->" : "  ");
      lcd.print("Dias_INI: ");
      lcd.print(tempo_choca_I);
      lcd.print(" D ");

      lcd.setCursor(0, 2);
      lcd.print(selected_param == 4 ? "->" : "  ");
      lcd.print("Temp_FIN: ");
      lcd.print(temp_target_F, 1);
      lcd.print(" C ");

      lcd.setCursor(0, 3);
      lcd.print(selected_param == 5 ? "->" : "  ");
      lcd.print("Umid_FIN: ");
      lcd.print(umid_target_F, 1);
      lcd.print("%  ");
      break;
    }
    case 6:
    case 7: {
      lcd.setCursor(0, 0);
      lcd.print(" <- CONFIG_PARAM  ");
      
      lcd.setCursor(0, 1);
      lcd.print(selected_param == 6 ? "->" : "  ");
      lcd.print("Dias_FIN: ");
      lcd.print(tempo_choca_F);
      lcd.print(" D ");

      lcd.setCursor(0, 3);
      lcd.print(selected_param == 7 ? "->" : "  ");
      if((choca_ativa) || (current_editing_profile > 0)){
        if(current_editing_profile == 1){
          lcd.print("SALVAR PERF1");
        }else if(current_editing_profile == 2){
          lcd.print("SALVAR PERF2");
        }else{
          lcd.print("SALVAR");      
        }
      } else{
        lcd.print("SALVAR E INICIAR");      
      }
      break;
    }
  }

  // Editing indicator
  if (editing_param) {
    int indicatorRow;
    if (selected_param < 3) {
      indicatorRow = selected_param + 1;
    } else if (selected_param < 6) {
      indicatorRow = selected_param - 2;
    } else if (selected_param == 6) {
      indicatorRow = selected_param - 5;
    }
    lcd.setCursor(19, indicatorRow);
    lcd.print("*");
  } else {
    for (int r = 0; r < 4; r++) {
      lcd.setCursor(19, r);
      lcd.print(" ");
    }
  }
}

void displayChocaAtivaMenu() {
  lcd.setCursor(0, 0);
  lcd.print(" --- CHOCA ATIVA ---");
  
  switch(section_menu) {
    case 0:
      lcd.setCursor(0, 1);
      lcd.print("-> STATUS CHOCA");
      lcd.setCursor(0, 2);
      lcd.print("   EDITAR CHOCA");
      lcd.setCursor(0, 3);
      lcd.print("   ENCERRAR CHOCA");
      break;
    case 1:
      lcd.setCursor(0, 1);
      lcd.print("   STATUS CHOCA");
      lcd.setCursor(0, 2);
      lcd.print("-> EDITAR CHOCA");
      lcd.setCursor(0, 3);
      lcd.print("   ENCERRAR CHOCA");
      break;
    case 2:
      lcd.setCursor(0, 1);
      lcd.print("   STATUS CHOCA");
      lcd.setCursor(0, 2);
      lcd.print("   EDITAR CHOCA");
      lcd.setCursor(0, 3);
      lcd.print("-> ENCERRAR CHOCA");
      break;
  }
}

void displayOpcoes() {
  lcd.setCursor(0, 0);
  lcd.print("   --- OPCOES ---");
  lcd.setCursor(0, 1);
  lcd.print(select_opcoes == 0 ? "-> BUZZER_BEEP" : "   BUZZER_BEEP");
  lcd.setCursor(0, 2);
  lcd.print(select_opcoes == 1 ? "-> CALIB_PLATAFORMA" : "   CALIB_PLATAFORMA"); // Nova opção
  lcd.setCursor(0, 3);
  lcd.print(select_opcoes == 2 ? "-> VENTILACAO" : "   VENTILACAO"); 
}

void displayFanSettings() {
  lcd.setCursor(0, 0);
  lcd.print("  -- VENTILACAO --");
  lcd.setCursor(0, 1);
  lcd.print(fan_troca_ar_mode ? "-> LIGADA         " : "-> DESLIGADA      ");
  lcd.setCursor(0, 3);
  lcd.print("Voltar (ESQ)");
}

void displayBuzzerSettings() {
  lcd.setCursor(0, 0);
  lcd.print("  -- BUZZER_BEEP --");
  lcd.setCursor(0, 1);
  lcd.print(buzzer_mode ? "-> ON         " : "-> OFF         ");
  lcd.setCursor(0, 3);
  lcd.print("Voltar (ESQ)");
}

void displaySemAgua() {
  lcd.setCursor(0, 0);
  lcd.print("  ---- AVISO! ----  ");
  lcd.setCursor(0, 1);
  lcd.print("    < SEM AGUA! >   ");
  lcd.setCursor(0, 3);
  lcd.print("  ABASTECA URGENTE! ");
}

void displayPredefinicoes() {
  lcd.setCursor(0, 0);
  lcd.print(" -CHOCAGENS SALVAS- ");
  switch (section_menu) {
    case 0:
      lcd.setCursor(0, 1);
      lcd.print("-> PERFIL_ULTIMO");
      lcd.setCursor(0, 2);
      lcd.print(perf1_created ? "   PERFIL 1" : "   CRIAR PERFIL 1");
      lcd.setCursor(0, 3);
      lcd.print(perf2_created ? "   PERFIL 2" : "   CRIAR PERFIL 2");
      break;
    case 1:
      lcd.setCursor(0, 1);
      lcd.print("   PERFIL_ULTIMO");
      lcd.setCursor(0, 2);
      lcd.print("-> " + String(perf1_created ? "PERFIL 1" : "CRIAR PERFIL 1"));
      lcd.setCursor(0, 3);
      lcd.print(perf2_created ? "   PERFIL 2" : "   CRIAR PERFIL 2");
      break;
    case 2:
      lcd.setCursor(0, 1);
      lcd.print("   PERFIL_ULTIMO");
      lcd.setCursor(0, 2);
      lcd.print(perf1_created ? "   PERFIL 1" : "   CRIAR PERFIL 1");
      lcd.setCursor(0, 3);
      lcd.print("-> " + String(perf2_created ? "PERFIL 2" : "CRIAR PERFIL 2"));
      break;
  }
}

void displayProfileOptions() {
  lcd.setCursor(0, 0);
  if(submenu_option >= 0){
    lcd.print("  OPCOES PERFIL ");
    lcd.print(current_selected_profile);
    lcd.print(":");
  }
  lcd.setCursor(0, 1);
  lcd.print(submenu_option == 0 ? "-> INICIAR" : "   INICIAR");
  lcd.setCursor(0, 2);
  lcd.print(submenu_option == 1 ? "-> EDITAR" : "   EDITAR");
  lcd.setCursor(0, 3);
  lcd.print(submenu_option == 2 ? "-> EXCLUIR" : "   EXCLUIR");
}

void displayErrors() {
  lcd.setCursor(0, 0);
  lcd.print("    --- ERROS ---   ");
  lcd.setCursor(0, 2);
  lcd.print("    << VAZIO >>   ");
}

void displayCalibracao() {
  lcd.setCursor(0, 0);
  lcd.print("  --- CALIBRAR ---  ");
  
  // Exibe o parâmetro "CENTRO" na linha 1
  lcd.setCursor(0, 1);
  lcd.print(calib_selected_param == 0 ? "->" : "  ");
  lcd.print("CENTRO <");
  lcd.print(calib_pos_meio);
  lcd.print(">");
  
  // Exibe o parâmetro "DIR." na linha 2
  lcd.setCursor(0, 2);
  lcd.print(calib_selected_param == 1 ? "->" : "  ");
  lcd.print("DIR.<");
  lcd.print(calib_pos_direita);
  lcd.print(">");
  
  // Exibe o parâmetro "ESQ." na linha 3 (parte 1)
  lcd.setCursor(0, 3);
  lcd.print(calib_selected_param == 2 ? "->" : "  ");
  lcd.print("ESQ.<");
  lcd.print(calib_pos_esquerda);
  lcd.print(">");
  
  // Exibe a opção "SALVAR" na linha 3 (parte 2, coluna 12)
  lcd.setCursor(12, 3);
  if (calib_selected_param == 3) {
    if (!editing_param)
      lcd.print("->SALVAR");
    else
      lcd.print("  SALVAR");
  } else {
    lcd.print("  SALVAR");
  }
  
  // Indicador de edição conforme a posição definida:
  // Para CENTRO: (15,1), para DIR: (12,2) e para ESQ ou SALVAR: (12,3)
  if (editing_param) {
    if (calib_selected_param == 0) {
      lcd.setCursor(14, 1);
      lcd.print("*");
    } else if (calib_selected_param == 1) {
      lcd.setCursor(11, 2);
      lcd.print("*");
    } else if (calib_selected_param == 2 || calib_selected_param == 3) {
      lcd.setCursor(11, 3);
      lcd.print("*");
    }
  } else {
    // Limpa os indicadores de edição nas posições definidas
    lcd.setCursor(14, 1);
    lcd.print(" ");
    lcd.setCursor(11, 2);
    lcd.print(" ");
    lcd.setCursor(11, 3);
    lcd.print(" ");
  }
}


void displayChocaStarted() {

  if (section_menu == 0) {
    lcd.setCursor(0, 0);
  lcd.print("   -STATUS CHOCA- ->");
  } else if (section_menu == 1) {
    lcd.setCursor(0, 0);
    lcd.print("<- -STATUS CHOCA-   ");
  }
  
  switch (section_menu) {
    case 0:
      // Buffer para formatar o tempo
      char time_buffer[20];

      // Limpa a linha 1 (assumindo 20 colunas)
      if((segundos == 0) && (TimerAtt_zero == 0)){
        lcd.setCursor(0, 1);
        lcd.print("                    ");
        TimerAtt_zero = 1;
      }
      
      lcd.setCursor(0, 1);

      if (controleT1T2_hora == 0) {
        lcd.print("T1:  ");
        sprintf(time_buffer, "%02dd %02dh %02dm %02ds", dias, horas, minutos, segundos);
      } else {
        lcd.print("T2:  ");  // Ajustado para ter o mesmo número de caracteres que "T1:  "
        sprintf(time_buffer, "%02dd %02dh %02dm %02ds", dias, horas, minutos, segundos);
      }
      lcd.print(time_buffer);

      lcd.setCursor(0, 2);
      lcd.print("T.A:");
      lcd.print(temperature_average, 1);
      lcd.print("C");
      

      if(controleT1T2_hora == 0){
        lcd.setCursor(10, 2);
        lcd.print(" T.F:");
        lcd.print(temp_target_I_last, 1);
        lcd.print("C");
        setPoint = temp_target_I_last;
      }else if(controleT1T2_hora == 1){
        lcd.setCursor(10, 2);
        lcd.print(" T.F:");
        lcd.print(temp_target_F_last, 1);
        lcd.print("C");
        setPoint = temp_target_F_last;
      }

      lcd.setCursor(0, 3);
      lcd.print("U.A:");
      lcd.print(humidity_average, 1);
      lcd.print("%");

      if(controleT1T2_hora == 0){
        lcd.setCursor(10, 3);
        lcd.print(" U.F:");
        lcd.print(umid_target_I_last, 1);
        lcd.print("%");
      }else if(controleT1T2_hora == 1){
        lcd.setCursor(10, 3);
        lcd.print(" U.F:");
        lcd.print(umid_target_F_last, 1);
        lcd.print("%");
      }
      break;
    case 1:

      lcd.setCursor(0, 3);
      lcd.print("TEMP1:");
      lcd.print(tempo_choca_I_last);
      lcd.print("D");

      lcd.setCursor(10, 3);
      lcd.print("  TEMP2:");
      lcd.print(tempo_choca_F_last);
      lcd.print("D");

      lcd.setCursor(0, 2);

      // Buffer para formatar o texto completo
      char rotacao_buffer[20]; 

      // Formata horas e minutos com dois dígitos cada
      sprintf(rotacao_buffer, "ROTACAO: %02d:%02dh", 
              intervalo_viragem_last / 60,    // Horas
              intervalo_viragem_last % 60);   // Minutos

      lcd.print(rotacao_buffer);
    
      break;
    
  }
}

void umidity_control() {
  static unsigned long lastCheck = 0;
  static unsigned long bombaStartTime = 0;
  static bool bombaActive = false;
  static unsigned long lastPumpActivation = 0; // Novo: controle do intervalo
  const unsigned long checkInterval = 8000;     // 8 segundos
  const unsigned long pumpInterval = 240000;    // 2 minutos (120,000 ms)
  const unsigned long bombaDuration = 20000;    // 20 segundos

  float targetUmid = (controleT1T2_hora == 0) ? umid_target_I_last : umid_target_F_last;

  if (!choca_ativa) {
    digitalWrite(FAN_EVAPORADOR, HIGH);
    digitalWrite(ULTRASSONICO, HIGH);
    if(fan_troca_ar_mode == true){
      digitalWrite(BOMBA, LOW);
    }
    else{
      digitalWrite(BOMBA, HIGH);
      bombaActive = false;
    }
    return;
  }

  if (millis() - lastCheck >= checkInterval) {
    lastCheck = millis();

    if (humidity_average < targetUmid) {
      // Ativar bomba a cada 2 minutos se houver água
      if (!bombaActive && (millis() - lastPumpActivation >= pumpInterval)) {
        if (sensor_nivel == 1) {
          bombaActive = true;
          bombaStartTime = millis();
          lastPumpActivation = millis();
          digitalWrite(BOMBA, LOW);
        }
      }
      // Manter ventilação ativa
      digitalWrite(FAN_EVAPORADOR, LOW);
      digitalWrite(ULTRASSONICO, LOW);
    } else {
      // Desligar tudo se umidade adequada
      digitalWrite(FAN_EVAPORADOR, HIGH);
      digitalWrite(ULTRASSONICO, HIGH);
      digitalWrite(BOMBA, HIGH);
      bombaActive = false;
    }
  }

  // Desligar bomba após 20 segundos
  if (bombaActive && (millis() - bombaStartTime >= bombaDuration)) {
    digitalWrite(BOMBA, HIGH);
    bombaActive = false;
  }
}

void func_sensorNivel() {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= 4000) {
    previousMillis = currentMillis;

    bool leitura_sensor = digitalRead(Pino_sensorNivel);
    sensor_nivel = leitura_sensor;

    Serial.print("Sensor Nivel: ");
    Serial.println(sensor_nivel);

    if (sensor_nivel == 0) { // Sem água
      if (!alertaSemAguaAtivo) {
        tela_anterior = tela; // Guarda a tela atual
        tela = 11; // Muda para tela de aviso
        alertaSemAguaAtivo = true;
        buzzerInterval = currentMillis; // Inicia contagem do buzzer
        buzzerState = true;
        lcd.clear();
      }
    } else { // Com água
      if (alertaSemAguaAtivo) {
        tela = tela_anterior; // Restaura a tela anterior
        alertaSemAguaAtivo = false;
        digitalWrite(buzzer_signal, LOW); // Garante desligar o buzzer
        lcd.clear();
      }
    }
  }

  // Controle do buzzer pulsante
  if (alertaSemAguaAtivo) {
    if (currentMillis - buzzerInterval >= 1000) {
      buzzerState = !buzzerState;
      digitalWrite(buzzer_signal, buzzerState ? HIGH : LOW);
      buzzerInterval = currentMillis;
    }
  }
}




void handle_buttons() {
  if (digitalRead(esq) == LOW) {
    // Verifica qual é a tela atual para determinar a ação
    if (tela == 9) { 
      // Submenu de perfis (Chocagens Salvas) -> Volta para Chocagens Salvas (tela 3)
      tela = 3;
      esq_tela1_edMemoria = 0;
    } else if (tela == 5) { 
      // Submenu de buzzer (Opções) -> Volta para Opções (tela 2)
      tela = 2;
    } else if ((tela == 1) && (esq_tela1_edMemoria == 1)){
      Serial.println(esq_tela1_edMemoria);
      tela = 9;
      esq_tela1_edMemoria = 0;
    } else if((tela == 0) && (choca_ativa)){
      tela = 8;
    } else if(tela == 6){
        tela = 2; // Volta para opções
        calibrating = false;
        calib_selected_param = 0;
        falseStartTime = 0;
        firstUpdateDone = false;

        if(!choca_ativa){
          // Move suavemente para a posição original
          moveServoSmoothlyAsync(currentPos, pos_meio);
        }
    } else if(tela == 7){
        tela = 2;
        digitalWrite(FAN_TROCA_DE_AR, HIGH);
        digitalWrite(BOMBA, HIGH); // Desliga a bomba ao sair
        fan_troca_ar_mode = false; // Atualiza a variável global
        section_menu = 0;
    } else if(tela == 11){
      return;
    } else { 
      // Demais casos -> Volta para o Menu Principal (tela 0)
      tela = 0;
    }
    

      // Resetar variáveis de controle
      section_menu = 0;
      editing_param = false;
      selected_param = 0;
      submenu_option = 0;
      current_editing_profile = 0;
      select_opcoes = 0;
      
      // Debounce e limpeza do display
      while (digitalRead(esq) == LOW);
      lcd.clear();
      handle_buzzer();
  }

  if (tela == 0) {
    if (digitalRead(avancar) == LOW) {
      section_menu = constrain(section_menu + 1, 0, 3);
      while (digitalRead(avancar) == LOW);
      lcd.clear();
      handle_buzzer();
    }
    if (digitalRead(retroceder) == LOW) {
      section_menu = constrain(section_menu - 1, 0, 3);
      while (digitalRead(retroceder) == LOW);
      lcd.clear();
      handle_buzzer();
    }
    if (digitalRead(okmenu) == LOW) {
      if (choca_ativa) { // Se estiver em CHOCA ATIVA
        switch (section_menu) {
        case 0: tela = 10; break;
        case 1: tela = 2; break;
        case 2: 
          MemChocaBlock_Animation(); 
          break;
        case 3: tela = 4; break;
      }
        while (digitalRead(okmenu) == LOW);
        lcd.clear();
        handle_buzzer();
        section_menu = 0;
      }
      else{
        switch (section_menu) {
        case 0: tela = 1; break;
        case 1: tela = 2; break;
        case 2: tela = 3; break;
        case 3: tela = 4; break;
      }
        while (digitalRead(okmenu) == LOW);
        lcd.clear();
        handle_buzzer();
        section_menu = 0;
      }
    }
  } else if (tela == 1) {
    if (!editing_param) {
      if (digitalRead(avancar) == LOW) {
        selected_param = (selected_param + 1) % 8;
        while (digitalRead(avancar) == LOW);
        lcd.clear();
        handle_buzzer();
      }
      if (digitalRead(retroceder) == LOW) {
        selected_param = (selected_param - 1);
        if (selected_param < 0) selected_param = 7;
        while (digitalRead(retroceder) == LOW);
        lcd.clear();
        handle_buzzer();
      }
      if (digitalRead(okmenu) == LOW) {
        if (current_editing_profile == 0) {
          if (selected_param == 7) {
            //adjustValue(1);
            //saveSettings();
            //handle_buzzer();

            if (choca_ativa) {
              debug_ram();
              last_control = true;
              selected_param = 0;
              handle_buzzer();
              saveSettings();
              lcd.clear();
              tela = 0;
            } else if (!choca_ativa){
              adjustValue(1);
              saveSettings();
              handle_buzzer();

              showStartupAnimation();
              debug_ram();
              last_control = true;
              selected_param = 0;
              tela = 8;
            }

          } else {
            editing_param = true;
            while (digitalRead(okmenu) == LOW);
            lcd.clear();
            handle_buzzer();
          }
        } else if (current_editing_profile == 1) {
          if (selected_param == 7) {
            handle_buzzer();
            saveMemory_Animation();
            saveSettings();
            debug_ram();
            selected_param = 0;
            tela = 3;
          }
          else {
            editing_param = true;
            while (digitalRead(okmenu) == LOW);
            lcd.clear();
            handle_buzzer();
          }
        } else if (current_editing_profile == 2) {
          if (selected_param == 7) {
            handle_buzzer();
            saveMemory_Animation();
            saveSettings();
            debug_ram();
            selected_param = 0;
            tela = 3;
          }
          else {
            editing_param = true;
            while (digitalRead(okmenu) == LOW);
            lcd.clear();
            handle_buzzer();
          }
        }
      }
    } else {
      if (digitalRead(avancar) == LOW) {
        adjustValue(1);
        while (digitalRead(avancar) == LOW);
        handle_buzzer();
      }
      if (digitalRead(retroceder) == LOW) {
        adjustValue(-1);
        while (digitalRead(retroceder) == LOW);
        handle_buzzer();
      }
      if (digitalRead(okmenu) == LOW) {
        editing_param = false;
        while (digitalRead(okmenu) == LOW);
        lcd.clear();
        handle_buzzer();
      }
    }
  } else if (tela == 2) {
    if (digitalRead(avancar) == LOW) {
      select_opcoes = (select_opcoes + 1);
      if(select_opcoes > 2){
        select_opcoes = 2;
      }
      while (digitalRead(avancar) == LOW);
      lcd.clear();
      handle_buzzer();
    }
    if (digitalRead(retroceder) == LOW) {
      select_opcoes = (select_opcoes - 1);
      if(select_opcoes < 0){
        select_opcoes = 0;
      }
      while (digitalRead(retroceder) == LOW);
      lcd.clear();
      handle_buzzer();
    }
    if (digitalRead(okmenu) == LOW) {
      if (select_opcoes == 0) tela = 5;
      if (select_opcoes == 1) { // Opção "CALIBRAR"
            // Ao entrar na tela de calibração, atualiza as variáveis de calibração
            calib_pos_meio = pos_meio;
            calib_pos_direita = pos_direita;
            calib_pos_esquerda = pos_esquerda;
            tela = 6;
        }
      if (select_opcoes == 2) tela = 7; // Nova tela para ventilação
      while (digitalRead(okmenu) == LOW);
      handle_buzzer();
      lcd.clear();
    }
  } else if (tela == 5) {
    if (digitalRead(avancar) == LOW || digitalRead(retroceder) == LOW) {
      buzzer_mode = !buzzer_mode;
      handle_buzzer();
      lcd.clear();
      while (digitalRead(avancar) == LOW || digitalRead(retroceder) == LOW);
    }
    if (digitalRead(esq) == LOW) {
      tela = 2;
      lcd.clear();
      while (digitalRead(esq) == LOW);
      handle_buzzer();
    }
  } else if (tela == 3) {
    if (digitalRead(avancar) == LOW) {
      section_menu = (section_menu + 1) % 3;
      while (digitalRead(avancar) == LOW);
      lcd.clear();
      handle_buzzer();
    }
    if (digitalRead(retroceder) == LOW) {
      section_menu = (section_menu - 1 + 3) % 3;
      while (digitalRead(retroceder) == LOW);
      lcd.clear();
      handle_buzzer();
    }
    if (digitalRead(okmenu) == LOW) {
      if (section_menu == 0) {
        if (!last_control) {
          noProfileAnimation();
        } else if (last_control) {
          temp_target_I_last = temp_target_I_last;
          umid_target_I_last = umid_target_I_last;
          temp_target_F_last = temp_target_F_last;
          umid_target_F_last = umid_target_F_last;
          tempo_choca_I_last = tempo_choca_I_last;
          tempo_choca_F_last = tempo_choca_F_last;
          intervalo_viragem_last = intervalo_viragem_last;
          choca_ativa = true;
          tela = 8;
          handle_buzzer();
          showStartupAnimation();
        }
      } else if (section_menu == 1) {
        current_selected_profile = 1;
        if (!perf1_created) {
          current_editing_profile = 1;
          //handle_buzzer();
          tela = 1;
        } else tela = 9;
      } else if (section_menu == 2) {
        current_selected_profile = 2;
        if (!perf2_created) {
          current_editing_profile = 2;
          //handle_buzzer();
          tela = 1;
        } else tela = 9;
      }
      while (digitalRead(okmenu) == LOW);
      lcd.clear();
      handle_buzzer();
    }
  } else if (tela == 9) {
    if (digitalRead(avancar) == LOW) {
      submenu_option = (submenu_option + 1) % 3;
      while (digitalRead(avancar) == LOW);
      lcd.clear();
      handle_buzzer();
    }
    if (digitalRead(retroceder) == LOW) {
      submenu_option = (submenu_option - 1 + 3) % 3;
      while (digitalRead(retroceder) == LOW);
      lcd.clear();
      handle_buzzer();
    }
    if (digitalRead(okmenu) == LOW) {
      if (submenu_option == 0) {
        if (current_selected_profile == 1) {
          temp_target_I_last = temp_target_I_perf1;
          umid_target_I_last = umid_target_I_perf1;
          temp_target_F_last = temp_target_F_perf1;
          umid_target_F_last = umid_target_F_perf1;
          tempo_choca_I_last = tempo_choca_I_perf1;
          tempo_choca_F_last = tempo_choca_F_perf1;
          intervalo_viragem_last = intervalo_viragem_perf1;
        } else if (current_selected_profile == 2) {
          temp_target_I_last = temp_target_I_perf2;
          umid_target_I_last = umid_target_I_perf2;
          temp_target_F_last = temp_target_F_perf2;
          umid_target_F_last = umid_target_F_perf2;
          tempo_choca_I_last = tempo_choca_I_perf2;
          tempo_choca_F_last = tempo_choca_F_perf2;
          intervalo_viragem_last = intervalo_viragem_perf2;
        }
        choca_ativa = true;
        tela = 8;
        handle_buzzer();
        showStartupAnimation();
      } else if (submenu_option == 1) {
        current_editing_profile = current_selected_profile;
        if (current_selected_profile == 1) {
          temp_target_I = temp_target_I_perf1;
          umid_target_I = umid_target_I_perf1;
          temp_target_F = temp_target_F_perf1;
          umid_target_F = umid_target_F_perf1;
          tempo_choca_I = tempo_choca_I_perf1;
          tempo_choca_F = tempo_choca_F_perf1;
          intervalo_viragem = intervalo_viragem_perf1;
        } else if (current_selected_profile == 2) {
          temp_target_I = temp_target_I_perf2;
          umid_target_I = umid_target_I_perf2;
          temp_target_F = temp_target_F_perf2;
          umid_target_F = umid_target_F_perf2;
          tempo_choca_I = tempo_choca_I_perf2;
          tempo_choca_F = tempo_choca_F_perf2;
          intervalo_viragem = intervalo_viragem_perf2;
        }
        debug_ram();
        esq_tela1_edMemoria = 1;
        tela = 1;
      } else if (submenu_option == 2) {
        if (current_selected_profile == 1) {
          perf1_created = false;
          temp_target_I_perf1 = 0;
          umid_target_I_perf1 = 0;
          temp_target_F_perf1 = 0;
          umid_target_F_perf1 = 0;
          tempo_choca_I_perf1 = 0;
          tempo_choca_F_perf1 = 0;
          intervalo_viragem_perf1 = 0;
          handle_buzzer();
          MemDelete_Animation();
          section_menu = 0;
          submenu_option = 0;
        } else if (current_selected_profile == 2) {
          perf2_created = false;
          temp_target_I_perf2 = 0;
          umid_target_I_perf2 = 0;
          temp_target_F_perf2 = 0;
          umid_target_F_perf2 = 0;
          tempo_choca_I_perf2 = 0;
          tempo_choca_F_perf2 = 0;
          intervalo_viragem_perf2 = 0;
          handle_buzzer();
          MemDelete_Animation();
          section_menu = 0;
          submenu_option = 0;
        }
        debug_ram();
        tela = 3;
      }
      while (digitalRead(okmenu) == LOW);
      lcd.clear();
      handle_buzzer();
    }
  }
  else if (tela == 10) {
    if (digitalRead(avancar) == LOW) {
      section_menu = constrain(section_menu + 1, 0, 2);
      while (digitalRead(avancar) == LOW);
      lcd.clear();
      handle_buzzer();
    }
    if (digitalRead(retroceder) == LOW) {
      section_menu = constrain(section_menu - 1, 0, 2);
      while (digitalRead(retroceder) == LOW);
      lcd.clear();
      handle_buzzer();
    }
    if (digitalRead(okmenu) == LOW) {
      switch (section_menu) {
        case 0: // STATUS CHOCA
          tela = 8;
          break;
        case 1: // EDITAR CHOCA

          temp_target_I = temp_target_I_last;
          umid_target_I = umid_target_I_last;
          temp_target_F = temp_target_F_last;
          umid_target_F = umid_target_F_last;
          tempo_choca_I = tempo_choca_I_last;
          tempo_choca_F = tempo_choca_F_last;
          intervalo_viragem = intervalo_viragem_last;

          current_editing_profile = 0;
          section_menu = 0;
          tela = 1;
           // Editando a choca atual
          break;
        case 2: // ENCERRAR CHOCA
          choca_ativa = false;
          section_menu = 0;
          tela = 0;
          firstUpdateDone = false;
          falseStartTime = 0;
          bomba_count = 10;
          break;
      }
      while (digitalRead(okmenu) == LOW);
      lcd.clear();
      handle_buzzer();
    }
  }
  else if (tela == 8) { // Tela de CHOCA ATIVA (Status em execução)
    if (digitalRead(avancar) == LOW) {
      section_menu = (section_menu + 1); // Alterna entre 0 e 1
      if(section_menu > 1){
        section_menu = 1;
      }
      while (digitalRead(avancar) == LOW);
      lcd.clear();
      handle_buzzer();
    }
    if (digitalRead(retroceder) == LOW) {
      section_menu = (section_menu - 1); // Alterna entre 0 e 1
      if(section_menu < 0){
        section_menu = 0;
      }
      while (digitalRead(retroceder) == LOW);
      lcd.clear();
      handle_buzzer();
    }
    if (digitalRead(okmenu) == LOW) {
      // Navega para o menu de controle da choca ativa
      tela = 10;
      section_menu = 0; // Reinicia a seleção do menu
      while (digitalRead(okmenu) == LOW);
      lcd.clear();
      handle_buzzer();
    }
  } else if (tela == 6) { 
    calibrating = true;
    falseStartTime = 0;
    firstUpdateDone = false;

    // Variáveis estáticas para controle de timing
    static unsigned long lastNavButtonPress = 0;
    static unsigned long lastEditButtonPress = 0;
    static bool avancarReleased = true;
    static bool retrocederReleased = true;
    static bool buttonHeld = false;
    const unsigned long debounceDelay = 50;
    const unsigned long repeatDelay = 300;    // Tempo antes de começar a repetição
    const unsigned long repeatInterval = 100;   // Intervalo entre repetições

    // Navegação entre parâmetros (modo não edição)
    if (!editing_param) {
        // Avançar
        if (digitalRead(avancar) == LOW) {
            if (avancarReleased && (millis() - lastNavButtonPress) > debounceDelay) {
                calib_selected_param = min(calib_selected_param + 1, 3);
                Serial.print("Navegação: calib_selected_param = ");
                Serial.println(calib_selected_param);
                lcd.clear();
                handle_buzzer();
                lastNavButtonPress = millis();
                avancarReleased = false; // Marca que o botão já foi acionado
            }
        } else {
            avancarReleased = true; // Botão liberado
        }
        // Retroceder
        if (digitalRead(retroceder) == LOW) {
            if (retrocederReleased && (millis() - lastNavButtonPress) > debounceDelay) {
                calib_selected_param = max(calib_selected_param - 1, 0);
                Serial.print("Navegação: calib_selected_param = ");
                Serial.println(calib_selected_param);
                lcd.clear();
                handle_buzzer();
                lastNavButtonPress = millis();
                retrocederReleased = false; // Marca que o botão já foi acionado
            }
        } else {
            retrocederReleased = true; // Botão liberado
        }
    }

    // Controle de edição/confirmação
    if (digitalRead(okmenu) == LOW) { 
    handle_buzzer();
    delay(20);

    if (editing_param) {
        // Se estiver em modo de edição, sai dele imediatamente,
        // independentemente do estado do servoMoving.
        editing_param = false;
        lcd.clear();
    } else if (!servoMoving) {
        // Se não estiver editando e o servo NÃO está se movendo, permite:
        if (calib_selected_param == 3) {
            // Caso de salvar os valores
            pos_meio = calib_pos_meio;
            pos_direita = calib_pos_direita;
            pos_esquerda = calib_pos_esquerda;
            
            if (!choca_ativa) { 
                moveServoSmoothlyAsync(currentPos, pos_meio);
            }

            Serial.println("Valores salvos!");
            savePlatform_Animation();
            tela = 2;
            select_opcoes = 0;
            calibrating = false; // Libera o controle do servo
            calib_selected_param = 0;
            falseStartTime = 0;
            firstUpdateDone = false;
            
            // Mantém a posição atual se não estiver em operação
            if (!choca_ativa) {
                currentPos = pos_meio;
                targetPos = pos_meio;
            }
        } else {
            // Ativa o modo de edição (para CENTRO, DIR. ou ESQ.)
            editing_param = true;
            if (calib_selected_param == 0) {
                lcd.setCursor(14, 1);
                lcd.print("*");
            } else if (calib_selected_param == 1) {
                lcd.setCursor(11, 2);
                lcd.print("*");
            } else if (calib_selected_param == 2) {
                lcd.setCursor(11, 3);
                lcd.print("*");
            }
            
            // Move o servo para o valor correspondente à opção selecionada
            int targetAngle;
            if (calib_selected_param == 0)
                targetAngle = calib_pos_meio;
            else if (calib_selected_param == 1)
                targetAngle = calib_pos_direita;
            else if (calib_selected_param == 2)
                targetAngle = calib_pos_esquerda;
            
            moveServoSmoothlyAsync(currentPos, targetAngle);
            currentPos = targetAngle;
        }
    }
    // Aguarda o botão ser liberado para evitar múltiplas execuções
    while (digitalRead(okmenu) == LOW);
}

    // Edição de valores (com repetição contínua)
    if (editing_param && calib_selected_param < 3 && !servoMoving) {
        int delta = 0;
        if (digitalRead(avancar) == LOW) delta = 1;
        else if (digitalRead(retroceder) == LOW) delta = -1;

        if (delta != 0) {
            unsigned long currentTime = millis();
              
            // Primeira pressão ou tempo suficiente para repetição
            if (!buttonHeld || (currentTime - lastEditButtonPress) > repeatDelay) {
                if (!buttonHeld) {
                    // Primeira alteração imediata
                    buttonHeld = true;
                    lastEditButtonPress = currentTime;
                } else {
                    // Repetição após o intervalo
                    lastEditButtonPress = currentTime - (repeatDelay - repeatInterval);
                }

                // Aplicar mudança conforme o parâmetro selecionado
                switch(calib_selected_param) {
                    case 0: 
                        calib_pos_meio = constrain(calib_pos_meio + delta, 0, 180);
                        myServo.write(calib_pos_meio);
                        currentPos = calib_pos_meio; // Atualiza currentPos
                        break;
                    case 1: 
                        calib_pos_direita = constrain(calib_pos_direita + delta, 0, 180);
                        myServo.write(calib_pos_direita);
                        currentPos = calib_pos_direita; // Atualiza currentPos
                        break;
                    case 2: 
                        calib_pos_esquerda = constrain(calib_pos_esquerda + delta, 0, 180);
                        myServo.write(calib_pos_esquerda);
                        currentPos = calib_pos_esquerda; // Atualiza currentPos
                        break;
                }
                lcd.clear();
                handle_buzzer();
            }
        } else {
            buttonHeld = false;
        }
    }
  }
      else if (tela == 7) { 
      // Verifica se o ventilador está ligado e se 1 minuto se passou
      if (fan_troca_ar_mode && (millis() - fanTimer >= 60000)) {
          // Desliga o ventilador automaticamente após 1 minuto
          fan_troca_ar_mode = false;
          digitalWrite(FAN_TROCA_DE_AR, HIGH);
          digitalWrite(BOMBA, HIGH); // Desliga a bomba automaticamente
          handle_buzzer();
          lcd.clear();
          // Opcional: reinicia o timer, se necessário
      }
      
      if (digitalRead(avancar) == LOW || digitalRead(retroceder) == LOW) {
        // Alterna o estado do ventilador
        fan_troca_ar_mode = !fan_troca_ar_mode;
        digitalWrite(FAN_TROCA_DE_AR, fan_troca_ar_mode ? LOW : HIGH); // Atualiza o estado do ventilador
        digitalWrite(BOMBA, fan_troca_ar_mode ? LOW : HIGH); // Nova linha para a bomba
        handle_buzzer();
        lcd.clear();
        // Aguarda a liberação dos botões
        while (digitalRead(avancar) == LOW || digitalRead(retroceder) == LOW);
        
        // Se o ventilador for ligado, reinicia o timer
        if (fan_troca_ar_mode) {
            fanTimer = millis();
        }
      }
  }
}

void moveServoSmoothlyAsync(int startAngle, int endAngle) {
    if (startAngle == endAngle) return; // Evita movimentos desnecessários

    currentAngle = startAngle;
    targetAngle = endAngle;
    step = (startAngle < endAngle) ? 1 : -1;
    myServo.write(currentAngle);
    servoMoving = true; // Bloqueia os botões
}

void updateServoMovement() {
    if (!servoMoving) return; // Só processa se estiver em movimento

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        currentAngle += step;
        myServo.write(currentAngle);
        
        if (currentAngle == targetAngle) {
            step = 0; 
            servoMoving = false; // Libera os botões ao finalizar o movimento
        }
    }
}

void debug_ram() {
  Serial.println("------------------------------------------------------");
  Serial.print("temp_target_I: ");
  Serial.println(temp_target_I);
  Serial.print("umid_target_I: ");
  Serial.println(umid_target_I);
  Serial.print("temp_target_F: ");
  Serial.println(temp_target_F);
  Serial.print("umid_target_F: ");
  Serial.println(umid_target_F);
  Serial.print("tempo_choca_I: ");
  Serial.println(tempo_choca_I);
  Serial.print("tempo_choca_F: ");
  Serial.println(tempo_choca_F);
  Serial.print("intervalo_viragem: ");
  Serial.println(intervalo_viragem);
  Serial.println("-------------------------------");
  Serial.print("temp_target_I_last: ");
  Serial.println(temp_target_I_last);
  Serial.print("umid_target_I_last: ");
  Serial.println(umid_target_I_last);
  Serial.print("temp_target_F_last: ");
  Serial.println(temp_target_F_last);
  Serial.print("umid_target_F_last: ");
  Serial.println(umid_target_F_last);
  Serial.print("tempo_choca_I_last: ");
  Serial.println(tempo_choca_I_last);
  Serial.print("tempo_choca_F_last: ");
  Serial.println(tempo_choca_F_last);
  Serial.print("intervalo_viragem_last: ");
  Serial.println(intervalo_viragem_last);
  Serial.println("-------------------------------");
  Serial.print("temp_target_I_perf1: ");
  Serial.println(temp_target_I_perf1);
  Serial.print("umid_target_I_perf1: ");
  Serial.println(umid_target_I_perf1);
  Serial.print("temp_target_F_perf1: ");
  Serial.println(temp_target_F_perf1);
  Serial.print("umid_target_F_perf1: ");
  Serial.println(umid_target_F_perf1);
  Serial.print("tempo_choca_I_perf1: ");
  Serial.println(tempo_choca_I_perf1);
  Serial.print("tempo_choca_F_perf1: ");
  Serial.println(tempo_choca_F_perf1);
  Serial.print("intervalo_viragem_perf1: ");
  Serial.println(intervalo_viragem_perf1);
  Serial.println("-------------------------------");
  Serial.print("temp_target_I_perf2: ");
  Serial.println(temp_target_I_perf2);
  Serial.print("umid_target_I_perf2: ");
  Serial.println(umid_target_I_perf2);
  Serial.print("temp_target_F_perf2: ");
  Serial.println(temp_target_F_perf2);
  Serial.print("umid_target_F_perf2: ");
  Serial.println(umid_target_F_perf2);
  Serial.print("tempo_choca_I_perf2: ");
  Serial.println(tempo_choca_I_perf2);
  Serial.print("tempo_choca_F_perf2: ");
  Serial.println(tempo_choca_F_perf2);
  Serial.print("intervalo_viragem_perf2: ");
  Serial.println(intervalo_viragem_perf2);
  Serial.print("Estado de Choca: ");
  Serial.println(choca_ativa);
}


  //CONTROLE DO SERVO MOTOR

  //const int pos_esquerda = 54;     // Ajuste para valores mais extremos
  //const int pos_direita = 16;    // se necessário
  //const int pos_meio = 30;

  //pos_meio -22 = pos_direita
  //pos_meio +16 = pos_esquerda

  void servo_function() {
    if(calibrating) return;

    // Modifique o controle do retorno ao centro
    if (!choca_ativa) { // <-- ADICIONE ESTA CONDIÇÃO
        if (currentPos != pos_meio) {
            targetPos = pos_meio;
            motorState = MOVING;
        }
    }
    else {
        // Restante da lógica normal quando choca_ativa == true
        if (motorState == STOPPED && (millis() - lastCompletedMove >= intervalo_viragem_last * 60 * 1000)) {
            targetPos = (targetPos == pos_direita) ? pos_esquerda : pos_direita;
            motorState = MOVING;
            lastCompletedMove = millis();
        }
    }

  // Animação de movimento (executa para ambos os casos)
  if (motorState == MOVING) {
    static unsigned long lastStep = 0;
    
    if (millis() - lastStep >= stepDelay) {
      currentPos += (targetPos - currentPos) * smoothingFactor;
      myServo.write((int)currentPos);
      
      if (abs(currentPos - targetPos) < 0.1) {
        currentPos = targetPos;
        motorState = STOPPED;
        Serial.print("Movimento ");
        Serial.print(!choca_ativa ? "de retorno " : "");
        Serial.println("completo às " + String(millis()/1000 * 60) + "min");
        handle_buzzer();
      }
      lastStep = millis();
    }
  }
}

//void smoothMovementAnimation(int target) {
//  static unsigned long lastStep = 0;
//  
//  if (millis() - lastStep >= stepDelay) {
//    // Cálculo da nova posição com suavização
//    currentPos += (target - currentPos) * smoothingFactor;
//    
//    // Escrever no servo
//    myServo.write((int)currentPos);
//    
//    // Verificar conclusão do movimento
//   if (abs(currentPos - target) < 0.5) {
//      currentPos = target;
//      motorState = STOPPED;
//      Serial.println("Movimento completo");
//    }
//    
//    lastStep = millis();
//  }
//}


void converterTempo(unsigned long segundos_totais) {
  dias = segundos_totais / 86400L;
  unsigned long resto = segundos_totais % 86400L;
  horas = resto / 3600;
  resto = resto % 3600;
  minutos = resto / 60;
  segundos = resto % 60;
}


void timerFunction() {
  if (!choca_ativa) {
    timer_inicio_fase_inicial = 0;
    timer_inicio_fase_final = 0;
    timer_fase_inicial_concluida = false;
    timer_fase_final_concluida = false;
    return;
  }

  if (millis() - ultimo_decremento >= 1000) {
    ultimo_decremento = millis();

    if (!timer_fase_inicial_concluida) {
      // Cálculo seguro do tempo restante para evitar underflow
      unsigned long elapsed = (millis() - timer_inicio_fase_inicial) / 1000;
      if (elapsed >= tempo_choca_I_last * 86400L) {
        tempo_restante_bloco1 = 0;
      } else {
        tempo_restante_bloco1 = (tempo_choca_I_last * 86400L) - elapsed;
      }
      converterTempo(tempo_restante_bloco1);

      controleT1T2_hora = 0;
      TimerAtt_zero = 0;

      //Serial.println("Fase 1 faltando: ");
      //Serial.println(String(dias) + "d " + String(horas) + "h " + String(minutos) + "m " + String(segundos) + "s");

      if (tempo_restante_bloco1 == 0) {
        timer_fase_inicial_concluida = true;
        timer_inicio_fase_final = millis();
      }
    } 
    else if (!timer_fase_final_concluida) {
      // Cálculo seguro do tempo restante para evitar underflow
      unsigned long elapsed = (millis() - timer_inicio_fase_final) / 1000;
      if (elapsed >= tempo_choca_F_last * 86400L) {
        tempo_restante_bloco2 = 0;
      } else {
        tempo_restante_bloco2 = (tempo_choca_F_last * 86400L) - elapsed;
      }
      converterTempo(tempo_restante_bloco2);

      controleT1T2_hora = 1;
      TimerAtt_zero = 0;

      //Serial.println("Fase 2 faltando: ");
      //Serial.println(String(dias) + "d " + String(horas) + "h " + String(minutos) + "m " + String(segundos) + "s");

      if (tempo_restante_bloco2 == 0) {
        timer_fase_final_concluida = true;
      }
    }
    else {
      // BLOCO FINAL - Aguardar confirmação
      Serial.println("CHOCA FINALIZADA!");
      choca_ativa = false;
      temp_target_I_last = 0;
      animation_terminoChoca(); // Chamar a animação aqui
      falseStartTime = 0;
      firstUpdateDone = false;
      section_menu = 0;
      tela = 0;
      lcd.clear();
}
  }
}


void handle_buzzer() {
  if (buzzer_mode == 1) {
    digitalWrite(buzzer_signal, HIGH);
    delay(10);
    digitalWrite(buzzer_signal, LOW);
  }
}


void adjustValue(int delta) {
  switch(selected_param) {
    case 0: // TempINI
      temp_target_I += delta * 0.1;
      temp_target_I = constrain(temp_target_I, 0.0, 42.0);
      break;
    case 1: // UmidINI
      umid_target_I += delta * 0.5;
      umid_target_I = constrain(umid_target_I, 0.0, 80.0);
      break;
    case 2: // Intervalo
      intervalo_viragem += delta * 30;
      intervalo_viragem = constrain(intervalo_viragem, 0, 360);
      break;
    case 3: // Dias_INI
      tempo_choca_I += delta;
      tempo_choca_I = constrain(tempo_choca_I, 0, 30);
      break;
    case 4: // Temp_FIN
      temp_target_F += delta * 0.1;
      temp_target_F = constrain(temp_target_F, 0.0, 42.0);
      break;
    case 5: // Umid_FIN
      umid_target_F += delta * 0.5;
      umid_target_F = constrain(umid_target_F, 0.0, 80.0);
      break;
    case 6: // Dias_FIN
      tempo_choca_F += delta;
      tempo_choca_F = constrain(tempo_choca_F, 0, 30);
      break;
    case 7: // Save and start
      choca_ativa = true;
      break;
  }
}

void saveSettings() {
  Serial.print("current_editing_profile dentro de saveSettings:");
  Serial.println(current_editing_profile);
  if (current_editing_profile == 0) {
    temp_target_I_last = temp_target_I;
    umid_target_I_last = umid_target_I;
    temp_target_F_last = temp_target_F;
    umid_target_F_last = umid_target_F;
    tempo_choca_I_last = tempo_choca_I;
    tempo_choca_F_last = tempo_choca_F;
    intervalo_viragem_last = intervalo_viragem;
    Serial.println("Salvo em Last !");
  } else if (current_editing_profile == 1) {
    temp_target_I_perf1 = temp_target_I;
    umid_target_I_perf1 = umid_target_I;
    temp_target_F_perf1 = temp_target_F;
    umid_target_F_perf1 = umid_target_F;
    tempo_choca_I_perf1 = tempo_choca_I;
    tempo_choca_F_perf1 = tempo_choca_F;
    intervalo_viragem_perf1 = intervalo_viragem;
    perf1_created = true;
    Serial.println("Salvo em Perf 1!");
  } else if (current_editing_profile == 2) {
    temp_target_I_perf2 = temp_target_I;
    umid_target_I_perf2 = umid_target_I;
    temp_target_F_perf2 = temp_target_F;
    umid_target_F_perf2 = umid_target_F;
    tempo_choca_I_perf2 = tempo_choca_I;
    tempo_choca_F_perf2 = tempo_choca_F;
    intervalo_viragem_perf2 = intervalo_viragem;
    perf2_created = true;
    Serial.println("Salvo em Perf 2!");
  }
  current_editing_profile = 0;
}

void animation_terminoChoca() {
  bool exitAnimation = false;
  unsigned long startTime;
  bool messageDisplayed = true;

  while (!exitAnimation) {
    if (messageDisplayed) {
      // Mostrar mensagem e ligar o buzzer
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("  ---- AVISO! ----  ");
      lcd.setCursor(0, 2);
      lcd.print("  CHOCA FINALIZADA!");
      lcd.setCursor(0, 3);
      lcd.print("       < OK >       ");
      if (buzzer_mode == 1) {
        digitalWrite(buzzer_signal, HIGH);
      }
      startTime = millis();
      while (millis() - startTime < 1000) {
        if (digitalRead(okmenu) == LOW) {
          exitAnimation = true;
          break;
        }
        delay(50);
      }
      messageDisplayed = false;
      digitalWrite(buzzer_signal, LOW);
    } else {
      // Limpar tela e desligar o buzzer
      lcd.clear();
      startTime = millis();
      while (millis() - startTime < 1000) {
        if (digitalRead(okmenu) == LOW) {
          exitAnimation = true;
          break;
        }
        delay(50);
      }
      messageDisplayed = true;
    }
    // Verificação adicional para sair
    if (digitalRead(okmenu) == LOW) {
      exitAnimation = true;
      while (digitalRead(okmenu) == LOW); // Espera soltar o botão
    }
  }
  digitalWrite(buzzer_signal, LOW);
  lcd.clear();
}


void saveMemory_Animation() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  ---- AVISO! ----  ");
  if (current_editing_profile == 1) {
    lcd.setCursor(0, 2);
    lcd.print("  PERFIL 1 SALVO!");
  } else if (current_editing_profile == 2) {
    lcd.setCursor(0, 2);
    lcd.print("  PERFIL 2 SALVO!");
  }
  delay(1000);
  handle_buzzer();
  lcd.clear();
}

void savePlatform_Animation() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  ---- AVISO! ----  ");
  
  lcd.setCursor(0, 2);
  lcd.print("   CONFIG. SALVA!   ");
  
  delay(1000);
  handle_buzzer();
  lcd.clear();
}

void MemChocaBlock_Animation() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  ---- AVISO! ----  ");
  lcd.setCursor(0,1);
  lcd.print("  < CHOCA ATIVA! > ");
  lcd.setCursor(0,3);
  lcd.print(" MEMORIA BLOQUEADA!");
  handle_buzzer();
  delay(1000);
  lcd.clear();
}

void MemDelete_Animation() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  ---- AVISO! ----  ");
  if (current_selected_profile == 1) {
    lcd.setCursor(0, 2);
    lcd.print(" PERFIL 1 DELETADO!");
  } else if (current_selected_profile == 2) {
    lcd.setCursor(0, 2);
    lcd.print(" PERFIL 2 DELETADO!");
  }
  delay(1000);
  lcd.clear();
}


void noProfileAnimation() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  ---- AVISO! ----  ");
  lcd.setCursor(1, 2);
  lcd.print("DADOS INEXISTENTES!");
  handle_buzzer();
  delay(1000);
  lcd.clear();
}

void showStartupAnimation() {
  lcd.clear();
  int colMsg = (colunas - String("CONFIG CARREGADA!").length()) / 2;
  lcd.setCursor(colMsg, 1);
  lcd.print("CONFIG CARREGADA!");
  delay(1000);
  lcd.clear();

  String mensagem = "INICIANDO CHOCA.";
  int colunaInicio = (colunas - mensagem.length()) / 2;
  byte progresso[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  lcd.createChar(0, progresso);

  for (int i = 0; i <= 100; i += 5) {
    lcd.setCursor(colunaInicio, 1);
    lcd.print(mensagem);
    for (int d = 0; d < (i / 25); d++) {
      lcd.print(".");
    }

    char spinner[] = {'|', '/', '-', '\\'};
    lcd.setCursor(colunaInicio + mensagem.length() + 3, 1);
    lcd.print(spinner[(i / 10) % 4]);

    lcd.setCursor(0, 3);
    lcd.print("[");
    int barraProgresso = map(i, 0, 100, 0, colunas - 2);
    for (int p = 0; p < colunas - 2; p++) {
      lcd.print(p <= barraProgresso ? (char)0 : ' ');
    }
    lcd.print("]");
    delay(100);
  }

  // REINICIA OS TIMERS QUANDO INICIA NOVA CHOCAGEM
  timer_inicio_fase_inicial = millis();
  timer_fase_inicial_concluida = false;
  timer_fase_final_concluida = false;
  section_menu == 0;

  handle_buzzer();
  lcd.clear();
}
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "esp_system.h"

// ── Configurações ──────────────────────────────────────────────
const char* WIFI_SSID     = "SF_VISITANTES";
const char* WIFI_PASSWORD = "";

// ── Supabase ───────────────────────────────────────────────────
const char* SUPABASE_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InhudGhtdnRqeGNhbnh4YmdydnNkIiwicm9sZSI6ImFub24iLCJpYXQiOjE3ODIxNDQyMDcsImV4cCI6MjA5NzcyMDIwN30.NYHuFSM8a0fgNbcL380tSIEkQyVvZvRCyqr0Hij5o8Q";
const char* SUPABASE_RPC_URL = "https://xnthmvtjxcanxxbgrvsd.supabase.co/rest/v1/rpc/inserir_leitura_via_serial";

String montarServerUrl() {
  return String(SUPABASE_RPC_URL);
}

const char* CODIGO_SERIAL = "SGE-034";
const unsigned long INTERVALO_MS = 6000;

// ── LEDs ──────────────────────────────────────────────────────
#define PINO_LED_VERDE    25
#define PINO_LED_VERMELHO 26

// ── Potenciômetro = simulação do acelerômetro ──────────────────
#define PINO_POT_ACCEL 34   // ADC1_6, input only, livre
const unsigned long INTERVALO_LEITURA_ACCEL_MS = 50; // 20 Hz
unsigned long ultimaLeituraAccel = 0;
int  leituraAccelAnterior = -1;
unsigned long tempoLeituraAnterior = 0;

// Limiar de velocidade de variação do potenciômetro (unidades ADC / segundo)
// Quanto maior o valor, mais rápido o giro precisa ser para contar como "queda".
const float LIMIAR_VELOCIDADE_QUEDA = 6000.0f;

// ── MAX30102 ──────────────────────────────────────────────────
MAX30105 sensorMax;
const byte TAXA_AMOSTRAS = 4;
byte taxas[TAXA_AMOSTRAS];
byte indiceRata = 0;
long ultimoPico = 0;
float bpmAtual = 0;
bool maxDisponivel = false;
bool cenarioControlaFC = false; // true só quando o cenário força o BPM manualmente (2,3,4)
const unsigned long INTERVALO_LEITURA_MAX_MS = 20;
unsigned long ultimaLeituraMax = 0;

// ── Monitor serial ─────────────────────────────────────────────
char modoVisualizacao = 'H'; // H = batimento, A = acelerometro(pot)
unsigned long ultimaExibicaoSerial = 0;
const unsigned long INTERVALO_EXIBICAO_MS = 500;

#define ANSI_CLEAR    "\033[2J\033[H"
#define ANSI_BOLD     "\033[1m"
#define ANSI_RESET    "\033[0m"
#define ANSI_VERDE    "\033[32m"
#define ANSI_AMARELO  "\033[33m"
#define ANSI_VERMELHO "\033[31m"
#define ANSI_CIANO    "\033[36m"
#define ANSI_CINZA    "\033[90m"

// ─────────────────────────────────────────────────────────────

unsigned long ultimoEnvio = 0;
volatile bool pendingEnvio = false;

AsyncWebServer servidorWeb(80);

// ── Estado atual ──────────────────────────────────────────────
struct Leitura {
  int    frequencia_cardiaca;
  int    oxigenacao_spo2;
  float  temperatura_corporal;
  int    nivel_bateria;
  bool   queda_detectada;
  float  accel_x, accel_y, accel_z;
  float  gyro_x,  gyro_y,  gyro_z;
  double latitude, longitude;
};

const Leitura LEITURA_NORMAL = {
  72, 98, 36.5, 87, false,
  0.12, 9.81, -0.05,
  0.01, -0.02, 0.00,
  -23.5505199, -46.6333094
};

Leitura leituraAtual = LEITURA_NORMAL;
String  cenarioAtual = "Normal";

// ── LEDs ───────────────────────────────────────────────────────
void atualizarLeds() {
  bool alerta = leituraAtual.queda_detectada
             || leituraAtual.frequencia_cardiaca < 50
             || leituraAtual.frequencia_cardiaca > 120
             || leituraAtual.oxigenacao_spo2     < 90
             || leituraAtual.temperatura_corporal > 38.0f
             || leituraAtual.temperatura_corporal < 35.0f
             || leituraAtual.nivel_bateria        < 15;

  digitalWrite(PINO_LED_VERMELHO, alerta  ? HIGH : LOW);
  digitalWrite(PINO_LED_VERDE,    !alerta ? HIGH : LOW);
}

// ── Histórico ─────────────────────────────────────────────────
#define HISTORICO_MAX 10

struct RegistroEnvio {
  String        cenario;
  String        payload;
  int           httpCode;
  String        resposta;
  unsigned long timestampMs;
};

RegistroEnvio historico[HISTORICO_MAX];
int historicoIndex = 0;
int historicoCount  = 0;

void registrarHistorico(const String& cenario, const String& payload, int httpCode, const String& resposta) {
  historico[historicoIndex] = { cenario, payload, httpCode, resposta, millis() };
  historicoIndex = (historicoIndex + 1) % HISTORICO_MAX;
  if (historicoCount < HISTORICO_MAX) historicoCount++;
}

// ── Wi-Fi ─────────────────────────────────────────────────────
void conectarWifi() {
  Serial.printf("Conectando ao Wi-Fi %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 20) {
    delay(500);
    Serial.print(".");
    tentativas++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWi-Fi conectado! IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("Mac: %s\n", WiFi.macAddress().c_str());
    Serial.printf("Painel de teste em: http://%s/\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nFalha ao conectar no Wi-Fi.");
  }
}

// ── Monta o JSON ──────────────────────────────────────────────
String montarPayload() {
  JsonDocument doc;
  doc["p_codigo_serial"]       = CODIGO_SERIAL;
  doc["p_frequencia_cardiaca"] = leituraAtual.frequencia_cardiaca;
  doc["p_temperatura"]         = leituraAtual.temperatura_corporal;

  JsonObject payload_obj = doc["p_payload"].to<JsonObject>();
  payload_obj["oxigenacao_spo2"] = leituraAtual.oxigenacao_spo2;
  payload_obj["nivel_bateria"]   = leituraAtual.nivel_bateria;
  payload_obj["queda_detectada"] = leituraAtual.queda_detectada;
  payload_obj["accel_x"]         = leituraAtual.accel_x;
  payload_obj["accel_y"]         = leituraAtual.accel_y;
  payload_obj["accel_z"]         = leituraAtual.accel_z;
  payload_obj["gyro_x"]          = leituraAtual.gyro_x;
  payload_obj["gyro_y"]          = leituraAtual.gyro_y;
  payload_obj["gyro_z"]          = leituraAtual.gyro_z;
  payload_obj["latitude"]        = leituraAtual.latitude;
  payload_obj["longitude"]       = leituraAtual.longitude;
  payload_obj["status_hardware"] = "ativo";

  String payload;
  serializeJson(doc, payload);
  return payload;
}

String montarPayloadDashboard() {
  JsonDocument doc;

  doc["bpm"] = leituraAtual.frequencia_cardiaca;
  doc["spo2"] = leituraAtual.oxigenacao_spo2;
  doc["temp"] = leituraAtual.temperatura_corporal;
  doc["bateria"] = leituraAtual.nivel_bateria;
  doc["queda"] = leituraAtual.queda_detectada;
  doc["cenario"] = cenarioAtual;
  
  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  doc["wifi_ok"] = wifiOk;
  doc["wifi_ip"] = wifiOk ? WiFi.localIP().toString() : "--";
  doc["uptime_ms"] = millis();

  int lastHttp = 0;
  if (historicoCount > 0) {
    int idx = (historicoIndex - 1 + HISTORICO_MAX) % HISTORICO_MAX;
    lastHttp = historico[idx].httpCode;
  }
  doc["ultimo_http"] = lastHttp;
  doc["max_ok"] = maxDisponivel;
  doc["ir"] = maxDisponivel ? sensorMax.getIR() : -1;

  String saida;
  serializeJson(doc, saida);
  return saida;
}

// ── Envio ─────────────────────────────────────────────────────
void enviarLeitura();
void enviarSeNaoEmCooldown(bool forcar = false) {
  unsigned long agora = millis();
  if (!forcar && (agora - ultimoEnvio < INTERVALO_MS)) {
    unsigned long faltam = (INTERVALO_MS - (agora - ultimoEnvio)) / 1000;
    Serial.printf(">> Envio ignorado (cooldown). Faltam ~%lus para o proximo envio permitido.\n", faltam);
    return;
  }
  ultimoEnvio = agora;
  enviarLeitura();
}

void enviarLeitura() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi desconectado, tentando reconectar...");
    conectarWifi();
    return;
  }

  String payload = montarPayload();
  HTTPClient http;
  http.begin(montarServerUrl());
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey",        SUPABASE_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);
  http.setTimeout(3000);
  http.setConnectTimeout(3000);

  int httpCode = http.POST(payload);
  String resposta;

  if (httpCode > 0) {
    resposta = http.getString();
    Serial.printf("[%s] HTTP %d — %s\n", cenarioAtual.c_str(), httpCode, resposta.c_str());
  } else {
    resposta = http.errorToString(httpCode);
    Serial.printf("[%s] Erro HTTP: %s\n", cenarioAtual.c_str(), resposta.c_str());
  }

  registrarHistorico(cenarioAtual, payload, httpCode, resposta);
  http.end();
}

// ── Potenciômetro (acelerômetro simulado) ──────────────────────
// Lê o potenciômetro, calcula a velocidade de variação (unid. ADC / s)
// e mapeia para "eixos" de aceleração fake. Se a velocidade ultrapassar
// o limiar, marca queda_detectada = true (só no cenário Normal,
// pra não atrapalhar cenários manuais via touch).
void lerPotenciometroAccel() {
  unsigned long agora = millis();
  if (agora - ultimaLeituraAccel < INTERVALO_LEITURA_ACCEL_MS) return;

  int leituraAdc = analogRead(PINO_POT_ACCEL);

  if (leituraAccelAnterior < 0) {
    // primeira leitura, so inicializa
    leituraAccelAnterior = leituraAdc;
    tempoLeituraAnterior = agora;
    ultimaLeituraAccel = agora;
    return;
  }

  float dt = (agora - tempoLeituraAnterior) / 1000.0f;
  if (dt <= 0) dt = 0.001f;

  float velocidade = fabsf((float)(leituraAdc - leituraAccelAnterior)) / dt;

  leituraAccelAnterior  = leituraAdc;
  tempoLeituraAnterior  = agora;
  ultimaLeituraAccel    = agora;

  if (cenarioAtual != "Normal") return; // cenarios manuais tem prioridade

  // Mapeia o valor do potenciometro (0-4095) para um eixo X "fake" em g (-2g a +2g)
  float axFake = ((leituraAdc / 4095.0f) * 4.0f) - 2.0f;
  leituraAtual.accel_x = axFake;
  leituraAtual.accel_y = 0.0f;
  leituraAtual.accel_z = 1.0f; // gravidade "parada"

  bool quedaPorVelocidade = (velocidade > LIMIAR_VELOCIDADE_QUEDA);
  leituraAtual.queda_detectada = quedaPorVelocidade;

  if (quedaPorVelocidade && cenarioAtual == "Normal") {
    // Atualiza tambem o nome do cenario soh para fins de exibicao/log,
    // mas sem disparar reenvio fora do ciclo normal de envio.
    cenarioAtual = "Queda detectada (auto)";
  } else if (!quedaPorVelocidade && cenarioAtual == "Queda detectada (auto)") {
    cenarioAtual = "Normal";
  }
}

// ── MAX30102 ──────────────────────────────────────────────────
void lerMAX30102() {
  if (!maxDisponivel) return;

  unsigned long agora = millis();
  if (agora - ultimaLeituraMax < INTERVALO_LEITURA_MAX_MS) return;
  ultimaLeituraMax = agora;

  long irValue = sensorMax.getIR();

  if (irValue < 50000) {
    bpmAtual = 0;
    // Sem dedo no sensor: NAO forca a leitura para 0.
    // Mantem o valor atual (que comeca em 72 = normal, vindo de LEITURA_NORMAL,
    // ou o valor do cenario manual ativo) ate existir uma leitura real do sensor.
    return;
  }
  

  if (checkForBeat(irValue)) {
    long delta = millis() - ultimoPico;
    ultimoPico = millis();

    float bpm = 60.0f / (delta / 1000.0f);
    if (bpm >= 20 && bpm <= 255) {
      taxas[indiceRata++] = (byte)bpm;
      indiceRata %= TAXA_AMOSTRAS;

      float soma = 0;
      for (byte i = 0; i < TAXA_AMOSTRAS; i++) soma += taxas[i];
      bpmAtual = soma / TAXA_AMOSTRAS;

      if (!cenarioControlaFC) {
        leituraAtual.frequencia_cardiaca = (int)bpmAtual;
      }
    }
  }
}

// ── Exibição Serial ───────────────────────────────────────────
void exibirSerial() {
  unsigned long agora = millis();
  if (agora - ultimaExibicaoSerial < INTERVALO_EXIBICAO_MS) return;
  ultimaExibicaoSerial = agora;

  Serial.print(ANSI_CLEAR);

  Serial.print(ANSI_BOLD);
  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║         SAGE - Monitor Serial        ║");
  Serial.println("╚══════════════════════════════════════╝");
  Serial.print(ANSI_RESET);

  Serial.printf(ANSI_CINZA "  Cenario: " ANSI_RESET "%s   Uptime: %lus\n\n",
                cenarioAtual.c_str(), millis() / 1000);

  if (modoVisualizacao == 'H') {
    Serial.print(ANSI_CIANO ANSI_BOLD);
    Serial.println("  [ BATIMENTO CARDIACO - MAX30102 ]");
    Serial.print(ANSI_RESET);

    if (!maxDisponivel) {
      Serial.print(ANSI_VERMELHO "  Sensor nao encontrado! Verifique I2C.\n" ANSI_RESET);
    } else {
      long ir = sensorMax.getIR();
      bool dedoPresente = ir >= 50000;

      if (!dedoPresente) {
        Serial.print(ANSI_AMARELO "  Aguardando dedo no sensor...\n" ANSI_RESET);
        Serial.printf(ANSI_CINZA "  IR raw: %ld\n" ANSI_RESET, ir);
      } else {
        const char* corBpm = (bpmAtual < 50 || bpmAtual > 120) ? ANSI_VERMELHO : ANSI_VERDE;
        Serial.printf("  BPM atual : %s%.0f bpm%s\n", corBpm, bpmAtual, ANSI_RESET);

        static bool pulso = false;
        pulso = !pulso;
        Serial.printf("  Pulso     : %s%s%s\n",
                      ANSI_VERMELHO, pulso ? " ♥  " : "  ♥ ", ANSI_RESET);

        Serial.printf(ANSI_CINZA "  IR raw   : %ld\n" ANSI_RESET, ir);
      }
    }

  } else if (modoVisualizacao == 'A') {
    Serial.print(ANSI_CIANO ANSI_BOLD);
    Serial.println("  [ ACELEROMETRO SIMULADO - Potenciometro ]");
    Serial.print(ANSI_RESET);

    int adc = analogRead(PINO_POT_ACCEL);
    const char* cor = leituraAtual.queda_detectada ? ANSI_VERMELHO : ANSI_VERDE;

    Serial.printf("  ADC raw    : %d / 4095\n", adc);
    Serial.printf("  Accel X    : %+.3f g\n", leituraAtual.accel_x);
    Serial.printf("  Queda      : %s%s%s\n", cor, leituraAtual.queda_detectada ? "SIM" : "nao", ANSI_RESET);

    if (leituraAtual.queda_detectada) {
      Serial.print(ANSI_VERMELHO ANSI_BOLD "  *** QUEDA DETECTADA! (giro rapido no pot) ***\n" ANSI_RESET);
    } else {
      Serial.print(ANSI_CINZA "  Gire devagar = normal | Gire rapido = queda\n" ANSI_RESET);
    }

  } else {
    Serial.print(ANSI_CIANO ANSI_BOLD);
    Serial.println("  [ TEMPERATURA / CENARIOS - Touch ]");
    Serial.print(ANSI_RESET);
    Serial.printf("  Temperatura atual: %.1f C\n", leituraAtual.temperatura_corporal);
    Serial.println("  Use os touch pins (ver menu) para mudar de cenario.");
  }

  Serial.println();
  Serial.print(ANSI_CINZA);
  Serial.println("  ──────────────────────────────────────");
  Serial.printf("  [h] Batimento  [a] Acelerometro  [t] Cenarios/Touch  (atual: %c)\n", modoVisualizacao);
  Serial.println("  [m] Menu de cenarios  [0] Reenviar agora");
  Serial.print(ANSI_RESET);
}

// ── Touch como gatilho de cenário: definição da tabela ──────────
// (precisa vir antes do motor de cenários abaixo, que referencia o tipo completo)
struct TouchCenario {
  uint8_t pino;
  char    tecla;
  const char* nome;
};

TouchCenario touchCenarios[] = {
  { T0, '2', "Parada cardiaca" },
  { T2, '3', "Bradicardia (BPM baixo)" },
  { T3, '4', "Taquicardia (BPM alto)" },
  { T4, '5', "Febre" },
  { T5, '6', "Hipotermia" },
  { T6, '7', "Hipoxia (SpO2 baixo)" },
  { T7, '8', "Queda detectada" },
  { T8, '9', "Bateria fraca" },
};
const int NUM_TOUCH = sizeof(touchCenarios) / sizeof(touchCenarios[0]);

// ── Cenários ──────────────────────────────────────────────────
bool cenarioAtivo[NUM_TOUCH] = {false}; // um flag por cenario de touch (teclas 2..9)

// Aplica APENAS os efeitos de campo de um cenario (sem mexer em cenarioAtual
// e sem resetar a struct inteira). Usada pelo motor de recalculo abaixo.
void aplicarEfeitosTecla(char tecla) {
  switch (tecla) {
    case '2': // Parada cardiaca
      leituraAtual.frequencia_cardiaca = 0;
      leituraAtual.oxigenacao_spo2     = 0;
      cenarioControlaFC = true;
      break;
    case '3': // Bradicardia (BPM baixo de verdade, < 50 = alerta no atualizarLeds)
      leituraAtual.frequencia_cardiaca = 45;
      cenarioControlaFC = true;
      break;
    case '4': // Taquicardia
      leituraAtual.frequencia_cardiaca = 165;
      cenarioControlaFC = true;
      break;
    case '5': // Febre
      leituraAtual.temperatura_corporal = 39.8;
      break;
    case '6': // Hipotermia
      leituraAtual.temperatura_corporal = 34.0;
      break;
    case '7': // Hipoxia
      leituraAtual.oxigenacao_spo2 = 85;
      break;
    case '8': // Queda detectada
      leituraAtual.queda_detectada = true;
      leituraAtual.accel_x = 2.8;
      leituraAtual.accel_y = -1.5;
      leituraAtual.accel_z = 3.2;
      break;
    case '9': // Bateria fraca
      leituraAtual.nivel_bateria = 8;
      break;
    default:
      break;
  }
}

int indiceTouchParaTecla(char tecla) {
  for (int i = 0; i < NUM_TOUCH; i++) {
    if (touchCenarios[i].tecla == tecla) return i;
  }
  return -1;
}

// Reconstroi leituraAtual do zero (a partir do LEITURA_NORMAL, ou seja,
// BPM 72 normal) e reaplica TODOS os cenarios que estiverem ativos em
// cenarioAtivo[]. Isso permite varios cenarios simultaneos (ex: bateria
// fraca + parada cardiaca) sem que soltar um deles apague o outro.
void recalcularCenarioComposto() {
  leituraAtual = LEITURA_NORMAL; // baseline: tudo normal, BPM 72
  cenarioControlaFC = false;

  String nomes = "";
  for (int i = 0; i < NUM_TOUCH; i++) {
    if (cenarioAtivo[i]) {
      aplicarEfeitosTecla(touchCenarios[i].tecla);
      if (nomes.length() > 0) nomes += " + ";
      nomes += touchCenarios[i].nome;
    }
  }

  cenarioAtual = (nomes.length() > 0) ? nomes : "Normal";

  Serial.printf("\n>> Cenario atual: %s\n", cenarioAtual.c_str());
  atualizarLeds();
}

// Selecao manual via serial (menu numerico) e sempre exclusiva: desliga
// qualquer cenario de touch que estivesse ativo e liga so o escolhido
// (ou nenhum, se for '1' = Normal). Depois delega pro mesmo motor de
// recalculo usado pelo touch, entao o estado nunca fica dessincronizado.
void selecionarCenario(char tecla) {
  for (int i = 0; i < NUM_TOUCH; i++) cenarioAtivo[i] = false;

  if (tecla != '1') {
    int idx = indiceTouchParaTecla(tecla);
    if (idx < 0) return; // tecla invalida, nao faz nada
    cenarioAtivo[idx] = true;
  }

  recalcularCenarioComposto();
}

void aplicarCenario(char tecla) {
  selecionarCenario(tecla);
  enviarSeNaoEmCooldown();
}

void imprimirMenu() {
  Serial.println("\n==============================================");
  Serial.println(" MENU DE TESTE - SAGE (simulacao de sensores)");
  Serial.println("==============================================");
  Serial.println(" [1] Normal");
  Serial.println(" [2] Parada cardiaca");
  Serial.println(" [3] Bradicardia (BPM baixo)");
  Serial.println(" [4] Taquicardia (BPM alto)");
  Serial.println(" [5] Febre (temperatura alta)");
  Serial.println(" [6] Hipotermia (temperatura baixa)");
  Serial.println(" [7] Hipoxia (SpO2 baixo)");
  Serial.println(" [8] Queda detectada (manual)");
  Serial.println(" [9] Bateria fraca");
  Serial.println(" [0] Reenviar leitura atual");
  Serial.println(" [m] Mostrar este menu");
  Serial.println("----------------------------------------------");
  Serial.println(" Cenarios tambem disparam via TOUCH (toggle):");
  Serial.println("   T0 (GPIO4)  -> Parada cardiaca");
  Serial.println("   T2 (GPIO2)  -> Bradicardia");
  Serial.println("   T3 (GPIO15) -> Taquicardia");
  Serial.println("   T4 (GPIO13) -> Febre");
  Serial.println("   T5 (GPIO12) -> Hipotermia");
  Serial.println("   T6 (GPIO14) -> Hipoxia");
  Serial.println("   T7 (GPIO27) -> Queda detectada (manual)");
  Serial.println("   T8 (GPIO33) -> Bateria fraca");
  Serial.println("----------------------------------------------");
  Serial.println(" Acelerometro = potenciometro no GPIO34:");
  Serial.println("   Giro lento  -> Normal");
  Serial.println("   Giro rapido -> Queda detectada (automatico)");
  Serial.println("----------------------------------------------");
  Serial.println(" [h] Ver batimento cardiaco (MAX30102)");
  Serial.println(" [a] Ver acelerometro (potenciometro)");
  Serial.println(" [t] Ver status de temperatura/cenario");
  Serial.println("==============================================");
  Serial.print("Escolha uma opcao: ");
}

// ── Touch como gatilho de cenário (toggle) ─────────────────────
const int LIMIAR_TOUCH = 30; // ajuste depois de calibrar (ver modo 'c')
const unsigned long DEBOUNCE_TOUCH_MS = 400;
bool tocadoAntes[NUM_TOUCH] = {false};
unsigned long ultimoToqueMs[NUM_TOUCH] = {0};

void verificarToques() {
  unsigned long agora = millis();
  bool mudou = false;

  for (int i = 0; i < NUM_TOUCH; i++) {
    int leitura = touchRead(touchCenarios[i].pino);
    bool tocando = (leitura < LIMIAR_TOUCH);

    if (tocando && !tocadoAntes[i] && (agora - ultimoToqueMs[i] > DEBOUNCE_TOUCH_MS)) {
      ultimoToqueMs[i] = agora;
      cenarioAtivo[i] = !cenarioAtivo[i]; // liga/desliga SO este cenario, sem mexer nos outros
      mudou = true;
    }
    tocadoAntes[i] = tocando;
  }

  if (mudou) {
    recalcularCenarioComposto();
    pendingEnvio = true;
  }
}

// Modo de calibração: imprime valores brutos de touch para ajudar a achar o LIMIAR_TOUCH
void calibrarTouch() {
  Serial.println("\n=== CALIBRACAO DE TOUCH (ctrl+C ou 'm' no serial para sair) ===");
  unsigned long inicio = millis();
  while (millis() - inicio < 8000) {
    for (int i = 0; i < NUM_TOUCH; i++) {
      Serial.printf("%s: %d   ", touchCenarios[i].nome, touchRead(touchCenarios[i].pino));
    }
    Serial.println();
    delay(300);
  }
  Serial.println("=== Fim da calibracao ===\n");
}

// ── Página HTML ───────────────────────────────────────────────
const char PAGINA_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-br">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SAGE - Painel de Teste</title>
<style>
  :root { color-scheme: dark; }
  body { font-family: 'Segoe UI', Arial, sans-serif; background:#10141a; color:#e6e9ef; margin:0; padding:20px; }
  h1 { font-size: 1.3rem; margin-bottom: 4px; }
  .sub { color:#8a93a3; font-size:0.85rem; margin-bottom:20px; }
  .card { background:#1a2029; border:1px solid #2a3340; border-radius:10px; padding:16px; margin-bottom:16px; }
  .card h2 { font-size:0.95rem; margin:0 0 10px; color:#9fb3ff; }
  .grid { display:grid; grid-template-columns: repeat(auto-fit, minmax(140px,1fr)); gap:10px; }
  .stat { background:#10141a; border-radius:8px; padding:10px; }
  .stat .label { font-size:0.7rem; color:#8a93a3; text-transform:uppercase; }
  .stat .value { font-size:1.4rem; font-weight:600; margin-top:2px; }
  .alerta .value { color:#ff6b6b; }
  pre { background:#0c0f14; padding:12px; border-radius:8px; overflow-x:auto; font-size:0.8rem; }
  .botoes { display:flex; flex-wrap:wrap; gap:8px; }
  button { background:#2a3340; color:#e6e9ef; border:1px solid #3a4555; border-radius:6px; padding:8px 12px; cursor:pointer; font-size:0.85rem; }
  button:hover { background:#3a4555; }
  table { width:100%; border-collapse:collapse; font-size:0.8rem; }
  th, td { text-align:left; padding:6px 8px; border-bottom:1px solid #2a3340; }
  .ok { color:#6bd17a; } .erro { color:#ff6b6b; }
</style>
</head>
<body>
  <h1>SAGE - Painel de Teste de Sensores</h1>
  <div class="sub">Atualiza automaticamente a cada 2s — IP: <span id="ip"></span></div>

  <div class="card">
    <h2>Ultima leitura enviada</h2>
    <div class="grid" id="statGrid"></div>
  </div>

  <div class="card">
    <h2>Disparar cenario remotamente</h2>
    <div class="botoes" id="botoesCenario"></div>
  </div>

  <div class="card">
    <h2>JSON exibido</h2>
    <pre id="jsonAtual">aguardando...</pre>
  </div>

  <div class="card">
    <h2>Historico de envios</h2>
    <table>
      <thead><tr><th>Cenario</th><th>HTTP</th><th>Resposta</th><th>Quando</th><th></th></tr></thead>
      <tbody id="historicoBody"></tbody>
    </table>
  </div>

<script>
const cenarios = [
  {id:'1', nome:'Normal'},
  {id:'2', nome:'Parada cardiaca'},
  {id:'3', nome:'Bradicardia'},
  {id:'4', nome:'Taquicardia'},
  {id:'5', nome:'Febre'},
  {id:'6', nome:'Hipotermia'},
  {id:'7', nome:'Hipoxia'},
  {id:'8', nome:'Queda detectada'},
  {id:'9', nome:'Bateria fraca'},
];

const botoesDiv = document.getElementById('botoesCenario');
cenarios.forEach(c => {
  const b = document.createElement('button');
  b.textContent = c.nome;
  b.onclick = () => fetch('/cenario?id=' + c.id).then(atualizar);
  botoesDiv.appendChild(b);
});

let ultimoHistorico = [];

function verJson(i) {
  const h = ultimoHistorico[i];
  if (h) document.getElementById('jsonAtual').textContent = JSON.stringify(JSON.parse(h.payload), null, 2);
}

async function atualizar() {
  try {
    const r = await fetch('/status');
    const d = await r.json();
    document.getElementById('ip').textContent = d.ip;

    document.getElementById('statGrid').innerHTML = `
      <div class="stat"><div class="label">Cenario</div><div class="value">${d.cenario}</div></div>
      <div class="stat ${d.frequencia_cardiaca===0?'alerta':''}"><div class="label">BPM</div><div class="value">${d.frequencia_cardiaca}</div></div>
      <div class="stat ${d.oxigenacao_spo2<90?'alerta':''}"><div class="label">SpO2</div><div class="value">${d.oxigenacao_spo2}%</div></div>
      <div class="stat ${(d.temperatura_corporal>38||d.temperatura_corporal<35)?'alerta':''}"><div class="label">Temp</div><div class="value">${d.temperatura_corporal}&deg;C</div></div>
      <div class="stat"><div class="label">Bateria</div><div class="value">${d.nivel_bateria}%</div></div>
      <div class="stat ${d.queda_detectada?'alerta':''}"><div class="label">Queda</div><div class="value">${d.queda_detectada?'SIM':'nao'}</div></div>
    `;

    document.getElementById('jsonAtual').textContent = JSON.stringify(JSON.parse(d.payload), null, 2);

    const hr = await fetch('/historico');
    const hist = await hr.json();
    ultimoHistorico = hist.slice().reverse();
    document.getElementById('historicoBody').innerHTML = ultimoHistorico.map((h, i) => `
      <tr>
        <td>${h.cenario}</td>
        <td class="${h.httpCode>0 && h.httpCode<400 ? 'ok':'erro'}">${h.httpCode}</td>
        <td>${h.resposta.substring(0,40)}</td>
        <td>${Math.round((d.uptimeMs - h.timestampMs)/1000)}s atras</td>
        <td><button onclick="verJson(${i})">ver</button></td>
      </tr>`).join('');
  } catch (e) {
    console.error(e);
  }
}

atualizar();
setInterval(atualizar, 2000);
</script>
</body>
</html>
)rawliteral";

// ── Servidor Web ──────────────────────────────────────────────
void configurarServidorWeb() {
  servidorWeb.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", PAGINA_HTML);
  });

  servidorWeb.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    doc["ip"]                   = WiFi.localIP().toString();
    doc["cenario"]              = cenarioAtual;
    doc["frequencia_cardiaca"]  = leituraAtual.frequencia_cardiaca;
    doc["oxigenacao_spo2"]      = leituraAtual.oxigenacao_spo2;
    doc["temperatura_corporal"] = leituraAtual.temperatura_corporal;
    doc["nivel_bateria"]        = leituraAtual.nivel_bateria;
    doc["queda_detectada"]      = leituraAtual.queda_detectada;
    doc["payload"]              = montarPayload();
    doc["uptimeMs"]             = millis();

    String saida;
    serializeJson(doc, saida);
    request->send(200, "application/json", saida);
  });

  servidorWeb.on("/historico", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();

    int total = historicoCount;
    int idx = (historicoIndex - total + HISTORICO_MAX) % HISTORICO_MAX;
    for (int i = 0; i < total; i++) {
      JsonObject o = arr.add<JsonObject>();
      RegistroEnvio &reg = historico[idx];
      o["cenario"]     = reg.cenario;
      o["httpCode"]    = reg.httpCode;
      o["resposta"]    = reg.resposta;
      o["timestampMs"] = reg.timestampMs;
      o["payload"]     = reg.payload;
      idx = (idx + 1) % HISTORICO_MAX;
    }

    String saida;
    serializeJson(doc, saida);
    request->send(200, "application/json", saida);
  });

  servidorWeb.on("/cenario", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("id")) {
      String id = request->getParam("id")->value();
      if (id.length() > 0) {
        selecionarCenario(id[0]);
        pendingEnvio = true;
      }
    }
    request->send(200, "text/plain", "ok");
  });

  servidorWeb.begin();
}

void setup() {
  Serial.begin(115200);
  delay(200); // da tempo do monitor serial conectar antes do primeiro print

  esp_reset_reason_t motivo = esp_reset_reason();
  Serial.print("\n>>> MOTIVO DO BOOT: ");
  switch (motivo) {
    case ESP_RST_POWERON:   Serial.println("POWERON (energia ligada/religada de verdade)"); break;
    case ESP_RST_BROWNOUT:  Serial.println("BROWNOUT (tensao caiu abaixo do limite -- ENERGIA)"); break;
    case ESP_RST_EXT:       Serial.println("EXT (pino EN foi puxado externamente -- pode ser DTR/RTS ou botao)"); break;
    case ESP_RST_SW:        Serial.println("SW (ESP.restart() ou reset por software)"); break;
    case ESP_RST_PANIC:     Serial.println("PANIC (crash/exception no codigo)"); break;
    case ESP_RST_INT_WDT:   Serial.println("WATCHDOG INTERNO (travou uma task)"); break;
    case ESP_RST_TASK_WDT:  Serial.println("WATCHDOG DE TASK (loop() travou por muito tempo)"); break;
    case ESP_RST_WDT:       Serial.println("OUTRO WATCHDOG"); break;
    case ESP_RST_DEEPSLEEP: Serial.println("SAIU DE DEEP SLEEP"); break;
    case ESP_RST_SDIO:      Serial.println("SDIO"); break;
    default:                Serial.printf("DESCONHECIDO (codigo %d)\n", (int)motivo); break;
  }
  Serial.println();

  pinMode(PINO_LED_VERDE,    OUTPUT);
  pinMode(PINO_LED_VERMELHO, OUTPUT);
  digitalWrite(PINO_LED_VERDE,    LOW);
  digitalWrite(PINO_LED_VERMELHO, LOW);

  // I2C usado apenas pelo MAX30102
  Wire.begin(21, 22); // SDA=21, SCL=22

  // MAX30102
  if (sensorMax.begin(Wire, I2C_SPEED_FAST, 0x3D)) {
    sensorMax.setup();
    sensorMax.setPulseAmplitudeRed(0x0A);
    sensorMax.setPulseAmplitudeGreen(0);
    Serial.println("MAX30102 OK.");
    maxDisponivel = true;
  } else {
    Serial.println("MAX30102 nao encontrado! Verifique I2C.");
  }

  conectarWifi();
  configurarServidorWeb();
  atualizarLeds();
  imprimirMenu();
}

void loop() {
  lerMAX30102();
  lerPotenciometroAccel();
  verificarToques();

  atualizarLeds();
  Serial.println("##JSON:" + montarPayloadDashboard());
  exibirSerial();

  if (Serial.available() > 0) {
    char tecla = Serial.read();
    if (tecla == '\n' || tecla == '\r') {
      // ignora
    } else if (tecla == 'h' || tecla == 'H') {
      modoVisualizacao = 'H';
    } else if (tecla == 'a' || tecla == 'A') {
      modoVisualizacao = 'A';
    } else if (tecla == 't' || tecla == 'T') {
      modoVisualizacao = 'T';
    } else if (tecla == 'm' || tecla == 'M') {
      imprimirMenu();
    } else if (tecla == 'c' || tecla == 'C') {
      calibrarTouch();
    } else if (tecla == '0') {
      Serial.println("\nReenviando leitura atual (forcado, ignora cooldown)...");
      enviarSeNaoEmCooldown(true);
    } else {
      aplicarCenario(tecla);
    }
  }

  if (pendingEnvio) {
    pendingEnvio = false;
    enviarSeNaoEmCooldown();
  }

  if (millis() - ultimoEnvio >= INTERVALO_MS) {
    enviarSeNaoEmCooldown();
  }
}
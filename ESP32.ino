//ATENÇÃO!!
//A distribuição de tarefas no sistema (scheduler) será feita de forma simplificada usando millis() devido ao tempo curto de desenvolvimento. O ideal seria usar o FreeRTOS disponível no ESP
//Pela possibilidade de trabalhar de maneira "paralela" evite funções que retornam valores e opte por usar variáveis globais

//Eletronicos x Protocolos
//BME - I2C
//GPS - UART/I2C
//LoRa - SPI (prioridade de SPI é dele)

// ====================== INCLUDES ======================
#include <stdio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// ====================== DEFINES ======================
#define Sea_Level_Pressure_HPA 1013.25

#define SAMPLE_RATE_BME   100     // 10 Hz  Pela seção 9.2 do datasheet o sample rate maximo suportado seria de 62,5Hz (considerou-se measurement time de 16ms)
#define SAMPLE_RATE_GPS   200     // 5 Hz
#define SAMPLE_RATE_LORA  1000    // 1 Hz
#define SD_TIMEOUT        3000    // flush SD

#define TIME_OUT_I2C 50 // Usamos 3x nosso MAIOR measurement time (estamos considerando 16ms)

// SPI
#define SCK_PIN  18
#define MISO_PIN 19
#define MOSI_PIN 23

// LoRa
#define LORA_CS   5
#define LORA_RST  14
#define LORA_IRQ  26

// SD
#define SD_CS 4

// ====================== STATUS FLAGS ======================
#define STATUS_BME   0x01
#define STATUS_GPS   0x02
#define STATUS_LORA  0x04

// ====================== STRUCT PRINCIPAL ======================
struct st_packet {
  uint32_t timestamp;
  uint8_t status;

  // BME
  float temp;
  float pressure;
  float alt;
  float hum;

  // GPS
  float lat;
  float lon;
  float gps_alt;
  uint8_t sats;

  // LoRa
  int16_t rssi;
  float snr;
};

// ====================== OBJETOS ======================
Adafruit_BME280 bme;
File sd_file;

// ====================== VARIÁVEIS GLOBAIS ======================
st_packet current_data;

unsigned long now = 0;
unsigned long last_bme = 0;
unsigned long last_gps = 0;
unsigned long last_lora = 0;
unsigned long last_sd_flush = 0;
bool data_was_collected = false;

// status sensores
bool status_bme = false;
bool status_lora = false;

// ====================== BUFFER LORA ======================
#define LORA_BUFFER_SIZE 12 //(1000/100) + 2 = 12. Em um ciclo do LoRa medimos a temperatura 10 vezes, vamos deixar uma folga de 2 medições
st_packet lora_buffer[LORA_BUFFER_SIZE];
uint8_t lora_count = 0;

// ====================== BUFFER SD ======================
#define SD_BUFFER_SIZE 25
st_packet sd_buffer[SD_BUFFER_SIZE];
uint8_t sd_head = 0;
uint8_t sd_count = 0;

// ======================================================
// ====================== BME ============================
// ======================================================
void init_bme() {
  if (bme.begin(0x76)){ //I2C esta 0x76 se SDO ligar em GND
    status_bme = true;
    Serial.println("BME Conectado no endereco 0x76");
  }
  else if (bme.begin(0x77)){ //I2C esta 0x77 se SDO ligar em VDDIO
    status_bme = true;
    Serial.println("BME Conectado no endereco 0x77");
  }
  else {
    Serial.println("BME280 não encontrado!");
    status_bme = false;
  }

  bme.setSampling(
                  Adafruit_BME280::MODE_FORCED, //Operaremos a BME no modo "Forced", assim quem define o gatilho da leitura é o scheduler
                  Adafruit_BME280::SAMPLING_X1, // temp
                  Adafruit_BME280::SAMPLING_X4, // pressão - 4x para uma maior precisão em altitude, pois não será usada IMU
                  Adafruit_BME280::SAMPLING_X1, // umidade
                  Adafruit_BME280::FILTER_X4 //Filtro de ruído - Com testes de bancada pode ser aumentando para X8 ou X16 caso o impacto na velocidade do firmware seja baixo
                );
//OBS: Checar seção 3.5.3 para configs recomndadas para INDOOR NAVIGATION (extrema precisão em altitude). Se for possível tentar aproximar as configs para as dessa seção
//Pela fórmula 9.1 do apêndice B o tempo máximo de measurement é ~16ms e o médio de ~15ms
}

bool ler_bme() {
  if (!status_bme) return false;

  if (bme.takeForcedMeasurement()) {
    current_data.temp = bme.readTemperature();
    current_data.pressure = bme.readPressure() / 100.0;
    current_data.alt = bme.readAltitude(Sea_Level_Pressure_HPA);
    current_data.hum = bme.readHumidity();

    current_data.status |= STATUS_BME; //Muda o último bit de STATUS_BME para 1
    return true;
  }

  current_data.status &= ~STATUS_BME; //Muda o último bit de STATUS_BME para 0
  return false;
}

// ======================================================
// ====================== GPS ============================
// ======================================================
bool ler_gps() {
  bool fix = false;

  if (fix) {
    current_data.lat = 0;
    current_data.lon = 0;
    current_data.gps_alt = 0;
    current_data.sats = 0;

    current_data.status |= STATUS_GPS;
    return true;
  }

  current_data.status &= ~STATUS_GPS;
  return false;
}

// ======================================================
// ====================== LORA ===========================
// ======================================================
void init_lora() {
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, LORA_CS);

  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (LoRa.begin(915E6)) {
    status_lora = true;
    Serial.println("Lora Inicializado");

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(250E3);
    LoRa.setCodingRate4(5);
  } else {
    status_lora = false;
    Serial.println("Erro na inicialização do Lora");
  }
}

void lora_add_buffer() {
  if (lora_count < LORA_BUFFER_SIZE) {
    lora_buffer[lora_count++] = current_data;
  }
}

bool send_lora() {
  if (!status_lora || lora_count == 0) return false;

  st_packet pkt = {0};
  pkt.timestamp = now;

  // média BME
  for (int i = 0; i < lora_count; i++) {
    pkt.temp += lora_buffer[i].temp;
    pkt.pressure += lora_buffer[i].pressure;
    pkt.alt += lora_buffer[i].alt;
    pkt.hum += lora_buffer[i].hum;
  }

  pkt.temp /= lora_count;
  pkt.pressure /= lora_count;
  pkt.alt /= lora_count;
  pkt.hum /= lora_count;

  // GPS (último valor)
  pkt.lat = current_data.lat;
  pkt.lon = current_data.lon;
  pkt.gps_alt = current_data.gps_alt;
  pkt.sats = current_data.sats;

  pkt.rssi = LoRa.packetRssi();
  pkt.snr = LoRa.packetSnr();

  pkt.status = current_data.status | STATUS_LORA; //Copia o status de current data para pkt

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket();

  lora_count = 0;
  return true;
}

// ======================================================
// ====================== SD =============================
// ======================================================
bool sd_init() {
  if (!SD.begin(SD_CS)) return false;

  if (!SD.exists("/data.csv")) {
    File f = SD.open("/data.csv", FILE_WRITE);
    if (f) {
      f.println("timestamp,status,temp,pressure,alt,hum,lat,lon,gps_alt,sats,rssi,snr");
      f.close();
    }
  }

  return true;
}

void sd_add() {
  if (sd_count < SD_BUFFER_SIZE) {
    sd_buffer[(sd_head + sd_count) % SD_BUFFER_SIZE] = current_data;
    sd_count++;
  }
}

void sd_flush() {
  if (sd_count == 0) return;

  File f = SD.open("/data.csv", FILE_APPEND);
  if (!f) return;

  while (sd_count > 0) {
    st_packet &p = sd_buffer[sd_head];

    f.print(p.timestamp); f.print(",");
    f.print(p.status); f.print(",");
    f.print(p.temp); f.print(",");
    f.print(p.pressure); f.print(",");
    f.print(p.alt); f.print(",");
    f.print(p.hum); f.print(",");
    f.print(p.lat, 6); f.print(",");
    f.print(p.lon, 6); f.print(",");
    f.print(p.gps_alt); f.print(",");
    f.print(p.sats); f.print(",");
    f.print(p.rssi); f.print(",");
    f.println(p.snr);

    sd_head = (sd_head + 1) % SD_BUFFER_SIZE;
    sd_count--;
  }

  f.flush(); //garante que os dados foram passados pro SD
  f.close();
}

// ======================================================
// ====================== SETUP ==========================
// ======================================================
void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22); //SDA, SCL
  Wire.setTimeOut(TIME_OUT_I2C); //Caso um sensor I2C pare de responder o ESP espera X ms, senão aborta a comunicação e libera o barramento pros outros (previne que um sensor com defeito bloqueie o barramento)

  init_bme();
  init_lora();
  sd_init();
}

// ======================================================
// ====================== LOOP ===========================
// ======================================================
void loop() {
  now = millis();

  current_data.timestamp = now;

  // ===== BME =====
  if (now - last_bme >= SAMPLE_RATE_BME) {
    ler_bme();
    last_bme = now;
    data_was_collected = true;
  }

  // ===== GPS =====
  if (now - last_gps >= SAMPLE_RATE_GPS) {
    ler_gps();
    last_gps = now;
  }

  // ===== BUFFER =====
  if (data_was_collected) { //O sample rate mais alto é o do BME. Só adicionamos no buffer algo quando adicionamos algo no BME, assim evitamos dados repetidos
    lora_add_buffer();
    sd_add();
    data_was_collected = false;
  }
  // ===== SD =====
  if (now - last_sd_flush >= SD_TIMEOUT) {
    sd_flush();
    last_sd_flush = now;
  }
}

  // ===== LORA =====
  if (now - last_lora >= SAMPLE_RATE_LORA) {
    send_lora();
    last_lora = now;
  }


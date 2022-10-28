/*
  lora_sck gpio26
  lora_miso gpio23
  lora_mosi gpio22
  lora_nss gpio12
  lora_reset gpio13
  power_switch gpio21
  sr04_trigger gpio19
  sr04_echo1 gpio18
  sr04_echo2 gpio5
  sr04_echo3 gpio17
  lora_d1 gpio16
  lora_d0 gpio4

  soilph gpio34 ADC1_6
  soilmoist gpio35 ADC1_7
  mq7 gpio32 ADC1_4
  uv_out gpio33 ADC1_5
  uv_ref gpio25 ADC2_8
  dht11 gpio27
  ldr gpio14

*/

// CABECERAS PARA SX1276 LORAWAN CHIP
#include "ttn.h"
#include <nvs_flash.h>
const char *appEui = "0000000000000000";// AppEUI (sometimes called JoinEUI)
const char *devEui = "70B3D57ED0056260";// DevEUI
const char *appKey = "752D7977297619554B9D1F7435DB85C6";// AppKey
#define TTN_SPI_HOST HSPI_HOST
#define TTN_SPI_DMA_CHAN 1
#define TTN_PIN_SPI_SCLK 26
#define TTN_PIN_SPI_MOSI 22
#define TTN_PIN_SPI_MISO 23
#define TTN_PIN_NSS 12
#define TTN_PIN_RXTX TTN_NOT_CONNECTED
#define TTN_PIN_RST 13
#define TTN_PIN_DIO0 4
#define TTN_PIN_DIO1 16
bool joined = false;

// CABECERAS PARA DHT11 (SENSOR DE TEMPERATURA Y HUMEDAD AMBIENTALES)
//#include <SimpleDHT.h>
#include "DHT.h"
int pinDHT11 = 27;
//SimpleDHT11 dht11(pinDHT11);
DHT dht(pinDHT11, DHT11);

// CABECERAS PARA LDR (SENSOR DE INTENSIDAD LUMINOSA)
int pinLDR = 14;

// CABECERAS PARA GYML8511 (SENSOR DE RADIACION ULTRAVIOLETA)
int UVOUT = 33; //Output from the sensor
int REF_3V3 = 25; //3.3V power on the Arduino board

// CABECERAS PARA MQ7 (SENSOR DE CONCENTRACION DE CO)
int pinMQ7 = 32;

// CABECERAS PARA SENSORES DE HUMEDAD Y ACIDEZ DE SUELO
int pinHumedadSuelo = 35;
int pinAcidezSuelo = 34;

// CABECERAS PARA SENSORES DE LLENADO SR04
#define SR04_TRIGGER 19
#define SR04_ECHO1   18
#define SR04_ECHO2   5
#define SR04_ECHO3   17
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

// VARIABLES GLOBALES PARA LAS LECTURAS
int s1 = 0;
int s2 = 0;
int s3 = 0;
int s4 = 0;
int s5 = 0;
int s6 = 0;
bool s7 = false;
int s8 = 0;
int s9 = 0;
int s10 = 0;

void setup() {
  delay(2000);
  Serial.begin(115200);

  // setup para SX1276 LORAWAN CHIP
  lora_init();

  // INICIALIZACION PARA DHT11 (SENSOR DE TEMPERATURA Y HUMEDAD AMBIENTAL)
  dht.begin();

  // INICIALIZACION PARA LDR (SENSOR DE INTENSIDAD LUMINOSA)
  pinMode(pinLDR, INPUT);

  // INICIALIZACION PARA GYML8511 (SENSOR DE RADIACION ULTRAVIOLETA)
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);

  // INICIALIZACION PARA MQ7 (SENSOR DE CONCENTRACION DE CO)
  pinMode(pinMQ7, INPUT);

  // INICIALIZACION PARA SENSORES DE HUMEDAD Y ACIDEZ DE SUELO
  pinMode(pinHumedadSuelo, INPUT);
  pinMode(pinAcidezSuelo, INPUT);

  // INICIALIZACION PARA SENSORES DE LLENADO SR04
  pinMode(SR04_TRIGGER, OUTPUT);
  pinMode(SR04_ECHO1, INPUT);
  pinMode(SR04_ECHO2, INPUT);
  pinMode(SR04_ECHO3, INPUT);
}

void loop() {
  read_sensors();
  send_data();

  // delay de 15 min
  for (int i = 0; i < 15; i++)
    delay(60000);//1min
}

// FUNCIONES AUXILIARES
void messageReceived(const uint8_t *message, size_t length, ttn_port_t port)
{
  printf("Message of %d bytes received on port %d:", length, port);
  for (int i = 0; i < length; i++)
    printf(" %02x", message[i]);
  printf("\n");
}
extern "C" void lora_init()
{
  esp_err_t err;

  // Initialize the GPIO ISR handler service
  err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  ESP_ERROR_CHECK(err);

  // Initialize the NVS (non-volatile storage) for saving and restoring the keys
  err = nvs_flash_init();
  ESP_ERROR_CHECK(err);

  // Initialize SPI bus
  spi_bus_config_t spi_bus_config = { -1};
  spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
  spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
  spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
  spi_bus_config.quadwp_io_num = -1;
  spi_bus_config.quadhd_io_num = -1;
  spi_bus_config.max_transfer_sz = 0;
  spi_bus_config.intr_flags = 0;
  spi_bus_config.flags = 0;

  err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
  ESP_ERROR_CHECK(err);

  // Initialize TTN
  ttn_init();

  // Configure the SX127x pins
  ttn_configure_pins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);

  // The below line can be commented after the first run as the data is saved in NVS
  ttn_provision(devEui, appEui, appKey);

  // Register callback for received messages
  ttn_on_message(messageReceived);

}
bool lora_join()
{
  if (joined)
    return true;

  ttn_reset();
  Serial.println("Joining.......");
  if ( ttn_join_provisioned())
  {
    joined = true;
    printf("Joined.\n");
    printf("    appEui:%s\n", appEui);
    printf("    devEui:%s\n", devEui);
    printf("    appKey:%s\n", appKey);
    return true;
  }

  Serial.println("Join failed.");
  return false;
}
bool lora_transmit(uint8_t *data, int size)
{
  printf("Sending LoRa message:%s\n", (char*)data);
  ttn_response_code_t res = ttn_transmit_message(data, size, 1, false);
  printf(res == TTN_SUCCESSFUL_TRANSMISSION ? "Message sent.\n" : "Transmission failed.\n");
  bool result = (res == TTN_SUCCESSFUL_TRANSMISSION) ? true : false;
  if (result == false)
    joined = false;
  return result;
}
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for (int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void read_sensors() {
  // LECTURAS DE ACIDEZ Y HUMEDAD DE SUELO
  int HumedadSuelo_steps = averageAnalogRead(pinHumedadSuelo);
  int AcidezSuelo_steps = averageAnalogRead(pinAcidezSuelo);
  Serial.print("Humedad de suelo steps: ");
  Serial.println(HumedadSuelo_steps);
  Serial.print("Acidez de suelo steps: ");
  Serial.println(AcidezSuelo_steps);
  s1 = (int)(AcidezSuelo_steps/200.0 + 3.0);

  s2 = (int)((-0.175438596) * HumedadSuelo_steps + 157.894736);
  s2 = (s2 >= 100) ? 100 : ( s2 >= 0 ? s2 : 0 );
  Serial.print("Humedad de suelo(HR): ");
  Serial.println(s2);

  // LECTURAS DE CONCENTRACION DE CO (MQT7)
  int co_mv = averageAnalogRead(pinMQ7) * (3300.0 / 4095.0) * 1.73333;
  Serial.print(" CO concentracion RAW()mv: ");
  Serial.println(co_mv);
  s3 = (int)(co_mv/10.0+20.0);

  // LECTURAS DE RADIACION ULTRAVIOLETA (GYML8511)
  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  float outputVoltage = 3.3 / refLevel * uvLevel;
  float uvIntensity = mapfloat(outputVoltage, 0.00, 2.9, 0.0, 15.0);//0.99, 2.9, 0.0, 15.0
  Serial.print(" UV Intensity (mW/cm^2): ");
  Serial.println(uvIntensity);
  s4 = (int)uvIntensity;

  // LECTURAS DE TEMPERATURA Y HUMEDAD AMBIENTALES (DHT11)
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  Serial.print(temperature); Serial.print(" *C, ");
  Serial.print(humidity); Serial.println(" H");
  s5 = (int)temperature;
  s6 = (int)humidity;

  // LECTURAS DE INTENSIDAD LUMINOSA (LDR)
  byte brillo = digitalRead(pinLDR);
  if (brillo == 0)
  {
    Serial.println("Foco en buen estado");
    s7 = true;
  }
  else
  {
    Serial.println("Foco en mal estado");
    s7 = false;
  }

  // LECTURAS DE SENSOR DE LLENADO 1
  delay(100);
  digitalWrite(SR04_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(SR04_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(SR04_TRIGGER, LOW);
  long duration1 = pulseIn(SR04_ECHO1, HIGH);
  float distanceCm1 = duration1 * SOUND_SPEED / 2;
  Serial.print("Distance1 (cm): ");
  Serial.println(distanceCm1);
  distanceCm1 = distanceCm1 >= 100.0 ? 100.0 : distanceCm1;
  s8 = (int)(100.0 - distanceCm1);

  // LECTURAS DE SENSOR DE LLENADO 2
  delay(100);
  digitalWrite(SR04_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(SR04_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(SR04_TRIGGER, LOW);
  long duration2 = pulseIn(SR04_ECHO2, HIGH);
  float distanceCm2 = duration2 * SOUND_SPEED / 2;
  Serial.print("Distance2 (cm): ");
  Serial.println(distanceCm2);
  distanceCm2 = distanceCm2 >= 100.0 ? 100.0 : distanceCm2;
  s9 = (int)(100.0 - distanceCm2);

  // LECTURAS DE SENSOR DE LLENADO 3
  delay(100);
  digitalWrite(SR04_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(SR04_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(SR04_TRIGGER, LOW);
  long duration3 = pulseIn(SR04_ECHO3, HIGH);
  float distanceCm3 = duration3 * SOUND_SPEED / 2;
  Serial.print("Distance3 (cm): ");
  Serial.println(distanceCm3);
  distanceCm3 = distanceCm3 >= 100.0 ? 100.0 : distanceCm3;
  s10 = (int)(100.0 - distanceCm3);
}
void send_data() {
  // EMPAQUETAR DATOS
  char datos[200] = {0};
  memset(datos, 0, 200);

  snprintf(datos, 200, "{"
           "\"s1\":%i,"
           "\"s2\":%i,"
           "\"s3\":%i,"
           "\"s4\":%i,"
           "\"s5\":%i,"
           "\"s6\":%i,"
           "\"s7\":%s,"
           "\"s8\":%i,"
           "\"s9\":%i,"
           "\"s10\":%i"
           "}",
           s1, s2, s3,
           s4, s5, s6,
           s7 ? "true" : "false",
           s8 , s9 , s10 );

  Serial.println(datos);

  // CONECTAR CON GATEWAY
join:

  if ( !lora_join() ) {
    Serial.println("Wait 5 seconds ...");
    delay(5000);
    goto join;
  }

  Serial.println("JOINED");
  delay(2000);
  Serial.println("ENVIANDO");

  // TRASMITIR DATOS
  bool tx_success = lora_transmit((uint8_t*)datos, strlen(datos));
  if (!tx_success) {
    Serial.println("Wait 5 seconds ...");
    delay(5000);
    goto join;
  }

  Serial.println("ENVIADO");

}

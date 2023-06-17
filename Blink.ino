/*
  LED Experiments

  Started from:
  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
#include <Arduino.h>
#include <bluefruit.h>
#include <avr/dtostrf.h>
#include <FastLED.h>

// https://learn.adafruit.com/led-tricks-gamma-correction/the-quick-fix
const uint8_t PROGMEM gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

#define MAX_PRPH_CONNECTION   2
uint8_t connection_count = 0;


BLEService lbsBatteryService(UUID16_SVC_BATTERY);
BLECharacteristic lbsBatteryAttr(UUID16_CHR_BATTERY_LEVEL);
BLEService lbsLedService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic lbsLedSwitchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic lbsLedHueCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214");

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(115200);
  Serial.println("Begin");
  for (int i=0; i++; i < 10 && !Serial) delay(100);
  delay(1000);

  Serial.println("Begin2");
  
  pinMode(VBAT_ENABLE, OUTPUT);
//  pinMode(BAT_CHARGE_STATE, INPUT);
//  digitalWrite(BAT_HIGH_CHARGE, HIGH); // charge with 50mA

  // BLE
  // Initialize Bluefruit with max concurrent connections as Peripheral = MAX_PRPH_CONNECTION, Central = 0
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin(MAX_PRPH_CONNECTION, 0);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  
  lbsBatteryService.begin();
  lbsBatteryAttr.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  lbsBatteryAttr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  lbsBatteryAttr.setFixedLen(1);
  lbsBatteryAttr.begin();
  lbsBatteryAttr.write8(0);

  lbsLedService.begin();
  lbsLedSwitchCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  lbsLedSwitchCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  lbsLedSwitchCharacteristic.setFixedLen(1);
  lbsLedSwitchCharacteristic.begin();
  lbsLedSwitchCharacteristic.write8(1);
  lbsLedSwitchCharacteristic.setWriteCallback(switch_write_callback);
  lbsLedHueCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  lbsLedHueCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  lbsLedHueCharacteristic.setFixedLen(1);
  lbsLedHueCharacteristic.begin();
  lbsLedHueCharacteristic.write8(0);
  lbsLedHueCharacteristic.setWriteCallback(hue_write_callback);
  
  startBleAdv();
}

void startBleAdv(void)
{
  // Don’t flash led while advertising
  Bluefruit.autoConnLed(false);
  
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include Service UUID
  Bluefruit.Advertising.addService(lbsBatteryService);
  Bluefruit.Advertising.addService(lbsLedService);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

uint32_t counter = 100;

const double vRef = 3.3; // Assumes 3.3V regulator output is ADC reference voltage
const unsigned int numReadings = 1024; // 10-bit ADC readings 0-1023, so the factor is 1024

void battery() {
  // https://forum.seeedstudio.com/t/xiao-ble-sense-battery-level-and-charging-status/263248/54
  // https://github.com/honvl/Seeed-Xiao-NRF52840-Battery/blob/main/xiaobattery.h
  digitalWrite(VBAT_ENABLE, LOW);
  unsigned int adcCount = analogRead(PIN_VBAT);
  double adcVoltage = (adcCount * vRef) / numReadings;
  double vBat = adcVoltage*1510.0/510.0; // Voltage divider from Vbat to ADC
  
  uint8_t percent = (vBat - 3.3) / (4.2 - 3.3) * 100;
  lbsBatteryAttr.write8(percent);

  char adcVoltageS[16];
  char vBatS[16];
  dtostrf(adcVoltage, 8, 4, adcVoltageS);
  dtostrf(vBat, 8, 4, vBatS);

  char output[128];
  sprintf(output, "adcCount = %3u = 0x%03X, adcVoltage = %sV, vBat = %s, percent = %u\n",
                   adcCount, adcCount, adcVoltageS, vBatS, percent);
  Serial.println(output);
  digitalWrite(VBAT_ENABLE, HIGH);
}

bool isLedOn = true;
CHSV color = CHSV(0, 255, 255);
int freezeLEDUntil = 0;
int lastBatteryUpdate = 0;

void shiftColors() {
  if (millis() > freezeLEDUntil) {
    color.setHSV(color.h+1, color.s, color.v);
    lbsLedHueCharacteristic.write8(color.h);
    lbsLedHueCharacteristic.notify8(color.h);
   
  }

  if (isLedOn) {
    CRGB outputColor = color;
    analogWrite(LED_RED, 255 - pgm_read_byte(&gamma8[outputColor.r]));
    analogWrite(LED_GREEN, 255 - pgm_read_byte(&gamma8[outputColor.g]));
    analogWrite(LED_BLUE, 255 - pgm_read_byte(&gamma8[outputColor.b]));
  } else {
    analogWrite(LED_RED, 255);
    analogWrite(LED_GREEN, 255);
    analogWrite(LED_BLUE, 255);
  }
}

// the loop function runs over and over again forever
void loop() {
  shiftColors();
  if (isLedOn) {
    delay(50);
  } else {
    delay(250);
  }

  if (millis() / 1000 > lastBatteryUpdate) {
    lastBatteryUpdate = millis() / 1000;
    Serial.println(counter++);
    battery();
  }
}


// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  (void) conn_handle;

  connection_count++;
  Serial.print("Connection count: ");
  Serial.println(connection_count);

  // Keep advertising if not reaching max
  if (connection_count < MAX_PRPH_CONNECTION)
  {
    Serial.println("Keep advertising");
    Bluefruit.Advertising.start(0);
  }
}

void switch_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  // data = 1 -> On
  // data = 0 -> Off
  if (len >= 1) {
    isLedOn = data[0] != 0;
    
    char output[32];
    sprintf(output, "switch command: 0x%02X\n",
                     data[0]);
    Serial.println(output);
  }
}
void hue_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len >= 1) {
    color.h = data[0];
    freezeLEDUntil = millis() + 10000;
    
    char output[32];
    sprintf(output, "hue command: %03u\n",
                     data[0]);
    Serial.println(output);
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

  connection_count--;
}

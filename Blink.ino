/*
  LED Experiments

  Started from:
  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
#include <Arduino.h>
#include <bluefruit.h>
#include <avr/dtostrf.h>

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
  
  uint8_t percent = (vBat - 3.3) / (3.8 - 3.3) * 100;
  lbsBatteryAttr.write8(percent);

  char adcVoltageS[16];
  char vBatS[16];
  dtostrf(adcVoltage, 8, 4, adcVoltageS);
  dtostrf(vBat, 8, 4, vBatS);

  char output[128];
  sprintf(output, "adcCount = %3u = 0x%03X, adcVoltage = %sV, vBat = %s\n",
                   adcCount, adcCount, adcVoltageS, vBatS);
  Serial.println(output);
  digitalWrite(VBAT_ENABLE, HIGH);
}

int R = -255;
int G = 0;
int B = 0;
int dR = 2;
int dG = -2;
int dB = 1;

// the loop function runs over and over again forever
void loop() {
  R = R + dR;
  G = G + dG;
  B = B + dB;
  if (R > 255) R = -255;
  if (G > 255) G = -255;
  if (B > 255) B = -255;
  if (R < -255) R = 255;
  if (G < -255) G = 255;
  if (B < -255) B = 255;
  
  analogWrite(LED_RED, 255 - pgm_read_byte(&gamma8[abs(R)]));
  analogWrite(LED_GREEN, 255 - pgm_read_byte(&gamma8[abs(G)]));
  analogWrite(LED_BLUE, 255 - pgm_read_byte(&gamma8[abs(B)]));
  delay(10);

  /*
  for (int brightness = 100; brightness >= 0; brightness--) {
    analogWrite(LED_R, (100 - brightness) * 255 / 100);
    delay(10);
    /*
    for (int repeat = 0; repeat < 100; repeat++) {
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on
    delayMicroseconds(brightness);
    digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off
    delayMicroseconds(100 - brightness);
    }
  }
    */
    /*
  for (int brightness = 0; brightness < 100; brightness++) {
    analogWrite(LED_BUILTIN, (100 - brightness) * 255 / 100);
//    delayMicroseconds(100*100);
    delay(10);
    /*
    for (int repeat = 0; repeat < 100; repeat++) {
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on
    delayMicroseconds(brightness);
    digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off
    delayMicroseconds(100 - brightness);
    }
  }
    */
//  delay(1000);                       // wait for a second
//  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
//  delay(1000);                       // wait for a second
if (B == 0) {
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

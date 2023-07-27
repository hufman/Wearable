/*
  LED Experiments

  Started from:
  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
#include <Arduino.h>
#include <bluefruit.h>
#include <avr/dtostrf.h>
#include <FastLED.h>
#include <MadgwickAHRS.h>

#define MAX_PRPH_CONNECTION   2
uint8_t connection_count = 0;

BLEDis bledis;
BLEService lbsBatteryService(UUID16_SVC_BATTERY);
BLECharacteristic lbsBatteryAttr(UUID16_CHR_BATTERY_LEVEL);
BLEService lbsLedService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic lbsLedSwitchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic lbsLedHueCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214");
BLEHidAdafruit blehid;

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
  
  // Configure and Start Device Information Service
  bledis.setManufacturer("ChezChat");
  bledis.setModel("Bangle");
  bledis.begin();
  
  blehid.begin();
  
  lbsBatteryService.begin();
  lbsBatteryAttr.setUserDescriptor("Battery Level");
  lbsBatteryAttr.setPresentationFormatDescriptor(BLE_GATT_CPF_FORMAT_UINT8, 0, UUID16_UNIT_PERCENTAGE, 0, 0);
  lbsBatteryAttr.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  lbsBatteryAttr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  lbsBatteryAttr.setFixedLen(1);
  lbsBatteryAttr.begin();
  lbsBatteryAttr.write8(0);

  lbsLedService.begin();
  lbsLedSwitchCharacteristic.setUserDescriptor("Switch");
  lbsLedSwitchCharacteristic.setPresentationFormatDescriptor(BLE_GATT_CPF_FORMAT_BOOLEAN, 0, UUID16_UNIT_UNITLESS, 0, 0);
  lbsLedSwitchCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  lbsLedSwitchCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  lbsLedSwitchCharacteristic.setFixedLen(1);
  lbsLedSwitchCharacteristic.begin();
  lbsLedSwitchCharacteristic.write8(1);
  lbsLedSwitchCharacteristic.setWriteCallback(switch_write_callback);
  lbsLedHueCharacteristic.setUserDescriptor("Hue");
  lbsLedHueCharacteristic.setPresentationFormatDescriptor(BLE_GATT_CPF_FORMAT_UINT8, 0, UUID16_UNIT_PLANE_ANGLE_DEGREE, 0, 0);
  lbsLedHueCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  lbsLedHueCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  lbsLedHueCharacteristic.setFixedLen(1);
  lbsLedHueCharacteristic.begin();
  lbsLedHueCharacteristic.write8(0);
  lbsLedHueCharacteristic.setWriteCallback(hue_write_callback);
  
  startBleAdv();

  // imu
  imuInit();
  // haptic 
  drvInit();
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
  Bluefruit.Advertising.addService(blehid);
  
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
  Bluefruit.Advertising.setInterval(32, 1636);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(0);      // number of seconds in fast mode
  Bluefruit.setName("Bangle");
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  5
}

uint32_t counter = 100;

const double vRef = 3.3; // Assumes 3.3V regulator output is ADC reference voltage
const unsigned int numReadings = 1024; // 10-bit ADC readings 0-1023, so the factor is 1024

bool isLedOn = true;
CHSV color = CHSV(0, 255, 128);
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
    analogWrite(LED_RED, 255 - outputColor.r);
    analogWrite(LED_GREEN, 255 - outputColor.g);
    analogWrite(LED_BLUE, 255 - outputColor.b);
  } else {
    analogWrite(LED_RED, 255);
    analogWrite(LED_GREEN, 255);
    analogWrite(LED_BLUE, 255);
  }
}

void battery() {
  // https://forum.seeedstudio.com/t/xiao-ble-sense-battery-level-and-charging-status/263248/54
  // https://github.com/honvl/Seeed-Xiao-NRF52840-Battery/blob/main/xiaobattery.h
  digitalWrite(VBAT_ENABLE, LOW);
  unsigned int adcCount = analogRead(PIN_VBAT);
  double adcVoltage = (adcCount * vRef) / numReadings;
  double vBat = adcVoltage*1510.0/510.0; // Voltage divider from Vbat to ADC
  
  uint8_t percent = (vBat - 3.3) / (3.85 - 3.3) * 100;
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

float previousAngle = -5000;
void imuColor() {
  if (millis() < freezeLEDUntil) {
    // set manually over Bluetooth, don't change color
    return;
  }
  
  Madgwick orientation = madgwickGet();
  int hue = orientation.getRoll() * (2 * 256) / 360 + 2 * 256;
  color.setHSV(hue, color.s, color.v);
  lbsLedHueCharacteristic.write8(color.h);
  lbsLedHueCharacteristic.notify8(color.h);

  // volume control by tilt
  int roll = orientation.getRoll();
  if (roll > 128) roll -= 360;
  if (previousAngle < -4000) previousAngle = roll;
  float angle_moved = previousAngle - roll;
  if (abs(angle_moved) > 128) angle_moved = 0;
  int notches = angle_moved / 10;
  
  if (abs(notches) > 0) {
   drvTrigger(26);
  }
  for (int i = 0; i < abs(notches); i++) {
    if (notches < 0) {
      blehid.consumerKeyPress(HID_USAGE_CONSUMER_VOLUME_INCREMENT);
    } else {
      blehid.consumerKeyPress(HID_USAGE_CONSUMER_VOLUME_DECREMENT);
    }
    delay(5);
    blehid.consumerKeyRelease();
    previousAngle = roll;
  }
}


int effect = 0;
// the loop function runs over and over again forever
void loop() {
  imuUpdate();
  imuColor();
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
    reportIMU();

    // haptic test
    drvIdle();
//    drvTrigger(effect++);
//    if (effect > 123) effect = 0;
    /*
    if (counter % 10 > 5) {
      blehid.consumerKeyPress(HID_USAGE_CONSUMER_VOLUME_INCREMENT);
    } else {
      blehid.consumerKeyPress(HID_USAGE_CONSUMER_VOLUME_DECREMENT);
    }
    */
  }
  readInput();
}

const byte serialInputSize = 127;
char receivedChars[serialInputSize];
byte receivedCharCount = 0;

void readInput() {
  byte origCount = receivedCharCount;
  while (Serial.available()) {
    char rc = Serial.read();
    if (rc != '\n') {
      receivedChars[receivedCharCount] = rc;
      receivedCharCount++;
      if (receivedCharCount == serialInputSize) {
        resetInput();
      }
    } else {
      parseInput();
      resetInput();
    }
  }
  if (origCount < receivedCharCount) {
    Serial.print("!! Received input: ");
    Serial.println(receivedChars);
  }
}

void parseInput() {
  // {"command": "enable_orientation_reporting"}
  if (strcmp(receivedChars, "{\"command\": \"enable_orientation_reporting\"}") == 0) {
    enableOrientationReporting();
  } else {
    Serial.print("!!!Unknown command: ");
    Serial.println(receivedChars);
  }
}

void resetInput() {
  receivedCharCount = 0;
  receivedChars[receivedCharCount] = 0;
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

/**
   ESPNOW - Basic communication - Slave
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and a Slave ESP32
   Description: This sketch consists of the code for the Slave module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   << This Device Slave >>

   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `slave` to the SSID of slave for an easy setup)
   Step 3 : Once found, add Slave as peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave

   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Update the SSID of Slave with a prefix of `slave`
   Step 3 : Set Slave in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the serial monitor

   Note: Master and Slave have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Master and Slave.
         Any devices can act as master or salve.
*/

//#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include "ssd1306.h"
#include "nano_engine.h"

#define CHANNEL 1

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

String line[8] = {"","","","","","","",""};
bool nextLine;

void logShift(String text) {
  line[7] = line[6];
  line[6] = line[5];
  line[5] = line[4];
  line[4] = line[3];
  line[3] = line[2];
  line[2] = line[1];
  for (int length = line[0].length(); length < 19; length++) {
    line[0] += " ";
  }
  line[1] = line[0];
  String textTemp = text;
  for (int length = textTemp.length(); length < 19; length++) {
    textTemp += " ";
  }
  line[0] = textTemp;
  // ssd1306_setColor(RGB_COLOR8(0,0,0));
  // ssd1306_fillRect8(0, 8 * 7, ssd1306_displayWidth() - 1, 8 * 7 + 7);
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t y = 7 - i;
    ssd1306_setColor(RGB_COLOR8(255,255,255));
    char charArray[line[i].length() + 1];
    line[i].toCharArray(charArray, line[i].length() + 1);
    ssd1306_printFixed8(0,  8 * y, charArray, STYLE_NORMAL);
  }
  line[0] = text;
}

void logUpdateRecent(String text) {
  ssd1306_setColor(RGB_COLOR8(255,255,255));
  char charArray[text.length() + 1];
  text.toCharArray(charArray, text.length() + 1);
  ssd1306_printFixed8(5 * line[0].length(),  8 * 7, charArray, STYLE_NORMAL);
  line[0] += text;
}

void logPrint(String text) {
  if (nextLine == true) {
    // Shift up and print text
    nextLine = false;
    logShift(text);
  }
  else {
    // nextLine stays false
    // Don't shift up and print text on the same line
    logUpdateRecent(text);
  }
}

void logPrintln(String text) {
  if (nextLine == true) {
    // nextLine stays true
    // Shift up and print text
    logShift(text);
  }
  else {
    // Print text on the same line
    nextLine = true;
    logUpdateRecent(text);
  }
}

void screenSetup() {
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1331_96x64_spi_init(22, 5, 21);
  ssd1306_setMode(LCD_MODE_NORMAL);
  ssd1306_clearScreen8();
  // ssd1306_setFixedFont(digital_font5x7);
  ssd1306_setFixedFont(ssd1306xled_font5x7);
  ssd1306_setColor(RGB_COLOR8(255,255,255));
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); 
  for (int i = 0; i < data_len; i++) {
    Serial.print((char)data[i]);
    logPrint( (String)( (char)data[i] ) );
  }
  Serial.print(" ("); Serial.print(data_len); Serial.println(")");
  logPrintln("");
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Setup the SSD1331
  screenSetup();
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);

  logPrintln("Hello");
  logPrintln("BIIIIIIIIIIIIG");
  logPrintln("smol");
  
}

unsigned long nextChar;

void loop() {
  unsigned long t = millis();
  if (t > nextChar) {
    nextChar = t + 50;

  }
}
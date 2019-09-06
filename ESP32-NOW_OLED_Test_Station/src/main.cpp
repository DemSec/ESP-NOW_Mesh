#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include "Ucglib.h"

#define CHANNEL 1

// CLK = 18; SDA = 23; RST = 21; CS = 5; D/C = 22;
Ucglib_SSD1331_18x96x64_UNIVISION_HWSPI ucg(21, 5, 22);
ucg_t *SSD1331;

String line[10];
bool nextLine;
unsigned long nextChar;
String writing;
bool logWaited;
bool typingMotion;
// To prevent horrible nightmare screen glitches
// I dare you turn it off
bool dataRecieveFlag; 

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

void screenInit() {
  ucg.begin(UCG_FONT_MODE_TRANSPARENT);
  ucg.clearScreen();
  SSD1331 = ucg.getUcg();
  ucg_SetColor(SSD1331, 0, 255, 255, 255);
  ucg_SetFont(SSD1331, ucg_font_5x8_tr);
  // ucg_SetFontMode(SSD1331, 0);
  typingMotion = true;
}

void logShift(String text) {

  logWaited = false;
  line[9] = line[8];
  line[8] = line[7];
  line[7] = line[6];
  line[6] = line[5];
  line[5] = line[4];
  line[4] = line[3];
  line[3] = line[2];
  line[2] = line[0];
  line[1] = "";
  line[0] = text;
  
  for (uint8_t i = 0; i < 8; i++) {

    char charArrayOld[line[i + 2].length() + 1]; // Create empty character array
    line[i + 2].toCharArray(charArrayOld, line[i + 2].length() + 1); // Fill it with old characters
    char charArrayNew[line[i + 1].length() + 1]; // Create empty character array
    line[i + 1].toCharArray(charArrayNew, line[i + 1].length() + 1); // Fill it with new characters

    for (uint8_t j = 0; j < 20; j++) {

      if (char c = line[i + 2].charAt(j)) {
        char charArrayTemp[2] = {c};
        ucg_SetColor(SSD1331, 0, 0, 0, 0); // Set color to black
        ucg_DrawString(SSD1331, 5 * j, 8 * (7 - i) + 7, 0, charArrayTemp); // Erase old character
      }
      if (char c = line[i + 1].charAt(j)) {
        char charArrayTemp[2] = {c};
        ucg_SetColor(SSD1331, 0, 255, 255, 255); // Set color to white
        ucg_DrawString(SSD1331, 5 * j, 8 * (7 - i) + 7, 0, charArrayTemp); // Draw new character
      }
    }
  }
}

void logUpdateRecent(String text) {
  line[0] += text;
  if (!typingMotion) {
    line[1] += text;
    char charArray[text.length() + 1];
    text.toCharArray(charArray, text.length() + 1);
    ucg_SetColor(SSD1331, 0, 255, 255, 255); // Set color to white
    ucg_DrawString(SSD1331, 5 * line[1].length(), 8 * (7 - 0) + 7, 0, charArray); // Draw new characters
  }
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

void logType() {
  unsigned long t = millis();
  if ( (line[0].length() > line[1].length()) & (t > nextChar) & !dataRecieveFlag ) {
    if (logWaited) {
      char c[2] = {line[0].charAt(line[1].length())};
      ucg_SetColor(SSD1331, 0, 255, 255, 255);
      ucg_DrawString(SSD1331, 5 * line[1].length(),  8 * 7 + 7, 0, c);
      line[1] += (String)c;
      // Don't do typing motion if typingMotion = false or the character was a space
      nextChar = t + (!typingMotion ? 0 : ((String)c == " " ? 0 : 50) );
    }
    else {
      nextChar = t + (!typingMotion ? 0 : 100);
      logWaited = true;
    }
  }
}

void serialInit() {
  Serial.begin(115200);
  Serial.flush();
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  dataRecieveFlag = true;
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); 
  for (int i = 0; i < data_len; i++) {
    Serial.print((char)data[i]);
    logPrint( (String)(char)data[i]);
  }
  Serial.print(" ("); Serial.print(data_len); Serial.println(")");
  logPrintln("");
  dataRecieveFlag = false;
}

void setup() {
  serialInit();
  screenInit();
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  logPrintln("MAC of Self: "); logPrintln(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
}

String command;

void serialEcho() {
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print(c);
    if (c == '\b') {
      command.remove(command.length() - 1);
      Serial.print(" ");
      Serial.print('\b');
    }
    else if (c == '\n') {
      logPrintln(command);
      command = "";
    } else {
      command += c;
    }
    Serial.flush();
  }
}

void loop()
{
  logType();
  serialEcho();
}

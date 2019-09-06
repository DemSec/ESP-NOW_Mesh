#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include "Ucglib.h"

#define CHANNEL 1

// CLK = 18; SDA = 23; RST = 21; CS = 5; D/C = 22;
Ucglib_SSD1331_18x96x64_UNIVISION_HWSPI ucg(21, 5, 22);
ucg_t *SSD1331;


// Global copy of slave
esp_now_peer_info_t slave;


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

bool manageSlave() {
  if (slave.channel == CHANNEL) {

    Serial.print("Slave Status: ");
    const esp_now_peer_info_t *peer = &slave;
    const uint8_t *peer_addr = slave.peer_addr;
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(peer_addr);
    if ( exists) {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(peer);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

bool slaveFound;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  dataRecieveFlag = true;
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  if ( !slaveFound ) {
    for (int ii = 0; ii < 6; ++ii ) {
      slave.peer_addr[ii] = (uint8_t) mac_addr[ii];
    }
    slaveFound = true;
    manageSlave();
  }
  Serial.print("Last Packet Recv Data: "); 
  for (int i = 0; i < data_len; i++) {
    Serial.print((char)data[i]);
    logPrint( (String)(char)data[i]);
  }
  Serial.print(" ("); Serial.print(data_len); Serial.println(")");
  logPrintln("");
  dataRecieveFlag = false;
}

uint8_t data = 0;
// send data
void sendData() {
  data++;
  const uint8_t *peer_addr = slave.peer_addr;
  Serial.print("Sending: "); Serial.println(data);
  
  // uint8_t arrayOfData[250];
  // for (uint8_t i = 0; i < 250; i++) {
  //   arrayOfData[i] = data;
  // }
  // esp_err_t result = esp_now_send(peer_addr, arrayOfData, sizeof(arrayOfData));
  
  esp_err_t result = esp_now_send(peer_addr, &data, sizeof(data));
  
  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
  esp_now_register_send_cb(OnDataSent);
}

String command;

void serialEcho() {
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print(c);
    // if ((uint8_t)c == 195) {
      
    // }
    // else if ((uint8_t)c == 160) {
      
    // }
    // else if ((uint8_t)c == 75) { // Left
      
    // }
    // else if ((uint8_t)c == 77) { // Right
      
    // }
    // else if ((uint8_t)c == 72) { // Up
      
    // }
    // else if ((uint8_t)c == 80) { // Down
      
    // }
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

unsigned long nextSend;

void loop()
{
  logType();
  serialEcho();
  unsigned long t = millis();
  if (t > nextSend && slaveFound) {
    nextSend = t + 500;
    sendData();
  }
}

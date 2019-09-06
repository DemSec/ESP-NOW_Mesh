#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SPI.h>
#include "Ucglib.h"

// Global copy of slave
esp_now_peer_info_t slave;
#define CHANNEL 3
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

#define DHTPIN 17
#define DHTTYPE DHT11

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

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

// Scan for slaves in AP mode
void ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        logPrintln("MAC of Other: "); logPrintln(BSSIDstr.c_str());
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x%c",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption
        //esp_now_set_pmk({(uint8_t)144});
        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound) {
    Serial.println("Slave Found, processing..");
  } else {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

void deletePeer() {
  const esp_now_peer_info_t *peer = &slave;
  const uint8_t *peer_addr = slave.peer_addr;
  esp_err_t delStatus = esp_now_del_peer(peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

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

void sendData(String str) {
  const size_t data_len = str.length();
  uint8_t data[data_len];
  Serial.print("Sending: ");
  for (int i = 0; i < data_len; i++) {
    data[i] = str.charAt(i);
    Serial.print((char)data[i]);
  }
  Serial.print(" ("); Serial.print(data_len); Serial.println(")");
  const uint8_t *peer_addr = slave.peer_addr;
  const size_t temp_len = 1;
  const uint8_t temp[temp_len] = {0xFA};
  digitalWrite(4, HIGH);
  digitalWrite(4, LOW);
  esp_err_t result = esp_now_send(peer_addr, data, data_len);
  digitalWrite(4, HIGH);
  digitalWrite(4, LOW);
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

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void dhtInit() {
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
}

void setup() {
  serialInit();
  screenInit();
  dhtInit();
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow/Basic/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  logPrintln("MAC of Self: "); logPrintln(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
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

unsigned long nextManage;
unsigned long nextSend;

void loop() {
  // In the loop we scan for slave
  ScanForSlave();
  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (slave.channel == CHANNEL) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    bool isPaired = manageSlave();
    nextManage = millis() + 1000;
    while (isPaired) {

      unsigned long t = millis();
      if (t > nextManage) {
        nextManage = t + 1000;
        isPaired = manageSlave();
      }

      logType();
      // serialEcho();

      if (t > nextSend) {
        nextSend = t + delayMS;
        sensors_event_t event;
        dht.temperature().getEvent(&event);
        String temperature;
        if (!isnan(event.temperature)) {
          temperature = String(event.temperature);
        }
        String humidity;
        dht.humidity().getEvent(&event);
        if (!isnan(event.relative_humidity)) {
          humidity = event.relative_humidity;
        }
        String str = "T = " + temperature.substring(0, 2) + " C, H = " + humidity.substring(0, 2) + " %";
        Serial.print("Reading: " + str); Serial.print(" ("); Serial.print(str.length()); Serial.println(")");
        sendData(str);
      }
    }
  }
}
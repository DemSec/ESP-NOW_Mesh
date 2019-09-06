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

#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include "Ucglib.h"

// Global copy of slave
esp_now_peer_info_t slave;
#define CHANNEL1 1
#define CHANNEL2 3
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0
// Please uncomment the node code to which you upload:
// For Node A
// #define SLAVE_SSID "Slave_A"
bool packetRecieved = true;
// For Node B
#define SLAVE_SSID "Slave_B"
// bool packetRecieved = false;

#define STA_MAC_RED   "24:0A:C4:0E:9E:80"
#define STA_MAC_BLACK "24:0A:C4:0E:9D:F4"

#define SCAN_PIN    14
#define MANAGE_PIN  26
#define SEND_PIN    27
#define RECEIVE_PIN 33
#define CONFIRM_PIN 25

// CLK = 18; SDA = 23; RST = 21; CS = 5; D/C = 22;
Ucglib_SSD1331_18x96x64_UNIVISION_HWSPI ucg(21, 5, 22);

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
void InitESPNow()
{
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESPNow Init Success");
  }
  else
  {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

void screenInit() {
  ucg.begin(UCG_FONT_MODE_SOLID);
  ucg.clearScreen();
  ucg.setColor(0, 255, 255, 255);
  ucg.setFont(ucg_font_5x8_mr);
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
  line[2] = line[1];
  line[1] = text;
  line[0] = "";
  // If there is typing motion, skip the first line
  for (uint8_t i = (typingMotion? 1 : 0); i < 8; i++) {

    ucg.setPrintPos(0, 8 * (7 - i) + 7);
    ucg.print(line[i + 1]);
    int numOfSpaces = 19 - line[i + 1].length();
    while (numOfSpaces > 0) {
      ucg.print(" ");
      numOfSpaces--;
    }
  }
}

void logUpdateRecent(String text) {
  line[0] += text;
  if (!typingMotion) {
    ucg.setPrintPos(5 * line[1].length(), 8 * 7 + 7);
    line[1] += text;
    ucg.print(text);
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
  if (typingMotion) {
    unsigned long t = millis();
    if ( (line[0].length() > line[1].length()) & (t > nextChar) & !dataRecieveFlag ) {
      if (logWaited) {
        String c = (String)line[0].charAt(line[1].length());
        ucg.setPrintPos(5 * line[1].length(), 8 * 7 + 7);
        line[1] += (String)c;
        ucg.print(c);
        // Don't do typing motion if typingMotion = false or the character was a space
        nextChar = t + (!typingMotion ? 0 : ((String)c == " " ? 0 : 50) );
      }
      else {
        nextChar = t + (!typingMotion ? 0 : 100);
        logWaited = true;
      }
    }
  }
}

// Scan for slaves in AP mode
void ScanForSlave()
{
  digitalWrite(SCAN_PIN, HIGH);
  int8_t scanResults = WiFi.scanNetworks();
  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  Serial.println("");
  if (scanResults == 0)
  {
    Serial.println("No WiFi devices in AP Mode found");
  }
  else
  {
    Serial.print("Found ");
    Serial.print(scanResults);
    Serial.println(" devices ");
    for (uint8_t i = 0; i < scanResults; ++i)
    {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS)
      {
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
      if (SSID.indexOf("Slave") == 0)
      {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" [");
        Serial.print(BSSIDstr);
        Serial.print("]");
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x%c", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
        {
          for (int ii = 0; ii < 6; ++ii)
          {
            slave.peer_addr[ii] = (uint8_t)mac[ii];
          }
        }

        slave.channel = CHANNEL2; // pick a channel
        slave.encrypt = 0;        // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound)
  {
    Serial.println("Slave Found, processing..");
  }
  else
  {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
  digitalWrite(SCAN_PIN, LOW);
}

void slaveAssign(char macAddress[]) {
  int mac[6];
  if (6 == sscanf(macAddress, "%x:%x:%x:%x:%x:%x%c", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
  {
    for (int ii = 0; ii < 6; ++ii)
    {
      slave.peer_addr[ii] = (uint8_t)mac[ii];
    }
  }

  slave.channel = CHANNEL2; // pick a channel
  slave.encrypt = 0;        // no encryption
}

void deletePeer()
{
  const esp_now_peer_info_t *peer = &slave;
  const uint8_t *peer_addr = slave.peer_addr;
  esp_err_t delStatus = esp_now_del_peer(peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK)
  {
    // Delete success
    Serial.println("Success");
  }
  else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT)
  {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  }
  else if (delStatus == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Invalid Argument");
  }
  else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.println("Peer not found.");
  }
  else
  {
    Serial.println("Not sure what happened");
  }
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave()
{
  digitalWrite(MANAGE_PIN, HIGH);
  digitalWrite(MANAGE_PIN, LOW);
  if (slave.channel == CHANNEL2)
  {
    if (DELETEBEFOREPAIR)
    {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    const esp_now_peer_info_t *peer = &slave;
    const uint8_t *peer_addr = slave.peer_addr;
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(peer_addr);
    if (exists)
    {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    }
    else
    {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(peer);
      if (addStatus == ESP_OK)
      {
        // Pair success
        Serial.println("Pair success");
        return true;
      }
      else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT)
      {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_ARG)
      {
        Serial.println("Invalid Argument");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_FULL)
      {
        Serial.println("Peer list full");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_NO_MEM)
      {
        Serial.println("Out of memory");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_EXIST)
      {
        Serial.println("Peer Exists");
        return true;
      }
      else
      {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  }
  else
  {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

uint8_t number = 0;
// send data
void sendData()
{
  digitalWrite(SEND_PIN, HIGH);
  number++;
  const uint8_t *peer_addr = slave.peer_addr;
  Serial.print("Sending: ");
  Serial.println(number);

  uint8_t arrayOfData[250];
  for (uint8_t i = 0; i < 250; i++)
  {
    arrayOfData[i] = number;
  }
  esp_err_t result = esp_now_send(peer_addr, arrayOfData, sizeof(arrayOfData));

  // esp_err_t result = esp_now_send(peer_addr, &number, sizeof(number));

  Serial.print("Send Status: ");
  if (result == ESP_OK)
  {
    Serial.println("Success");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_INIT)
  {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  }
  else if (result == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Invalid Argument");
  }
  else if (result == ESP_ERR_ESPNOW_INTERNAL)
  {
    Serial.println("Internal Error");
  }
  else if (result == ESP_ERR_ESPNOW_NO_MEM)
  {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.println("Peer not found.");
  }
  else
  {
    Serial.println("Not sure what happened");
  }
  digitalWrite(SEND_PIN, LOW);
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  digitalWrite(CONFIRM_PIN, HIGH);
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status != ESP_NOW_SEND_SUCCESS) {
    packetRecieved = true;
  }
  digitalWrite(CONFIRM_PIN, LOW);
}

// config AP SSID
void configDeviceAP()
{
  const char *SSID = SLAVE_SSID;
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL1, 0);
  if (!result)
  {
    Serial.println("AP Config failed.");
  }
  else
  {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  digitalWrite(RECEIVE_PIN, HIGH);
  dataRecieveFlag = true;
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: ");
  Serial.println(macStr);
  Serial.print("Last Packet Recv Data: ");
  Serial.println(*data);
  Serial.println("");
  number = data[0];
  String logText = "Number: " + (String)number;
  logPrintln(logText);
  dataRecieveFlag = false;
  packetRecieved = true;
  digitalWrite(RECEIVE_PIN, LOW);
}

void setup()
{
  pinMode(SCAN_PIN, OUTPUT);  digitalWrite(SCAN_PIN, LOW); // ScanForSlave
  pinMode(MANAGE_PIN, OUTPUT);  digitalWrite(MANAGE_PIN, LOW); // manageSlave
  pinMode(SEND_PIN, OUTPUT);  digitalWrite(SEND_PIN, LOW); // sendData
  pinMode(RECEIVE_PIN, OUTPUT);  digitalWrite(RECEIVE_PIN, LOW); // dataRecieve
  pinMode(CONFIRM_PIN, OUTPUT);  digitalWrite(CONFIRM_PIN, LOW); // sentConfirm
  Serial.begin(115200);
  screenInit();
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_MODE_APSTA);
  // configure device AP mode
  // configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  // Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // logPrintln("AP MAC: "); logPrintln(WiFi.softAPmacAddress());
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  logPrintln("STA MAC: "); logPrintln(WiFi.macAddress());
  // logPrintln("");
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_send_cb(OnDataSent); // Does order here matter?
  esp_now_register_recv_cb(OnDataRecv);
  // typingMotion = true;
  slaveAssign(STA_MAC_BLACK);
  packetRecieved = false;
}

unsigned long nextManage;

void loop() 
{
  // In the loop we scan for slave
  // ScanForSlave();
  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (slave.channel == CHANNEL2)
  { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    bool isPaired = manageSlave();
    nextManage = millis() + 10000;
    while (isPaired)
    {
      // logType();

      if (packetRecieved)
      {
        packetRecieved = false;
        // Send data to device
        sendData();
      }

      unsigned long t = millis();
      if (t > nextManage)
      {
        nextManage = t + 10000;
        // pair success or already paired
        isPaired = manageSlave();
      }
    }
    Serial.println("Slave pair failed!");
  }
}
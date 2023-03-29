

#include <FS.h> //this needs to be first, or it all crashes and burns...
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson (install v5.x)
#include <ArtNode.h>              // https://github.com/tobiasebsen/ArtNode
#include <ESP_DoubleResetDetector.h> //https://github.com/khoih-prog/ESP_DoubleResetDetector

#define OEM_CODE 0x2BFA

char hostname[64] = "lonestar-";

#define BUFFERSIZE 1024
#define VERSION_HI 0
#define VERSION_LO 1
//#define LED1 2 // D1
#define LED1 15  //D8 ESP-WROOM-02
#define LED2 13  //D7 ESP-WROOM-02
#define LED3 12  //D6 ESP-WROOM-02
#define LED4 14  //D5 ESP-WROOM-02

#define LED_onboard 16 // ESP_WROOM_02
// #define LED_onboard 2 // D1
#define USE_LITTLEFS            true
#define ESP_DRD_USE_LITTLEFS    true
#define ESP_DRD_USE_SPIFFS      false
#define ESP_DRD_USE_EEPROM      false
// #define DOUBLERESETDETECTOR_DEBUG       true  //false
#define DRD_TIMEOUT 10
#define DRD_ADDRESS 0
DoubleResetDetector *drd;



// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

byte buffer[BUFFERSIZE];
int pwmRange = 26000;

char universe[2] = "1";

char dmx_msb1[3] = "1";
char dmx_lsb1[3] = "2";
char dmx_msb2[3] = "3";
char dmx_lsb2[3] = "4";
char dmx_msb3[3] = "5";
char dmx_lsb3[3] = "6";
char dmx_msb4[3] = "7";
char dmx_lsb4[3] = "8";

int int_universe = 0;
int int_dmx_msb1 = 0;
int int_dmx_lsb1 = 0;
int int_dmx_msb2 = 0;
int int_dmx_lsb2 = 0;
int int_dmx_msb3 = 0;
int int_dmx_lsb3 = 0;
int int_dmx_msb4 = 0;
int int_dmx_lsb4 = 0;

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

const IPAddress gateway(2,0,0,1); 
const IPAddress subnet(255,0,0,0); // this needs to be 255.0.0.0 for artpoll to work

IPAddress subnetmask;
IPAddress localIp;
WiFiUDP udp;

ArtNode node;

ArtConfig ArtnetConfig = {
  .mac = { 0, 0, 0, 0, 0, 0 },
  .ip = { 0, 0, 0, 0 },
  .mask = { 0, 0, 0, 0 },
  .udpPort = 0x1936,
  .dhcp = false,
  .net = 0,
  .subnet = 0,
// .shortName //short device name
// .longName //long device name
  .numPorts = 1,
  .portTypes = {
    PortTypeDmx | PortTypeInput,
    PortTypeDmx | PortTypeInput,
    PortTypeDmx | PortTypeInput,
    PortTypeDmx | PortTypeInput
  },
  .portAddrIn = { 10, 10, 10, 10 }, // (DMX universe - 1)
  .portAddrOut = { 0, 0, 0, 0 },
  .verHi = VERSION_HI,
  .verLo = VERSION_LO
};


void printArtnetConfig()
{
  Serial.printf(
    "ArtnetConfig.mac         = 0x%x:0x%x:0x%x:0x%x:0x%x:0x%x \r\n"
    "ArtnetConfig.ip          = %d.%d.%d.%d \r\n"
    "ArtnetConfig.mask        = %d.%d.%d.%d \r\n"
    "ArtnetConfig.udpPort     = %d \r\n"
    "ArtnetConfig.dhcp        = %s \r\n"
    "ArtnetConfig.net         = %d \r\n"
    "ArtnetConfig.subnet      = %d \r\n"
    "ArtnetConfig.shortName   = %s \r\n"
    "ArtnetConfig.longName    = %s \r\n"
    "ArtnetConfig.numPorts    = %d \r\n"
    "ArtnetConfig.portTypes   = 0x%x, 0x%x, 0x%x, 0x%x \r\n"
    "ArtnetConfig.portAddrIn  = 0x%x, 0x%x, 0x%x, 0x%x \r\n"
    "ArtnetConfig.portAddrOut = 0x%x, 0x%x, 0x%x, 0x%x \r\n"
    "ArtnetConfig.verHi       = %d \r\n"
    "ArtnetConfig.verLi       = %d \r\n",
    ArtnetConfig.mac[0], ArtnetConfig.mac[1], ArtnetConfig.mac[2], ArtnetConfig.mac[3], ArtnetConfig.mac[4], ArtnetConfig.mac[5],
    ArtnetConfig.ip[0], ArtnetConfig.ip[1], ArtnetConfig.ip[2], ArtnetConfig.ip[3],
    ArtnetConfig.mask[0], ArtnetConfig.mask[1], ArtnetConfig.mask[2], ArtnetConfig.mask[3],
    ArtnetConfig.udpPort,
    ArtnetConfig.dhcp ? "True" : "False",
    ArtnetConfig.net,
    ArtnetConfig.subnet,
    ArtnetConfig.shortName,
    ArtnetConfig.longName,
    ArtnetConfig.numPorts,
    ArtnetConfig.portTypes[0], ArtnetConfig.portTypes[1], ArtnetConfig.portTypes[2], ArtnetConfig.portTypes[3],
    ArtnetConfig.portAddrIn[0], ArtnetConfig.portAddrIn[1], ArtnetConfig.portAddrIn[2], ArtnetConfig.portAddrIn[3],
    ArtnetConfig.portAddrOut[0], ArtnetConfig.portAddrOut[1], ArtnetConfig.portAddrOut[2], ArtnetConfig.portAddrOut[3],
    ArtnetConfig.verHi,
    ArtnetConfig.verLo
  );
}

void setup() {
  
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED_onboard, OUTPUT);
  digitalWrite(LED_onboard, LOW);
  analogWriteFreq(500);
  analogWriteRange(pwmRange);
  Serial.begin(115200);
  
  //clean FS, for testing
  // SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          strcpy(universe, json["universe"]);
          strcpy(dmx_msb1, json["dmx_msb1"]);
          strcpy(dmx_lsb1, json["dmx_lsb1"]);
          strcpy(dmx_msb2, json["dmx_msb2"]);
          strcpy(dmx_lsb2, json["dmx_lsb2"]);
          strcpy(dmx_msb3, json["dmx_msb3"]);
          strcpy(dmx_lsb3, json["dmx_lsb3"]);
          strcpy(dmx_msb4, json["dmx_msb4"]);
          strcpy(dmx_lsb4, json["dmx_lsb4"]);

          int_universe = atoi(universe);
          int_dmx_msb1 = atoi(dmx_msb1);
          int_dmx_lsb1 = atoi(dmx_lsb1);
          int_dmx_msb2 = atoi(dmx_msb2);
          int_dmx_lsb2 = atoi(dmx_lsb2);
          int_dmx_msb3 = atoi(dmx_msb3);
          int_dmx_lsb3 = atoi(dmx_lsb3);
          int_dmx_msb4 = atoi(dmx_msb4);
          int_dmx_lsb4 = atoi(dmx_lsb4);
          
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
  
  WiFiManagerParameter custom_universe("artnet_universe", "artnet_universe", universe, 2);
  WiFiManagerParameter custom_dmx_msb1("dmx_msb1", "dmx_msb1", dmx_msb1, 3);
  WiFiManagerParameter custom_dmx_lsb1("dmx_lsb1", "dmx_lsb1", dmx_lsb1, 3);
  WiFiManagerParameter custom_dmx_msb2("dmx_msb2", "dmx_msb2", dmx_msb2, 3);
  WiFiManagerParameter custom_dmx_lsb2("dmx_lsb2", "dmx_lsb2", dmx_lsb2, 3);
  WiFiManagerParameter custom_dmx_msb3("dmx_msb3", "dmx_msb3", dmx_msb3, 3);
  WiFiManagerParameter custom_dmx_lsb3("dmx_lsb3", "dmx_lsb3", dmx_lsb3, 3);
  WiFiManagerParameter custom_dmx_msb4("dmx_msb4", "dmx_msb4", dmx_msb4, 3);
  WiFiManagerParameter custom_dmx_lsb4("dmx_lsb4", "dmx_lsb4", dmx_lsb4, 3);


  // config static IP address from OEM_CODE and MAC address
  uint16_t oem_code_16bit = OEM_CODE;
  byte oem_code []= { oem_code_16bit >> 8 , oem_code_16bit & 0x00FF };
  byte wifimac[6];
  WiFi.macAddress(wifimac);
  uint8_t ipB = (oem_code[0]+oem_code[1]+wifimac[3]);
  const IPAddress ip(2,ipB,wifimac[4],wifimac[5]); // static IP computed according to Artistic License white paper

  WiFiManager wifiManager;
  //  wifiManager.setDebugOutput(false);
  wifiManager.setWiFiAutoReconnect(true);
  wifiManager.setSTAStaticIPConfig(ip, gateway, subnet);
 String  hostName = "lonestar-" + String(wifimac[0],HEX) + String(wifimac[1],HEX) + String(wifimac[2],HEX) + String(wifimac[3],HEX) + String(wifimac[4],HEX) + String(wifimac[5],HEX);
  wifiManager.setHostname(hostName);
  WiFiManagerParameter custom_text_box("my_text", "ExM lonestar config", "default string", 50);
   std::vector<const char *> wm_menu  = {"wifi", "restart"};
//  wifiManager.setConfigPortalTimeout(120);
  wifiManager.setShowInfoUpdate(false);
  wifiManager.setShowInfoErase(false);
  wifiManager.setShowStaticFields(false);
  wifiManager.setMenu(wm_menu);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_universe);
  wifiManager.addParameter(&custom_dmx_msb1);
  wifiManager.addParameter(&custom_dmx_lsb1);
  wifiManager.addParameter(&custom_dmx_msb2);
  wifiManager.addParameter(&custom_dmx_lsb2);
  wifiManager.addParameter(&custom_dmx_msb3);
  wifiManager.addParameter(&custom_dmx_lsb3);
  wifiManager.addParameter(&custom_dmx_msb4);
  wifiManager.addParameter(&custom_dmx_lsb4);
  
  // Uncomment and run it once, if you want to erase all the stored information
  // wifiManager.resetSettings();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  wifiManager.autoConnect("lonestar config"); // captive portal SSID
 digitalWrite(LED_onboard, HIGH);
  
  // if you get here you have connected to the WiFi
  Serial.println("Connected.");
  
  strcpy(universe, custom_universe.getValue());
  strcpy(dmx_msb1, custom_dmx_msb1.getValue());
  strcpy(dmx_lsb1, custom_dmx_lsb1.getValue());
  strcpy(dmx_msb2, custom_dmx_msb2.getValue());
  strcpy(dmx_lsb2, custom_dmx_lsb2.getValue());
  strcpy(dmx_msb3, custom_dmx_msb3.getValue());
  strcpy(dmx_lsb3, custom_dmx_lsb3.getValue());
  strcpy(dmx_msb4, custom_dmx_msb4.getValue());
  strcpy(dmx_lsb4, custom_dmx_lsb4.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["universe"] = universe;
    json["dmx_msb1"] = dmx_msb1;
    json["dmx_lsb1"] = dmx_lsb1;
    json["dmx_msb2"] = dmx_msb2;
    json["dmx_lsb2"] = dmx_lsb2;
    json["dmx_msb3"] = dmx_msb3;
    json["dmx_lsb3"] = dmx_lsb3;
    json["dmx_msb4"] = dmx_msb4;
    json["dmx_lsb4"] = dmx_lsb4;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }
    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
Serial.print("Universe : ");
Serial.println(universe);
Serial.print("dmx_msb1 : ");
Serial.println(dmx_msb1);
Serial.print("dmx_lsb1 : ");
Serial.println(dmx_lsb1);
Serial.print("dmx_msb2 : ");
Serial.println(dmx_msb2);
Serial.print("dmx_lsb2 : ");
Serial.println(dmx_lsb2);
Serial.print("dmx_msb3 : ");
Serial.println(dmx_msb3);
Serial.print("dmx_lsb3 : ");
Serial.println(dmx_lsb3);
Serial.print("dmx_msb4 : ");
Serial.println(dmx_msb4);
Serial.print("dmx_lsb4 : ");
Serial.println(dmx_lsb4);

  
  server.begin();

    for (int i = 0; i < 6; i++) ArtnetConfig.mac[i] = wifimac[i];

char macStr[13];
snprintf(macStr, sizeof(macStr), "%02X%02X%02X%02X%02X%02X",
wifimac[0], wifimac[1], wifimac[2], wifimac[3], wifimac[4], wifimac[5]);
char longname[64];
sprintf(longname,"%s%s",hostname,macStr);
strcpy(ArtnetConfig.shortName, macStr);
strcpy(ArtnetConfig.longName, longname); //https://stackoverflow.com/questions/2218290/concatenate-char-array-in-c
    
  // set the ip/broadcast address in the ArtnetConfig.
  localIp = WiFi.localIP();
  subnetmask = WiFi.subnetMask();
  for (int i = 0; i < 4; i++) ArtnetConfig.ip[i] = localIp[i];
  for (int i = 0; i < 4; i++) ArtnetConfig.mask[i] = subnetmask[i];

  // initialize the artnet node.
  printArtnetConfig();
  node = ArtNode(ArtnetConfig, BUFFERSIZE, buffer);

  // start udp listening service on the artnet port.
  udp.begin(ArtnetConfig.udpPort);


  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
  if (drd->detectDoubleReset())
  {
    Serial.println("Resetting settings as there was a double reset detected");
    wifiManager.resetSettings();
    ESP.restart();
  }
  
}

void loop(){
    drd->loop();

  // check for udp data.
  while (udp.parsePacket())
  {
    int n = udp.read(buffer, min(udp.available(), BUFFERSIZE));
    // check if there is atleast header data and if it is a valid packet.
    if (n >= sizeof(ArtHeader) && node.isPacketValid())
    {
      // parse opcodes so that we can act accordingly
      switch (node.getOpCode())
      {
        case OpPoll:
          {
            // construct a ArtPoll reply
            ArtPoll* poll = (ArtPoll*)buffer;
            node.createPollReply();

            // send the reply right back to the remote (no need to broadcast)
            udp.beginPacket(udp.remoteIP(), ArtnetConfig.udpPort);
            udp.write(buffer, sizeof(ArtPollReply));
            udp.endPacket();
          } break;
        case OpDmx: {
            // parse dmx data.
            ArtDmx* dmx = (ArtDmx *)buffer;
            int port = node.getPort(dmx->Net, dmx->SubUni);
            int len = dmx->getLength();

            // access pointer to the data.
            byte *data = dmx->Data;

            int BigDMX1 = ((data[int_dmx_msb1-1] << 8) + data[int_dmx_lsb1-1]);
            int BigDMX2 = ((data[int_dmx_msb2-1] << 8) + data[int_dmx_lsb2-1]);
            int BigDMX3 = ((data[int_dmx_msb3-1] << 8) + data[int_dmx_lsb3-1]);
            int BigDMX4 = ((data[int_dmx_msb4-1] << 8) + data[int_dmx_lsb4-1]);

 //        write value to LED as PWM and remap the whole 65535 range to max PWM range of this board (pwmRange)
           analogWrite(LED1, map(BigDMX1, 0, 65536, 0, pwmRange));
           analogWrite(LED2, map(BigDMX2, 0, 65536, 0, pwmRange));
           analogWrite(LED3, map(BigDMX3, 0, 65536, 0, pwmRange));
           analogWrite(LED4, map(BigDMX4, 0, 65536, 0, pwmRange));

          } break;
      }
    }
  }

}

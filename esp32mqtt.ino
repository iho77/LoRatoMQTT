#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include "SSD1306.h" 
#include "images.h"
#include <WiFi.h>
#include <PubSubClient.h>

// Pin definetion of WIFI LoRa 32
// HelTec AutoMation 2017 support@heltec.cn 
#define SCK     5    // GPIO5  -- SX127x's SCK
#define MISO    19   // GPIO19 -- SX127x's MISO
#define MOSI    27   // GPIO27 -- SX127x's MOSI
#define SS      18   // GPIO18 -- SX127x's CS
#define RST     14   // GPIO14 -- SX127x's RESET
#define DI00    26   // GPIO26 -- SX127x's IRQ(Interrupt Request)

#define BAND    433E6  //you can set band here directly,e.g. 868E6,915E6
#define PABOOST true

typedef struct {
  byte src_addr;
  char msg[255];
  unsigned long count;
 } _l_packet;

_l_packet pkt;
boolean gotpacket;


SSD1306 display(0x3c, 4, 15);

String displaymessages[5] = {"str1","str2","str3","str4","str5"};



void logo(){
  display.clear();
  display.drawXbm(0,5,logo_width,logo_height,logo_bits);
  display.display();
}


 
const char* ssid = "iho-iot";
const char* password =  "pkjgjk_wifi";
const char* mqttServer = "192.168.1.110";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";
 
WiFiClient espClient;
PubSubClient client(espClient);
 
void setup() {
 
  Serial.begin(115200);

  
  gotpacket = false;
  pkt.src_addr = 0;
  //pkt.msg;
  pkt.count = 0;
  initdisplay();
  initwifi();
  initlora();
  initmqtt();
  LoRa.receive();
  
 
}
 
void loop() {
  String topic;
  client.loop();

  
  if (WiFi.status() == WL_CONNECTED) {
    displaymessages[0] = "WiFi OK, RSSI :"+String(WiFi.RSSI());
    } else { displaymessages[0] = "WiFi disconnected";}

  
  if (client.connected()) {
    displaymessages[1] = "MQTT connected";
    } else { displaymessages[1] = "MQTT disconnected";}

      
  if (gotpacket){
    gotpacket = false;
    topic = "iho-iot/"+String(pkt.src_addr);
    client.publish(topic.c_str(),(const char *)&pkt.msg[0]);
    displaymessages[4] = pkt.msg;
    displaymessages[3] = "Got pkt#"+String(pkt.count)+" RSSI:"+String(LoRa.packetRssi());
    displaymessages[2] = "LoRa SNR"+String(LoRa.packetSnr());
  }

  statusDisplay();
 
}

void messageLog(const char *msg){
     display.clear();
     display.setTextAlignment(TEXT_ALIGN_LEFT);
     display.setFont(ArialMT_Plain_10);
     display.drawString(0, 0, msg);
     display.display();
}

void statusDisplay(){
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  for (int i = 0; i<=3; i++){
    display.drawString(1 , 12*i , displaymessages[i]);
  }
  display.drawStringMaxWidth(0 , 48 , 128, displaymessages[4]);
  display.display();
}

void initdisplay(){
     pinMode(16,OUTPUT);
     pinMode(25,OUTPUT);
     digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
     delay(50); 
     digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
     display.init();
     display.flipScreenVertically();  
     display.setFont(ArialMT_Plain_10);
     logo();
     delay(1500);
     display.clear();
}


void initwifi(){
  
  WiFi.begin(ssid, password);
  while ( WiFi.status() !=  WL_CONNECTED) {
     messageLog("Connecting to WiFi..");
  }

    messageLog("Connected to the WiFi network");
    delay(1000);
}


void initmqtt(){
  
  client.setServer(mqttServer, mqttPort);
 
  while (!client.connect("ESP32Client", mqttUser, mqttPassword )) {
    messageLog("Connecting to MQTT...");
 
    }
   messageLog("Connected to MQTT... ");
   delay(1000);
}


void initlora(){
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI00);
  
  while (!LoRa.begin(BAND,PABOOST))
  {
    messageLog("Initialising LoRa module....");
  }
  
  messageLog("LoRa Init success!");
  delay(1000);
  
  LoRa.onReceive(onReceive);
  
 }


void onReceive(int packetSize)
{
  
 // byte *buff;
 // buff = (byte *)malloc(packetSize);
 // LoRa.readBytes(buff,packetSize);
  LoRa.readBytes((uint8_t *)&pkt,packetSize); 
  gotpacket = true;
  //memcpy((void *)&pkt.src_addr,(void *)buff,1);
  //memcpy((void *)&pkt.msg,(void *)(buff+1),packetSize-1);
  pkt.msg[packetSize-1]='\0';
  pkt.count++;
  //free(buff);
  
}






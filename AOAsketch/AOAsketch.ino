
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <adk.h>
#include "DHT.h"

#define DHTPIN 4     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define PKT_UPDATE 0x05 
#define PKT_CMD 0x06 

#define RFTYPE_REGISTER 1
#define RFTYPE_ACCEPTED 2
#define RFTYPE_UPDATE 3
#define RFTYPE_CMD 4
#define RFTYPE_LED 5


#define CMD_SOLENOID   0x10

#define CMD_SET_RED    0x50
#define CMD_SET_GREEN  0x51
#define CMD_SET_BLUE   0x52
#define CMD_SET_ALL    0x53

#define PIN_SOLENOID 6

boolean solenoidState = false;
DHT dht(DHTPIN, DHTTYPE);
float humidityVal;
float temperatureVal;
long humidLong;
long tempLong;
uint8_t recMsg[64];
byte lightByte;
byte currentRed,currentGreen, currentBlue;
USB Usb;
ADK adk(&Usb, "Electric Green Thumb", // Manufacturer Name
"Scarecrow", // Model Name
"Electric Scarecrow", // Description (user-visible string)
".1", // Version
"http://www.bskenyon.com", // URL (web page to visit if no installed apps support the accessory)
"123456789"); // Serial Number (optional)

uint32_t timer;

struct NODE{
  byte nodeNumber;
  char address[5];
};

byte nodeSoil[4];
int nodeTemp[4];
byte nodeLight[4];
int nodeBat[4];

NODE clients[4];
uint8_t numClients = 0;

struct USB_PACKET{
  byte type;
  union {
    struct UPDATE{
      uint8_t nodes;
      uint8_t soilSensor[2];
      uint8_t light[2];
      uint16_t temperature[2];
      uint32_t baseTemp;
      uint32_t baseHumid;
      uint8_t baseLight;
      uint8_t rValue;
      uint8_t gValue;
      uint8_t bValue;
    }
    UPDATE;
    struct CMD{
      uint8_t arguments[6];
    }
    CMD;
    struct LED{
      uint8_t color;
      uint8_t value;
      uint8_t arguments[6];
    }
    LED;
    uint8_t asBytes[6];
  }
  pktTypes; 
};
USB_PACKET outPacket;


struct RF_PACKET{
  byte type;
  union {
    struct INIT{
      NODE newNode;
    }
    INIT;
    struct UPDATE{
      byte nodeNumber;
      uint8_t soilSensor;
      uint8_t battery;
      uint8_t light;
      uint16_t temperature;
    }
    UPDATE;
    struct CMD{
      uint8_t arguments[5];
    }
    CMD;
    struct LED{
      uint8_t color;
      uint8_t value;
      uint8_t arguments[3];
    }
    LED;
    uint8_t asBytes[6];
  }
  pktTypes; 
};

RF_PACKET packet;
RF_PACKET pktBuffer[5];
byte pktBufferPt = 0;

void setLED(unsigned int color, int value){
  switch(color){
  case CMD_SET_RED:
    Serial.print("RED SET: 0x");
    currentRed = value&0xFF;
    break;
  case CMD_SET_GREEN:
    Serial.print("GREEN SET: 0x");
    currentGreen = value&0xFF;
    break;
  case CMD_SET_BLUE:
    Serial.print("BLUE SET: 0x");
    currentBlue = value&0xFF;
    break;
  }
  if (numClients>0){

  Mirf.setTADDR((byte *)clients[0].address);
  packet.type = RFTYPE_LED;
  packet.pktTypes.LED.color = color&0xFF;
  packet.pktTypes.LED.value = value&0xFF;
  Mirf.send((byte *)&packet);

  /* Wait for module to finish sending */
  while(Mirf.isSending());      
  Serial.print(value);
}else{
      Serial.print("No Nodes");
}  
}

void rf_interupt(){

}

void setup() {
  pktBufferPt = 0;
  Serial.begin(9600);
  dht.begin();
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print("\r\nOSCOKIRQ failed to assert");
    while (1); // halt
  }
  Serial.print("\r\nArduino Started");
  pinMode(PIN_SOLENOID, OUTPUT);
  digitalWrite(PIN_SOLENOID, true);

  //Setup RF Transiever
  /* Define the DIO used for SPI CSN & CE. Defaults are DIO
   *  8 & 7 respectively. */
  Mirf.cePin = 7;
  Mirf.csnPin = 8;

  Mirf.spi = &MirfHardwareSpi;

  /* Initialise the module */
  Mirf.init();

  /* Set the receiving address of this module. This must be a 5 byte
   *  character string */
  Mirf.setRADDR((byte *)"BASE1");

  /* Set the payload length to size unsigned int (size of data from
   *  the analogue input A0) */
  Mirf.payload = sizeof(packet);

  /* Set the channel. Channel 90 = 2.490GHz */
  Mirf.channel = 90;

  /* Configure the module */
  Mirf.config();

  // attachInterrupt(1,rf_interupt,LOW);

}

void setSolenoid(byte solenoid, byte state) {
  Serial.print("\r\nSolenoid: ");
  Serial.print(solenoid);
  if (state == 1) {
    if (!solenoidState) {
      Serial.print("Activated");
      solenoidState = true;
      digitalWrite(PIN_SOLENOID, false);
    }
  } 
  else if (state == 0) {
    if (solenoidState) {
      solenoidState = false;
      Serial.print("Deactivated");
      digitalWrite(PIN_SOLENOID, true);
    }
  } 
  else {
    Serial.print("Error: ");
    Serial.print(state);
  }
}

void loop() {
  if (pktBufferPt>0){
    pktBufferPt--;

    Mirf.setTADDR((byte *)pktBuffer[pktBufferPt].pktTypes.INIT.newNode.address);
    //Mirf.setTADDR((byte *)"NODE1");
    Mirf.send((byte *)&pktBuffer[pktBufferPt]);
    while(Mirf.isSending());
    Serial.print("Send Response to :");
    Serial.println(pktBuffer[pktBufferPt].pktTypes.INIT.newNode.address);
  }
  if(!Mirf.isSending() && Mirf.dataReady()){
    Serial.println("Got packet");
    Mirf.getData((byte*)&packet);
    Serial.print("Type: ");
    Serial.println(packet.type);
    switch(packet.type){
    case RFTYPE_REGISTER:
      for (int i=0; i<numClients; i++){
        if (strncmp(clients[i].address, (const char*)packet.pktTypes.INIT.newNode.address,5)  == 0){
          packet.type = RFTYPE_ACCEPTED;
          packet.pktTypes.INIT.newNode.nodeNumber = clients[i].nodeNumber;
          Serial.print("Node ");
          Serial.println(i+1);
          Serial.println("Already registered");
          pktBuffer[pktBufferPt++] = packet;
          //   Mirf.setTADDR((byte *)clients[i].address);
          //   Mirf.send((byte *)&packet);
          //while(Mirf.isSending()); 
          // Serial.println("Responded to node");
          return;
        }
      }
      packet.type = RFTYPE_ACCEPTED;
      packet.pktTypes.INIT.newNode.nodeNumber = numClients+1;
      clients[numClients++] = packet.pktTypes.INIT.newNode;
      Serial.println("New Node Registered");
      Serial.print("Address:");
      Serial.println(packet.pktTypes.INIT.newNode.address);
      pktBuffer[pktBufferPt++] = packet;
      //Mirf.send((byte *)&packet);
      //while(Mirf.isSending());    
      //erial.println("Responded to Node");       
      break;     

    case RFTYPE_UPDATE:
      if(packet.pktTypes.UPDATE.nodeNumber>0){
        nodeTemp[packet.pktTypes.UPDATE.nodeNumber-1] = packet.pktTypes.UPDATE.temperature;
        nodeSoil[packet.pktTypes.UPDATE.nodeNumber-1] = packet.pktTypes.UPDATE.soilSensor;
        nodeLight[packet.pktTypes.UPDATE.nodeNumber-1] = packet.pktTypes.UPDATE.light;
        nodeBat[packet.pktTypes.UPDATE.nodeNumber-1] = packet.pktTypes.UPDATE.battery;
        Serial.print("Node ");
        Serial.print(packet.pktTypes.UPDATE.nodeNumber);
        Serial.println(" Data Recieved");
        Serial.print("Tenp: ");
        Serial.println(nodeTemp[packet.pktTypes.UPDATE.nodeNumber-1]);
        Serial.print("Soil Moisture: ");
        Serial.println(nodeSoil[packet.pktTypes.UPDATE.nodeNumber-1]);
        Serial.print("Light Sensor: ");
        Serial.println(nodeLight[packet.pktTypes.UPDATE.nodeNumber-1]);
        Serial.print("Battery: ");
        Serial.println(nodeBat[packet.pktTypes.UPDATE.nodeNumber-1]);
      }
      break;
    }
  }

  Usb.Task();
  if (adk.isReady()) {
    uint16_t len = sizeof (recMsg);
    uint8_t rcode = adk.RcvData(&len, recMsg);
    if (rcode && rcode != hrNAK) {
      Serial.print(F("\r\nData rcv: "));
      Serial.print(rcode, HEX);
    } 
    else if (len > 0) {
      switch (recMsg[0]) {
      case PKT_CMD:
        switch (recMsg[1]) {
        case CMD_SOLENOID:
          setSolenoid(recMsg[2], recMsg[3]);
          break;
        case CMD_SET_RED:
          setLED(recMsg[1], recMsg[2]);
          break;
        case CMD_SET_GREEN:
          setLED(recMsg[1], recMsg[2]);
          break;
        case CMD_SET_BLUE:
          setLED(recMsg[1], recMsg[2]);
          break;
          case CMD_SET_ALL:
          setLED(CMD_SET_RED, recMsg[2]);
          setLED(CMD_SET_GREEN, recMsg[3]);
          setLED(CMD_SET_BLUE, recMsg[4]);
break;
        }
        break;
      }
    }

    if (millis() - timer >= 60000) { // Send data every 1s
      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
      humidityVal = dht.readHumidity();
      temperatureVal = dht.readTemperature();
      humidLong = (long) (humidityVal * 1000);
      tempLong = (long) (temperatureVal * 1000);
      lightByte = 55;
      // check if returns are valid, if they are NaN (not a number) then something went wrong!
      if (isnan(temperatureVal) || isnan(humidityVal)) {
        Serial.println("Failed to read from DHT");
      }
      outPacket.type = PKT_UPDATE;
      outPacket.pktTypes.UPDATE.nodes = numClients;
      outPacket.pktTypes.UPDATE.baseTemp = tempLong;
      outPacket.pktTypes.UPDATE.baseHumid = humidLong;
      outPacket.pktTypes.UPDATE.baseLight = lightByte;
      outPacket.pktTypes.UPDATE.soilSensor[0] = nodeSoil[0];
      outPacket.pktTypes.UPDATE.soilSensor[1] = nodeSoil[1];
      outPacket.pktTypes.UPDATE.light[0] = nodeLight[0];
      outPacket.pktTypes.UPDATE.light[1] = nodeLight[1];
      outPacket.pktTypes.UPDATE.soilSensor[0] = nodeSoil[0];
      outPacket.pktTypes.UPDATE.soilSensor[1] = nodeSoil[1];
      outPacket.pktTypes.UPDATE.temperature[0] = nodeTemp[0];
      outPacket.pktTypes.UPDATE.temperature[1] = nodeTemp[1];
      outPacket.pktTypes.UPDATE.rValue = currentRed;
      outPacket.pktTypes.UPDATE.gValue = currentGreen;
      outPacket.pktTypes.UPDATE.bValue = currentBlue;

      timer = millis();
      rcode = adk.SndData(sizeof (outPacket), (uint8_t*) & outPacket);
      Serial.print("Humidity: ");
      Serial.print(humidityVal);
      Serial.print(" %\t");
      Serial.print("Temperature: ");
      Serial.print(temperatureVal);
      Serial.println(" *C");
    }
  }
}




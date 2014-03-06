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


#define CMD_SOLENOID 0x10
#define CMD_LED 0x20

#define PIN_SOLENOID 2

boolean solenoidState = false;
DHT dht(DHTPIN, DHTTYPE);
float humidityVal;
float temperatureVal;
long humidLong;
long tempLong;
uint8_t recMsg[64];

USB Usb;
ADK adk(&Usb, "Electric Green Thumb", // Manufacturer Name
"Scarecrow", // Model Name
"Electric Scarecrow", // Description (user-visible string)
".1", // Version
"http://www.bskenyon.com", // URL (web page to visit if no installed apps support the accessory)
"123456789"); // Serial Number (optional)

uint32_t timer;
uint8_t testPacket[32];

struct NODE{
  byte nodeNumber;
  uint8_t address[5];
};

NODE clients[4];
uint8_t numClients = 0;

struct PACKET{
  byte type;
  union {
    struct INIT{
        NODE newNode;
      }
    INIT;
    struct UPDATE{
      byte nodeNumber;
      uint8_t soilSensor1;
      uint8_t soilSensor2;
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
  }pktTypes; 
};

PACKET packet;

#define RGB_RED    3
#define RGB_GREEN  5
#define RGB_BLUE   6

void setLED(unsigned int color, byte value){
  switch(color){
  case RGB_RED:
    Serial.print("RED SET: 0x");
    break;
  case RGB_GREEN:
    Serial.print("GREEN SET: 0x");
    break;
  case RGB_BLUE:
    Serial.print("BLUE SET: 0x");
    break;
  }
  Mirf.setTADDR((byte *)"clie1");
  packet.pktTypes.LED.color = color&0xFF;
  packet.pktTypes.LED.value = value&0xFF;
  Mirf.send((byte *)&packet);

  /* Wait for module to finish sending */
  while(Mirf.isSending());                 
}

void rf_interupt(){
  if(!Mirf.isSending() && Mirf.dataReady()){
    Serial.println("Got packet");
    Mirf.getData((byte*)&packet);
    switch(packet.type){
    case RFTYPE_REGISTER:
    for (int i=0; i<numClients; i++){
          if (clients[i].address == packet.pktTypes.INIT.newNode.address){
            packet.type = RFTYPE_ACCEPTED;
            packet.pktTypes.INIT.newNode.nodeNumber = clients[i].nodeNumber;
              Mirf.send((byte *)&packet);
              return;
          }
    }
    packet.type = RFTYPE_ACCEPTED;
    packet.pktTypes.INIT.newNode.nodeNumber = numClients+1;
    clients[numClients++] = packet.pktTypes.INIT.newNode;
    Mirf.send((byte *)&packet);
    break;     
    }
  }
}

void setup() {
  Serial.begin(9600);
  dht.begin();
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print("\r\nOSCOKIRQ failed to assert");
    while (1); // halt
  }
  Serial.print("\r\nArduino Started");
  pinMode(PIN_SOLENOID, OUTPUT);
  digitalWrite(PIN_SOLENOID, false);

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
  
  attachInterrupt(1,rf_interupt,LOW);

}

void setSolenoid(byte solenoid, byte state) {
  Serial.print("\r\nSolenoid: ");
  Serial.print(solenoid);
  if (state == 1) {
    if (!solenoidState) {
      Serial.print("Activated");
      solenoidState = true;
      digitalWrite(PIN_SOLENOID, true);
      delay(20);
      digitalWrite(PIN_SOLENOID, false);
    }
  } 
  else if (state == 0) {
    if (solenoidState) {
      solenoidState = false;
      Serial.print("Deactivated");
      digitalWrite(PIN_SOLENOID, true);
      delay(20);
      digitalWrite(PIN_SOLENOID, false);
    }
  } 
  else {
    Serial.print("Error: ");
    Serial.print(state);
  }
}

void loop() {
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
        case CMD_LED:
          setLED(recMsg[2],recMsg[3]);
          break;
        }
        break;
      }
    }

    if (millis() - timer >= 2000) { // Send data every 1s
      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
      
                setLED(0x54,0x32);

      humidityVal = dht.readHumidity();
      temperatureVal = dht.readTemperature();
      humidLong = (long) (humidityVal * 1000);
      tempLong = (long) (temperatureVal * 1000);

      // check if returns are valid, if they are NaN (not a number) then something went wrong!
      if (isnan(temperatureVal) || isnan(humidityVal)) {
        Serial.println("Failed to read from DHT");
      }
      testPacket[0] = PKT_UPDATE;
      testPacket[1] = (tempLong >> 24)&0xFF;
      testPacket[2] = (tempLong >> 16)&0xFF;
      testPacket[3] = (tempLong >> 8)&0xFF;
      testPacket[4] = tempLong & 0xFF;

      testPacket[5] = (humidLong >> 24)&0xFF;
      testPacket[6] = (humidLong >> 16)&0xFF;
      testPacket[7] = (humidLong >> 8)&0xFF;
      testPacket[8] = humidLong & 0xFF;

      timer = millis();
      rcode = adk.SndData(sizeof (testPacket), (uint8_t*) & testPacket);
      Serial.print("Humidity: ");
      Serial.print(humidityVal);
      Serial.print(" %\t");
      Serial.print("Temperature: ");
      Serial.print(temperatureVal);
      Serial.println(" *C");
    }
  }
}



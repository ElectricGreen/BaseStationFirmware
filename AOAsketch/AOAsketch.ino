#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <adk.h>
#include <DHT.h>
#include <Wire.h>
#include <Stepper.h>
#include <usb.h>

#include "Defines.h"
#include "Structs.h"
#include "Pins.h"

//Variables
//DHT dht(PIN_DHT, DHTTYPE);

//USB Variables
USB Usb;
ADK adk(&Usb, "Electric Green Thumb", // Manufacturer Name
"Scarecrow", // Model Name
"Electric Scarecrow", // Description (user-visible string)
".1", // Version
"http://www.bskenyon.com", // URL (web page to visit if no installed apps support the accessory)
"123456789"); // Serial Number (optional)

uint8_t recMsg[64];
USB_PACKET outPacket;

//RF Variables
RF_PACKET packet;
RF_PACKET pktBuffer[5];
byte pktBufferPt = 0;

//BaseStation Status
boolean solenoidState[2] = {
  false,false};
int headDirection = 2; //Center Front
float humidityVal;
float temperatureVal;
long humidLong;
long tempLong;
byte lightByte;
byte currentRed,currentGreen, currentBlue;
byte exInput,exOutput;
long lastEvent;
uint32_t timer; //Count to Send Update
long motorTime;
long changeTime;
boolean oldIoIntVal,ioIntVal;
boolean secEnable;
boolean secLed;
boolean secSound;
int oldInput;

//Node Status
byte nodeSoil[MAX_CLIENTS];
int nodeTemp[MAX_CLIENTS];
byte nodeLight[MAX_CLIENTS];
int nodeBat[MAX_CLIENTS];

NODE clients[MAX_CLIENTS];
uint8_t numClients = 0;

//Stepper
Stepper stepper(MOTOR_STEPS, PIN_MOTOR1, PIN_MOTOR2, PIN_MOTOR3, PIN_MOTOR4);

//Functions Declarations
void setRGBLED(unsigned int color, int value);
void ioInterrupt();
void setSolenoid(byte solenoid, byte state);
void setPwmFrequency(int pin, int divisor);
void setSound(byte arg);
void setExOutput(int output, boolean value);


void setup() {
//  Serial.begin(9600);
//  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
//  Serial.println("\r\nArduino Started");

  //USB
  if (Usb.Init() == -1) {
    // Serial.print("\r\nOSCOKIRQ failed to assert");
    while (1); // halt
  }

  //InitVariables
  exInput = 0;
  exOutput = 0;
  pktBufferPt = 0;
  solenoidState[0] = false;
  solenoidState[1] = false;
  headDirection = 2; //center forward
  lastEvent = 0;
  motorTime = 0;
  changeTime = 0;
  secEnable = false;
  secLed = false;
  secSound = false;
  
  //dht.begin(); //Start DHT Sensor

  //Set PWM Frequency options 1(62.5khz),8(7.8khz),64,256,1024 
  pinMode(PIN_SOUND, OUTPUT); 
  //setPwmFrequency(PIN_SOUND,8);

  //Interrupt on IO Expander INT
  pinMode(PIN_IO_INT, INPUT); 
  digitalWrite(PIN_IO_INT, LOW);
  //PCintPort::attachInterrupt(PIN_IO_INT, &ioInterrupt, RISING);  // add more attachInterrupt code as required
  //Setup I2C Bus
  Wire.begin(); // wake up I2C bus

  Wire.beginTransmission(I2C_ADDRESS); //I2C Address
  Wire.write(IO_IODIRA); // IODIRA register
  Wire.write(0xFF); // set all of port A to inputs
  Wire.endTransmission();

  Wire.beginTransmission(I2C_ADDRESS); //I2C Address
  Wire.write(IO_GPINTENA); // Change notice register
  Wire.write(0x0F); // set first 4 inputs to interrupt
  Wire.endTransmission();

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(IO_IODIRB); // IODIRB register
  Wire.write(0x00); // set all of port B to outputs
  Wire.endTransmission();

  //Motor Speed - 30 RPMs
  pinMode(PIN_MOTOR_EN, OUTPUT); 

  digitalWrite(PIN_MOTOR_EN, MOTOR_OFF);
  stepper.setSpeed(30);


  //Setup RF Transiever
  Mirf.cePin = PIN_RF_CE;
  Mirf.csnPin = PIN_RF_CSN;
  Mirf.spi = &MirfHardwareSpi;

  /* Initialise the module */
  Mirf.init();
  Mirf.setRADDR((byte *)"BASE1"); //Receive Address
  Mirf.payload = sizeof(packet);
  Mirf.channel = 90; //Channel 90 = 2.490GHz
  Mirf.config();

    ioIntVal = digitalRead(PIN_IO_INT);
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(IO_GPIOA); // set MCP23017 memory pointer to GPIOA address
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDRESS, 1); // request one byte of data from MCP20317
    exInput=Wire.read(); // store the incoming byte into "inputs"
    oldIoIntVal = ioIntVal;
}

int transHeadDirection(int input){
  switch(input){
  case 1: //North
    return 2;

  case 2: //East
    return 4;

    break;
  case 3: //NE
    return 3;

  case 4: //South
    return 6;

  case 6: //SE
    return 5;

  case 8: //West
    return 8;

  case 12: //SW
    return 7;

  case 9://NW
    return 1;

  }
}
void setHeadDirection(int dir, boolean sec){
  int steps = 0;
  switch(dir){
  case 2: //North
    steps = 0;
    break;
  case 4: //East
    steps = 50;
    break;
  case 3: //NE
    steps = 25;
    break;
  case 6: //South
    steps = 100;
    break;
  case 5: //SE
    steps = 75;
    break;
  case 8: //West
    steps = -50;
    break;
  case 7: //SW
    steps = -75;
    break;
  case 1://NW
    steps = -25;
    break;    
  }
  digitalWrite(PIN_MOTOR_EN, MOTOR_ON);
  stepper.step(steps);
  
  if (sec){
    if (secLed)
      setExOutput(2, true); //Turn on LED
    if (secSound)
      setSound(true); //activate Noise
  }

  delay(5000); //Delay 5s to move head
  stepper.step(-steps); //turn back
  //delay(2000);//Delay 2s to move head
  digitalWrite(PIN_MOTOR_EN, MOTOR_OFF); 

  if (sec){
    if (secLed)
      setExOutput(2, false); //Turn off LED
    if (secSound)
      setSound(false); //deactivate Noise
  }

}
void loop() {
  
    long currentTime = millis();
    
    if ((changeTime+60000<currentTime)&&secEnable){ //Motion hasnt been detected for 2mins
    ioIntVal = digitalRead(PIN_IO_INT);
   
      if (!ioIntVal){ //Interrupt is high, read inputs
      //Serial.println("Interupt Low");
      Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(IO_GPIOA); // set MCP23017 memory pointer to GPIOA address
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDRESS, 1); // request one byte of data from MCP20317
    exInput=Wire.read(); // store the incoming byte into "inputs"
      
    if (exInput!=oldInput){//Yes a change did occur
          oldInput = exInput;
          if (exInput!=0){ //Are Any inputs high?
        //          if (secEnable){
        changeTime = currentTime; //Save Time
        //Send Alert Packet
        outPacket.type = PKT_ALERT;
        outPacket.pktTypes.ALERT.motionSense = transHeadDirection(exInput);
        adk.SndData(sizeof (outPacket), (uint8_t*) & outPacket);
        
        //Look and Activate Deterants
        setHeadDirection(outPacket.pktTypes.ALERT.motionSense,true);
        //}

    }//End of exInput!=0
    }//End of exInput!=oldInput
      } //End of ioIntVal High
      else{
             // Serial.println("Interupt High");
      }
        } 
//    lastEvent = currentTime;
//  }

//Send Queued RF Packets
  if (pktBufferPt>0){
    pktBufferPt--;

    Mirf.setTADDR((byte *)pktBuffer[pktBufferPt].pktTypes.INIT.newNode.address);
    //Mirf.setTADDR((byte *)"NODE1");
    Mirf.send((byte *)&pktBuffer[pktBufferPt]);
    while(Mirf.isSending());
    // Serial.print("Send Response to :");
    //Serial.println(pktBuffer[pktBufferPt].pktTypes.INIT.newNode.address);
  }
  
  //Respond To RF Packets
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
          Serial.println(packet.pktTypes.INIT.newNode.nodeNumber);
          Serial.println("Already registered");
          pktBuffer[pktBufferPt++] = packet;
          //   Mirf.setTADDR((byte *)clients[i].address);
          //   Mirf.send((byte *)&packet);
          //while(Mirf.isSending()); 
          //Serial.println("Responded to node");
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
//     Serial.println("Responded to Node");       
      break;     

    case RFTYPE_UPDATE:
       //  Serial.print("Node ");

      if(packet.pktTypes.UPDATE.nodeNumber>0){
        nodeTemp[packet.pktTypes.UPDATE.nodeNumber-1] = packet.pktTypes.UPDATE.temperature;
        nodeSoil[packet.pktTypes.UPDATE.nodeNumber-1] = packet.pktTypes.UPDATE.soilSensor;
        nodeLight[packet.pktTypes.UPDATE.nodeNumber-1] = packet.pktTypes.UPDATE.light;
        nodeBat[packet.pktTypes.UPDATE.nodeNumber-1] = packet.pktTypes.UPDATE.battery;
        Serial.print("Node ");
        Serial.print(packet.pktTypes.UPDATE.nodeNumber);
        Serial.println(" Data Recieved");
        Serial.print("Temp: ");
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

//USB Tasks
  Usb.Task();
  //Process USB Packets
  if (adk.isReady()) {
    uint16_t len = sizeof (recMsg);
    uint8_t rcode = adk.RcvData(&len, recMsg);
    if (rcode && rcode != hrNAK) {
//      Serial.print(F("\r\nData rcv: "));
//      Serial.print(rcode, HEX);
    } 
    else if (len > 0) {
      switch (recMsg[0]) {
      case PKT_CMD:
        switch (recMsg[1]) {
        case CMD_SOLENOID:
          setSolenoid(recMsg[2], recMsg[3]);
          break;
        case CMD_SET_SECURITY:
          secEnable = (recMsg[2]);
          secSound = (recMsg[3]);
          secLed = (recMsg[4]);
          break;

        case CMD_MOVE_HEAD:
          setHeadDirection(recMsg[2],false);
          break;
        case CMD_SET_RED:
          setRGBLED(recMsg[1], recMsg[2]);
          break;
        case CMD_SET_GREEN:
          setRGBLED(recMsg[1], recMsg[2]);
          break;
        case CMD_SET_BLUE:
          setRGBLED(recMsg[1], recMsg[2]);
          break;
        case CMD_SET_ALL:
          setRGBLED(CMD_SET_RED, recMsg[2]);
          setRGBLED(CMD_SET_GREEN, recMsg[3]);
          setRGBLED(CMD_SET_BLUE, recMsg[4]);
          break;
        }
        break;
      }
    }

    if (millis() - timer >= 2000) { // Send data every 10s
      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
//            humidityVal = dht.readHumidity();
//            temperatureVal = dht.readTemperature();
//            humidLong = (long) (humidityVal * 1000);
//            tempLong = (long) (temperatureVal * 1000);
      lightByte = analogRead(PIN_LIGHT)*.097656;
      // check if returns are valid, if they are NaN (not a number) then something went wrong!
      //      if (isnan(temperatureVal) || isnan(humidityVal)) {
      //        Serial.println("Failed to read from DHT");
      //      }
      outPacket.type = PKT_UPDATE;
      outPacket.pktTypes.UPDATE.nodes = numClients;
      outPacket.pktTypes.UPDATE.baseTemp = 71000;
      outPacket.pktTypes.UPDATE.baseHumid = 48000;
      outPacket.pktTypes.UPDATE.baseLight = lightByte;
      outPacket.pktTypes.UPDATE.baseDirection = headDirection;
      outPacket.pktTypes.UPDATE.soilSensor[0] = nodeSoil[0];
      outPacket.pktTypes.UPDATE.soilSensor[1] = nodeSoil[1];
      outPacket.pktTypes.UPDATE.light[0] = nodeLight[0];
      outPacket.pktTypes.UPDATE.light[1] = nodeLight[1];
      outPacket.pktTypes.UPDATE.battery[0] = nodeBat[0];
      outPacket.pktTypes.UPDATE.battery[1] = nodeBat[1];
      outPacket.pktTypes.UPDATE.temperature[0] = nodeTemp[0];
      outPacket.pktTypes.UPDATE.temperature[1] = nodeTemp[1];
      outPacket.pktTypes.UPDATE.rValue = currentRed;
      outPacket.pktTypes.UPDATE.gValue = currentGreen;
      outPacket.pktTypes.UPDATE.bValue = currentBlue;

      timer = millis();
      rcode = adk.SndData(sizeof (outPacket), (uint8_t*) & outPacket);
      //Serial.print("Humidity: ");
      //Serial.print(humidityVal);
      //Serial.print(" %\t");
      //Serial.print("Temperature: ");
      //Serial.print(temperatureVal);
      //Serial.println(" *C");
    }
  }
}

void setSolenoid(byte solenoid, byte state) {
 // Serial.print("\r\nSolenoid: ");
  // Serial.print(solenoid);
  if (state == 1) {
   // if (!solenoidState[solenoid]) {
      //Serial.print("Activated");
      solenoidState[solenoid] = true;
      setExOutput(solenoid,true);
    }
//  } 
  else if (state == 0) {
//    if (solenoidState[solenoid]) {
      solenoidState[solenoid] = false;
      setExOutput(solenoid,false);
            //Serial.print("deactivated");

   // }
  } 
  else {
    //Serial.print("Error: ");
    //Serial.print(state);
  }
}

void setExOutput(int output, boolean value){
  if (value){
    exOutput = exOutput|(1<<output);
  }
  else{
    exOutput =   exOutput&(~(1<<output));
  }
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(IO_GPIOB); // GPIOB
  Wire.write(exOutput); // port B
  Wire.endTransmission();
}
void setRGBLED(unsigned int color, int value){
  switch(color){
  case CMD_SET_RED:
    //Serial.print("RED SET: 0x");
    currentRed = value&0xFF;
    break;
  case CMD_SET_GREEN:
    //Serial.print("GREEN SET: 0x");
    currentGreen = value&0xFF;
    break;
  case CMD_SET_BLUE:
    //Serial.print("BLUE SET: 0x");
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
    // Serial.println(value);
  }
  else{
    //   Serial.print("No Nodes");
  }  
}
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
    case 1: 
      mode = 0x01; 
      break;
    case 8: 
      mode = 0x02; 
      break;
    case 64: 
      mode = 0x03; 
      break;
    case 256: 
      mode = 0x04; 
      break;
    case 1024: 
      mode = 0x05; 
      break;
    default: 
      return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } 
    else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } 
  else if(pin == 3 || pin == 11) {
    switch(divisor) {
    case 1: 
      mode = 0x01; 
      break;
    case 8: 
      mode = 0x02; 
      break;
    case 32: 
      mode = 0x03; 
      break;
    case 64: 
      mode = 0x04; 
      break;
    case 128: 
      mode = 0x05; 
      break;
    case 256: 
      mode = 0x06; 
      break;
    case 1024: 
      mode = 0x7; 
      break;
    default: 
      return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

void setSound(byte arg){
  if (arg == 1){
    analogWrite(PIN_SOUND, 128);
  }
  else{
    analogWrite(PIN_SOUND, 0);
  }
}





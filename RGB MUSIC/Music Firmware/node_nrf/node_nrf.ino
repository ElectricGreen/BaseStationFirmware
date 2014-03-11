/* FILE:    Microduino_nRF24L01_Transceiver_Module_HCMIDU0006_Sensor_Example
*  DATE:    03/06/13
*  VERSION: 0.1
*
* This is an example of how to use the Hobby Components Microduino nRF24L01 2.4GHz
* wireless transceiver (HCMIDU0006). This transceiver module is a very cost effective
* way to transmit data from one microcontroller to another over short distances. This
* simple example will configure the module using the freely available Mirf library
* to transmit the analogue value from analogue input A0 to a remote module.
*
*
* You may copy, alter and reuse this code in any way you like, but please leave
* reference to HobbyComponents.com in your comments if you redistribute this code.
* This software may not be used directly for the purpose of selling products that
* directly compete with Hobby Components Ltd's own range of products.
*
* THIS SOFTWARE IS PROVIDED "AS IS". HOBBY COMPONENTS MAKES NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ACCURACY OR LACK OF NEGLIGENCE.
* HOBBY COMPONENTS SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR ANY DAMAGES,
* INCLUDING, BUT NOT LIMITED TO, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY
* REASON WHATSOEVER.
*
*/

/* Include the SPI anf Mirf libraries */
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

/* Stores the current analogue input value */
byte incomingByte = 0;   // for incoming serial data
byte currentColor = 0;
unsigned int packet = 0;

#define SOIL_PIN   5

#define RGB_RED    3
#define RGB_GREEN  5
#define RGB_BLUE   6

void setColor(unsigned int color);
void setup(){
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  /* Define the DIO used for SPI CSN & CE. Defaults are DIO
  *  10 & 9 respectively. */
  Mirf.cePin = 9;
  Mirf.csnPin = 10;
 
  Mirf.spi = &MirfHardwareSpi;
 
  /* Initialise the module */
  Mirf.init();
 
  /* Set the receiving address of this module. This must be a 5 byte
  *  character string */
  Mirf.setRADDR((byte *)"SENS1");
 
  /* Set the payload length to size unsigned int (size of data from
  *  the analogue input A0) */
  Mirf.payload = sizeof(packet);
 
  /* Set the channel. Channel 90 = 2.490GHz */
  Mirf.channel = 90;
 
  /* Configure the module */
  Mirf.config();
  
  pinMode(RGB_RED, OUTPUT);      // sets the digital pin as output
  pinMode(RGB_GREEN, OUTPUT);      // sets the digital pin as output
  pinMode(RGB_BLUE, OUTPUT);      // sets the digital pin as output\\\\\\\\\\

  Serial.println("Module initialised");
}


void setColor(unsigned int color, byte value){
                  switch(color){
                    case RGB_RED:
                                      Serial.print("RED SET: 0x");
                                      
                          //analogWrite(RGB_RED,incomingByte);
                    break;
                    case RGB_GREEN:
                                      Serial.print("GREEN SET: 0x");
                          //analogWrite(RGB_GREEN,incomingByte);
                    break;
                    case RGB_BLUE:
                                      Serial.print("BLUE SET: 0");
                          //analogWrite(RGB_BLUE,incomingByte);
                    break;
                    
                  }
                   Serial.println(value, HEX);
                     /* ...and send it to the client */
  packet = ((color&0xFF)<<8)+(value&0xFF);
  Mirf.send((byte *)&packet);
 
  /* Wait for module to finish sending */
  while(Mirf.isSending());
                  
}


/******* Main Program ********/
void loop()
{
  
 //  float h = dht.readHumidity();
  /* Set transmit address to client module 1 */
  Mirf.setTADDR((byte *)"CLI01");
 
  /* Read the analogue input A0... */
  //val = analogRead(SOIL_PIN);    // read the input pin
  //Serial.print("Sending: ");
  //Serial.println(val);
 if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();
                // say what you got:
                switch (incomingByte){
                  case 0xFF:
                  //case 'r':
                           Serial.print("RED\n");
                           currentColor = RGB_RED;
                  break;
                  case 0xFE:
                  //case 'g':
                           Serial.print("Green\n");
                           currentColor = RGB_GREEN;
                  break;
                  case 0xFD:
                  //case 'b':
                           Serial.print("Blue\n");
                           currentColor = RGB_BLUE;
                  break;
                 //case 'f':
                           Serial.print("Fade\n");
                           for (int i = 0;i<255;i++){
                             setColor(RGB_RED,i);
                           }
                           for (int i=0; i<255; i++){
                             setColor(RGB_GREEN,i);
                           }
                           for (int i=0; i<255; i++){
                             setColor(RGB_BLUE,i);
                           }
                           break;
                           default:
                           setColor(currentColor, incomingByte);
                           break;
                }
}
}

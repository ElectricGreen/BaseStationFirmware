/* FILE:    Microduino_nRF24L01_Transceiver_Module_HCMIDU0006_CLient_Example
*  DATE:    03/06/13
*  VERSION: 0.1
*
* This is an example of how to use the Hobby Components Microduino nRF24L01 2.4GHz
* wireless transceiver (HCMIDU0006). This transceiver module is a very cost effective
* way to transmit data from one microcontroller to another over short distances. This
* simple example will configure the module using the freely available Mirf library
* to receive the analogue value from analogue input A0 on a remote module.
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

unsigned int packet = 0;

#define SOIL_PIN   5

#define RGB_RED    3
#define RGB_GREEN  5
#define RGB_BLUE   6

void setup(){

  /* Define the DIO used to SPI CSN & CE. Defaults are DIO
  *  10 & 9 respectively. */
  Mirf.cePin = 9;
  Mirf.csnPin = 10;
 
  Mirf.spi = &MirfHardwareSpi;
 
  /* Initialise the module */
  Mirf.init();
 
  /* Set the receiving address of this module. This must be a 5 byte
  *  character string */
 
  Mirf.setRADDR((byte *)"CLI01");
 
  /* Set the payload length to size unsigned int (size of data from
  *  the analogue input A0) */
  Mirf.payload = sizeof(packet);
 
  /* Set the channel. Channel 90 = 2.490GHz */
  Mirf.channel = 90;
 
  /* Configure the module */
  Mirf.config();
 
    pinMode(RGB_RED, OUTPUT);      // sets the digital pin as output
      pinMode(RGB_GREEN, OUTPUT);      // sets the digital pin as output
    pinMode(RGB_BLUE, OUTPUT);      // sets the digital pin as output
   Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
                                                        Serial.print("Test start");

}


/******* Main Program ********/
void loop()
{
 
  /* If we are not currently sending an there is new data available
   * then get the new data and output it to the UART */   
  if(!Mirf.isSending() && Mirf.dataReady()){

    /* Get the data from the receive buffer */
    Mirf.getData((byte *) &packet);
                                                        Serial.print("Got");
                 analogWrite(((packet&0xFF00)>>8),packet&0xFF);

                }
  }

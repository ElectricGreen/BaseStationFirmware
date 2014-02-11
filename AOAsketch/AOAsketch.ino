#include <adk.h>
#include "DHT.h"

#define DHTPIN 4     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define PKT_UPDATE 0x05 
#define PKT_CMD 0x06 

#define CMD_SOLENOID 0x10
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
    } else if (state == 0) {
        if (solenoidState) {
            solenoidState = false;
            Serial.print("Deactivated");
            digitalWrite(PIN_SOLENOID, true);
            delay(20);
            digitalWrite(PIN_SOLENOID, false);
        }
    } else {
        Serial.print("Error: ");
        Serial.print(state);
    }
}

void loop() {
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    humidityVal = dht.readHumidity();
    temperatureVal = dht.readTemperature();
    humidLong = (long) (humidityVal * 1000);
    tempLong = (long) (temperatureVal * 1000);

    // check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(temperatureVal) || isnan(humidityVal)) {
        Serial.println("Failed to read from DHT");
    }

    Usb.Task();
    if (adk.isReady()) {
        uint16_t len = sizeof (recMsg);
        uint8_t rcode = adk.RcvData(&len, recMsg);
        if (rcode && rcode != hrNAK) {
            Serial.print(F("\r\nData rcv: "));
            Serial.print(rcode, HEX);
        } else if (len > 0) {
            switch (recMsg[0]) {
                case PKT_CMD:
                    switch (recMsg[1]) {
                        case CMD_SOLENOID:
                            setSolenoid(recMsg[2], recMsg[3]);
                            break;
                    }
                    break;
            }
        }


        if (millis() - timer >= 2000) { // Send data every 1s
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

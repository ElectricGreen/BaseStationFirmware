 //while(true){ //Test Loop
    //read light sensor GOOOD!
    //int val = analogRead(PIN_LIGHT);
    //Serial.println(val);
    //delay(500);

    //Check Sound Output GOOD!
    //setSound(1);
    //while(1);
 
    //check DHT GOOD!
//      humidityVal = dht.readHumidity();
//      temperatureVal = dht.readTemperature();
//      humidLong = (long) (humidityVal * 1000);
//      tempLong = (long) (temperatureVal * 1000);
//      Serial.print("Humidity: ");
//      Serial.print(humidityVal);
//      Serial.print(" %\t");
//      Serial.print("Temperature: ");
//      Serial.print(temperatureVal);
//      Serial.println(" *C");
   
     //Test IO EX Outputs
//     delay(1000);
//     setExOutput(1, true);
//     delay(1000);
//      setExOutput(2, true);
//      delay(1000);
//     setExOutput(0, true);
//     delay(5000);
//     setExOutput(0, false);
//     delay(5000);

    //Reading inputs
    
//    long currentTime = millis();
//    if (currentTime>lastEvent+200){ //Check Every 200ms
//      int oldInput = exInput;
// Wire.beginTransmission(I2C_ADDRESS);
// Wire.write(IO_GPIOA); // set MCP23017 memory pointer to GPIOA address
// Wire.endTransmission();
// Wire.requestFrom(I2C_ADDRESS, 1); // request one byte of data from MCP20317
// exInput=Wire.read(); // store the incoming byte into "inputs"
// 
// if ((exInput!=0)&&(changeTime==0)){ //First time motion is detected
//   changeTime = currentTime;
// }else if ((exInput!=0)&(currentTime>changeTime+10000)){
//   outPacket.type = PKT_ALERT;
// outPacket.pktTypes.ALERT.motionSense = exInput;
//       //adk.SndData(sizeof (outPacket), (uint8_t*) & outPacket);
//        Serial.println(exInput);
//       if (currentTime>motorTime+60000){
//         
//           
//         motorTime = currentTime;
//       }
//       changeTime =0;
// }
//      lastEvent = currentTime;
//  }
//
//  digitalWrite(PIN_MOTOR_EN, MOTOR_ON);
//  stepper.step(100);
//  delay(2000);
//  digitalWrite(PIN_MOTOR_EN, MOTOR_OFF);
//delay(5000);
//digitalWrite(PIN_MOTOR_EN, MOTOR_ON);
//  //stepper.step(-50);
//  delay(20000);
//  digitalWrite(PIN_MOTOR_EN, MOTOR_OFF);
//  delay(200000);
//  }

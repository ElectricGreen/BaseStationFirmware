
struct NODE{
  byte nodeNumber;
  char address[5];
};

struct USB_PACKET{
  byte type;
  union {
    struct UPDATE{
      uint8_t nodes;
      uint8_t soilSensor[2];
      uint8_t light[2];
      uint8_t battery[2];
      uint16_t temperature[2];
      uint32_t baseTemp;
      uint32_t baseHumid;
      uint8_t baseLight;
      uint8_t baseDirection;
      uint8_t rValue;
      uint8_t gValue;
      uint8_t bValue;
    }
    UPDATE;
    struct ALERT{
      uint8_t type;
      uint8_t motionSense;
    }
    ALERT;
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

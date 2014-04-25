
//Change notice interrupts on PORT C (Analogs) Only
#define NO_PORTB_PINCHANGES // to indicate that port b will not be used for pin change interrupts
#define NO_PORTD_PINCHANGES // to indicate that port d will not be used for pin change interrupts
#define DISABLE_PCINT_MULTI_SERVICE

#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define PKT_UPDATE  0x05 
#define PKT_CMD     0x06 
#define PKT_ALERT   0x07

#define RFTYPE_REGISTER 1
#define RFTYPE_ACCEPTED 2
#define RFTYPE_UPDATE 3
#define RFTYPE_CMD 4
#define RFTYPE_LED 5

#define CMD_SOLENOID   0x10
#define CMD_MOVE_HEAD  0x20
#define CMD_SET_SECURITY      0x30

#define CMD_SET_RED    0x50
#define CMD_SET_GREEN  0x51
#define CMD_SET_BLUE   0x52
#define CMD_SET_ALL    0x53

#define MAX_CLIENTS 2

#define SOLENOID_OFF 0
#define SOLENOID_ON 1

#define MOTOR_OFF 0
#define MOTOR_ON 1

#define MOTOR_STEPS 200

#define I2C_ADDRESS 0x20
#define IO_IODIRA   0x00
#define IO_IODIRB   0x01

#define IO_GPINTENA 0x04
#define IO_GPINTENB 0x05

#define IO_GPIOA    0x12
#define IO_GPIOB    0x13


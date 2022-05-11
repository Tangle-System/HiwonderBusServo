#include <HiwonderBusServo.h>

BusServoNetwork network(UART_NUM_1); //UART_NUM_1 or UART_NUM_2
BusServoNetwork::Servo servo_broadcast = network.getServoBroadcast();

// for communication to servos only we can ignore TX_EN_PIN, RX_PIN and RX_EN_PIN and only use TX_PIN.
#define TX_PIN 23
#define TX_EN_PIN -1
#define RX_PIN -1
#define RX_EN_PIN -1

void setup() {
  // start the servo communications
  network.begin(TX_PIN, TX_EN_PIN, RX_PIN, RX_EN_PIN);
}

void loop() {
	
  // set positions (servo mode)
  servo_broadcast.setPosition(0, 1000);
  delay(1000);
  servo_broadcast.setPosition(1000, 1000);
  delay(1000);

  // set speed (motor mode)
  servo_broadcast.setSpeed(-500);
  delay(10000);
  servo_broadcast.setSpeed(200);
  delay(10000);
}
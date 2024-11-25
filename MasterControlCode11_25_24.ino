#include <Usb.h>
#include <usbhub.h>
#include <hiduniversal.h>

// Pin Definitions
#define HC05_EN 30       // Control EN/KEY pin through a voltage divider
#define HC05_POWER 31    // Control power to HC-05 via MOSFET
#define HC05_TXD 15      // HC-05 TXD connected to Arduino RX3
#define HC05_RXD 14      // HC-05 RXD connected to Arduino TX3 via voltage divider

#define AT_MODE 0        // Set to 1 for AT mode, 0 for pairing mode
#define SLAVE_ADDR "98D3,02,965455"

USB     Usb;
USBHub  Hub(&Usb);
HIDUniversal Hid(&Usb);

// class for handling joystick events and event handling
class JoystickEvents {
public:
  void OnGamePadChanged(const uint8_t* data, uint8_t len);
};

// parses new inputs
class JoystickReportParser : public HIDReportParser {
  JoystickEvents *joyEvents;
  uint8_t oldPad[8];

public:
  JoystickReportParser(JoystickEvents *evt);

  void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);
};

JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

// AT command handling
void send_AT_Command(const char *AT_COMMAND){
  Serial.print("Sending AT Command: ");
  Serial.println(AT_COMMAND);
  Serial3.print(AT_COMMAND);
  Serial3.print("\r\n");  // ensure proper termination
  delay(500);

  while(Serial3.available()){
    char c = Serial3.read();
    Serial.write(c);
  }
  Serial.println();
}

void setup() {
  // init Serial Ports
  Serial.begin(9600);      // Serial Monitor
  while (!Serial); // Wait for serial port to be ready
  Serial3.begin(38400);    // HC-05 default AT mode baud rate

  // set Pin Modes
  pinMode(HC05_EN, OUTPUT);
  pinMode(HC05_POWER, OUTPUT);

  // init Pins
  digitalWrite(HC05_POWER, LOW);    // ensure HC-05 is OFF initially (P-MOSFET OFF)

  Serial.println("HC-05 Power Control Test Starting...");

  if(AT_MODE){
    digitalWrite(HC05_EN, HIGH);
    Serial.println("Starting in AT_MODE");
  } else {
    digitalWrite(HC05_EN, LOW);
    Serial.println("Starting in pairing mode");
  }

  delay(100);                       // Small delay to set EN/KEY
  digitalWrite(HC05_POWER, HIGH);   // Turn ON HC-05
  delay(1000);                      // Wait for HC-05 to initialize

  if(AT_MODE){
    // config HC-05 in AT mode
    send_AT_Command("AT");
    send_AT_Command("AT+RMAAD");
    send_AT_Command("AT+CMODE=0");
    send_AT_Command("AT+NAME=HC05_USER_MASTER");
    send_AT_Command("AT+UART=9600,0,0");
    send_AT_Command("AT+PSWD=0000");
    send_AT_Command("AT+ROLE=1");

    char buf[50];
    snprintf(buf, sizeof(buf), "AT+BIND=%s", SLAVE_ADDR);
    send_AT_Command(buf);

    // after changing the HC-05 baud rate, reinitialize Serial3
    Serial3.end();
    Serial3.begin(9600);
  } else {
    // for pairing mode, ensure Serial3 is at 9600 baud
    Serial3.end();
    Serial3.begin(9600);
  }

  // init USB Host Shield
  if (Usb.Init() == -1) {
    Serial.println(F("USB Host Shield did not start."));
    while (1); // Halt
  }

  delay(200);

  Hid.SetReportParser(0, &Joy);
  Serial.println(F("Joystick ready!"));

  Serial.println("Setup complete. You can now send commands via Serial Monitor.");
}

void loop() {
  Usb.Task(); // Process USB tasks

  // USB Testing Functionality
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // Read the command from Serial Monitor
    command.trim();                                 // Remove any leading/trailing whitespace

    if (command.length() > 0) {
      Serial.print("Sending Command to HC-05: ");
      Serial.println(command);

      // send command to HC-05
      Serial3.print(command);
      Serial3.print("\r\n");  // ensure proper termination

      delay(500);  // Wait for response

      // read response from HC-05
      while (Serial3.available()) {
        char c = Serial3.read();
        Serial.write(c);
      }
      Serial.println();
    }
  }

  // Read any unsolicited data from HC-05
  while (Serial3.available()) {
    char c = Serial3.read();
    Serial.write(c);
  }
}

JoystickReportParser::JoystickReportParser(JoystickEvents *evt) :
  joyEvents(evt) {
  memset(oldPad, 0, sizeof(oldPad));
}

void JoystickReportParser::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
  // check if report is the same as previous
  if (memcmp(buf, oldPad, len) != 0) {
    // Report changed
    if (joyEvents)
      joyEvents->OnGamePadChanged(buf, len);

    memcpy(oldPad, buf, len);
  }
}

void JoystickEvents::OnGamePadChanged(const uint8_t* data, uint8_t len) {
  // debug
  Serial.print(F("Data: "));
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(data[i], HEX);
    Serial.print(F(" "));
  }
  Serial.println();

  // interpret joystick data and send commands via Bluetooth
  uint8_t xValue = data[0]; //  X-axis is at data[0]
  uint8_t yValue = data[1]; //  Y-axis is at data[1]

  int centerValue = 128;
  int deadZone = 20;

  if (xValue < centerValue - deadZone) {
    Serial.println(F("LEFT"));
    Serial3.println(F("LEFT")); // send command via Bluetooth using Serial3

  } else if (xValue > centerValue + deadZone) {
    Serial.println(F("RIGHT"));
    Serial3.println(F("RIGHT")); // send command via Bluetooth using Serial3

  } else if (yValue < centerValue - deadZone){
    Serial.println(F("Up"));
    Serial3.println(F("Up"));

  } else if (yValue > centerValue + deadZone){
    Serial.println(F("Down"));
    Serial3.println(F("Down"));
    Serial.println(yValue);

  } else {
    Serial.println(F("NEUTRAL"));
    Serial3.println(F("NEUTRAL")); // send command via Bluetooth using Serial3

  }
}
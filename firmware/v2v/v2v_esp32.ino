#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi Config
const char* ssid = "ros";
const char* password = "00000000";
WiFiUDP udp;
const int udpPort = 5555;
const int packetBufferSize = 255;
char packetBuffer[packetBufferSize];

// Motor Pins
int m1_1 = 25;  // PWM speed control motor 1
int m1_2 = 26;  // Direction motor 1
int m2_1 = 32;  // Direction motor 2
int m2_2 = 33;  // PWM speed control motor 2

int buzzer = 18; // Buzzer pin

// PWM Config
const int freq = 5000;       // 5 kHz PWM frequency
const int pwmChannelM1 = 0;  // PWM channel for motor 1 speed
const int pwmChannelM2 = 1;  // PWM channel for motor 2 speed
const int resolution = 8;    // 8-bit resolution (0-255)
const int lowSpeedDuty = 10; // Very low speed duty cycle (~4%)

// LCD Config
LiquidCrystal_I2C lcd(0x27, 16, 2);

bool once = true;

void setupReceiver() {
  Serial.begin(115200);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  if (udp.begin(udpPort)) {
    Serial.println("UDP Started on port 5555");
  } else {
    Serial.println("UDP Failed to start!");
  }
  WiFi.setSleep(false);
}

String receiveBroadcast() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet! Size: ");
    Serial.println(packetSize);
    Serial.print("From IP: ");
    Serial.print(udp.remoteIP());
    Serial.print(", Port: ");
    Serial.println(udp.remotePort());
    int len = udp.read(packetBuffer, packetBufferSize - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';
      Serial.print("Data: ");
      Serial.println(packetBuffer);
      return String(packetBuffer);
    }
  }
  return "";
}

void Forward_Motors() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Motors low speed");

  //  forward movement
  digitalWrite(m1_2, LOW);
  digitalWrite(m2_1, LOW);

  //  low speed
  ledcWrite(pwmChannelM1, lowSpeedDuty);
  ledcWrite(pwmChannelM2, lowSpeedDuty);
}

void Emergency_Alert() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EMERGENCY ALERT!");
  lcd.setCursor(0, 1);
  lcd.print("Faint detected");
  
  // Sound buzzer for 1.5 seconds
  digitalWrite(buzzer, HIGH);
  delay(1500);
  digitalWrite(buzzer, LOW);
}

void starting_message() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Welcome, ADMAS!");
  delay(2000);
}

void setup() {
  pinMode(m1_2, OUTPUT);
  pinMode(m2_1, OUTPUT);

  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW); 


  ledcSetup(pwmChannelM1, freq, resolution);
  ledcAttachPin(m1_1, pwmChannelM1);

  ledcSetup(pwmChannelM2, freq, resolution);
  ledcAttachPin(m2_2, pwmChannelM2);

  lcd.init();
  lcd.backlight();
  setupReceiver();
}

void loop() {
  String receivedValue = receiveBroadcast();

  if (once) {
    starting_message();
    Forward_Motors();
    once = false;
  }

  if (receivedValue != "") {
    Emergency_Alert();
    receivedValue = "";
  }
}

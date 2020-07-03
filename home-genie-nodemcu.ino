/*
  Basic ESP8266 MQTT example
  This sketch demonstrates the capabilities of the pubsub library in combination
  with the ESP8266 board/library.
  It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP
  Led, else switch it off It will reconnect to the server if the connection is
  lost using a blocking reconnect function. See the 'mqtt_reconnect_nonblocking'
  example for how to achieve the same result without blocking the main loop. To
  install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences ->
  Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the
  ESP8266"
  - Select your ESP8266 in "Tools -> Board"
*/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <stdlib.h>

const char *ssid = "Void";
const char *password = "password";
const char *mqtt_server = "mqtt.eclipse.org";

const char *topic1 = "topic/ftflteam3/commands";
const char *topic2 = "topic/ftflteam3/activities";
const char *topic3 = "topic/ftflteam3/heartbeats";
const char *topic4 = "topic/ftflteam3/temperature";

// pins
int d0 = 16;  // blue led sw1
int d1 = 5;   // green led sw2
int d2 = 4;   // red led sw3
int d3 = 0;   // led
int d5 = 14;  // motor
int analogPin = A0;
int motionPin = 10;

// states
int heartbeat = 0;
bool lightAuto = true;
bool fanAuto = true;
double temperature = 0;
bool hasMotion = false;
int hasMotionCycles = 0;
int noMotionCycles = 0;

// timekeepers
unsigned long lastMsg = 0;          // millis
unsigned long lastMotionCheck = 0;  // millis
unsigned long lastTempCheck = 0;    // millis
unsigned long lastStatusSent = 0;   // millis

// intervals
const int heartbeatInterval = 5000;
const int motionCheckInterval = 1000;  // millis
const int tempCheckInterval = 1000;    // millis
const int sendStatusInterval = 1000;   // millis

// constants
const int fanStartTemp = 40;  // degree celsius
const int tempThreshold = 2;  // degree celsius

// constants needed for thermistor calculation
const double VCC = 3.3;              // NodeMCU on board 3.3v vcc
const double R2 = 10000;             // 10k ohm series resistor
const double adc_resolution = 1023;  // 10-bit adc
const double A = 0.001129148;        // thermistor equation parameters
const double B = 0.000234125;
const double C = 0.0000000876741;

// objects
WiFiClient espClient;
PubSubClient client(espClient);
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.println("");
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived at [");
  Serial.print(topic);
  Serial.print("]  ==> ");

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strcmp(topic, topic1) == 0) {
    doAction(payload, length);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(topic2, "hello!");
      // ... and resubscribe
      client.subscribe(topic1);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(d0, OUTPUT);  // sw1 blue led
  pinMode(d1, OUTPUT);  // sw2 green
  pinMode(d2, OUTPUT);  // sw3 red
  pinMode(d3, OUTPUT);  // led
  pinMode(d5, OUTPUT);  // motor
  pinMode(motionPin, INPUT);
  pinMode(analogPin, INPUT);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  sendHeartbeat();

  motionDetect();

  thermistorTemp();

  // flow
  if (fanAuto) {
    if (temperature > fanStartTemp) {
      digitalWrite(d5, HIGH);
    } else {
      if (temperature < (fanStartTemp - tempThreshold)) {
        digitalWrite(d5, LOW);
      }
    }
  }

  if (lightAuto) {
    if (hasMotion) {
      digitalWrite(d3, HIGH);
    } else {
      digitalWrite(d3, LOW);
    }
  }

  sendStatus();

}  // loop()

void sendHeartbeat() {
  unsigned long now = millis();
  if (now - lastMsg > heartbeatInterval) {
    lastMsg = now;
    ++heartbeat;
    snprintf(msg, MSG_BUFFER_SIZE, "Heartbeat %ld", heartbeat);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish(topic3, msg);
  }
}

void sendStatus() {
  unsigned long now = millis();
  if (now - lastStatusSent > sendStatusInterval) {
    lastStatusSent = now;
    if (lightAuto) {
      client.publish(topic2, "switch6-on");
    } else {
      client.publish(topic2, "switch6-off");
    }
    delay(10);
    if (fanAuto) {
      client.publish(topic2, "switch7-on");
    } else {
      client.publish(topic2, "switch7-off");
    }
    delay(10);
    if (digitalRead(d0) == HIGH) {
      client.publish(topic2, "switch1-on");
    } else {
      client.publish(topic2, "switch1-off");
    }
    delay(10);
    if (digitalRead(d1) == HIGH) {
      client.publish(topic2, "switch2-on");
    } else {
      client.publish(topic2, "switch2-off");
    }
    delay(10);
    if (digitalRead(d2) == HIGH) {
      client.publish(topic2, "switch3-on");
    } else {
      client.publish(topic2, "switch3-off");
    }
    delay(10);
    if (digitalRead(d3) == HIGH) {
      client.publish(topic2, "switch4-on");
    } else {
      client.publish(topic2, "switch4-off");
    }
    delay(10);
    if (digitalRead(d5) == HIGH) {
      client.publish(topic2, "switch5-on");
    } else {
      client.publish(topic2, "switch5-off");
    }
    delay(10);
  }
}  // sendStatus()

void doAction(byte *payload, unsigned int length) {
  bool needToPublish = true;
  delay(20);
  if (!strncmp((char *)payload, "switch1-on", length)) {
    digitalWrite(d0, HIGH);
  } else if (!strncmp((char *)payload, "switch1-off", length)) {
    digitalWrite(d0, LOW);
  } else if (!strncmp((char *)payload, "switch2-on", length)) {
    digitalWrite(d1, HIGH);
  } else if (!strncmp((char *)payload, "switch2-off", length)) {
    digitalWrite(d1, LOW);
  } else if (!strncmp((char *)payload, "switch3-on", length)) {
    digitalWrite(d2, HIGH);
  } else if (!strncmp((char *)payload, "switch3-off", length)) {
    digitalWrite(d2, LOW);
  } else if (!strncmp((char *)payload, "switch4-on", length)) {
    if (!lightAuto) digitalWrite(d3, HIGH);
  } else if (!strncmp((char *)payload, "switch4-off", length)) {
    if (!lightAuto) digitalWrite(d3, LOW);
  } else if (!strncmp((char *)payload, "switch5-on", length)) {
    if (!fanAuto) digitalWrite(d5, HIGH);
  } else if (!strncmp((char *)payload, "switch5-off", length)) {
    if (!fanAuto) digitalWrite(d5, LOW);
  } else if (!strncmp((char *)payload, "switch6-on", length)) {
    lightAuto = true;
  } else if (!strncmp((char *)payload, "switch6-off", length)) {
    lightAuto = false;
  } else if (!strncmp((char *)payload, "switch7-on", length)) {
    fanAuto = true;
  } else if (!strncmp((char *)payload, "switch7-off", length)) {
    fanAuto = false;
  } else if (!strncmp((char *)payload, "led-on", length)) {
    digitalWrite(d3, HIGH);
  } else if (!strncmp((char *)payload, "led-off", length)) {
    digitalWrite(d3, LOW);
  } else if (!strncmp((char *)payload, "motor-on", length)) {
    digitalWrite(d5, HIGH);
  } else if (!strncmp((char *)payload, "motor-off", length)) {
    digitalWrite(d5, LOW);
  } else {
    needToPublish = false;
  }

  if (needToPublish) {
    char outMsg[length];
    for (int i = 0; i < length; i++) {
      outMsg[i] = (char)payload[i];
    }
    delay(10);
    client.publish(topic2, outMsg);
    delay(10);
    Serial.print("Published to topic2: ");
    Serial.println(outMsg);
  }
}  // doAction()

void motionDetect() {
  if (!lightAuto) {
    return;
  }

  const int motionTriggerCycles = 2;
  const int noMotionTriggerCycles = 6;
  unsigned long now = millis();
  if (now - lastMotionCheck > motionCheckInterval) {
    lastMotionCheck = now;
    int motionVal = digitalRead(motionPin);
    if (motionVal == HIGH) {
      Serial.print("Motion detected! Cycle = ");
      noMotionCycles = 0;
      hasMotionCycles++;
      Serial.println(hasMotionCycles);
      if (hasMotionCycles > motionTriggerCycles) {
        hasMotion = true;
        hasMotionCycles = 0;
      }
    } else {
      Serial.print("No motion detected. Cycle = ");
      hasMotionCycles = 0;
      noMotionCycles++;
      Serial.println(noMotionCycles);
      if (noMotionCycles > noMotionTriggerCycles) {
        hasMotion = false;
        noMotionCycles = 0;
      }
    }
  }
}

void thermistorTemp() {
  if (!fanAuto) {
    return;
  }

  unsigned long now = millis();
  if (now - lastTempCheck > tempCheckInterval) {
    lastTempCheck = now;
    double Vout, Rth, adc_heartbeat;

    temperature = 0;
    adc_heartbeat = analogRead(analogPin);
    Vout = (adc_heartbeat * VCC) / adc_resolution;
    Rth = (VCC * R2 / Vout) - R2;

    /*  Steinhart-Hart Thermistor Equation:
      Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)
      where A = 0.001129148, B = 0.000234125 and C = 8.76741*10^-8  */
    temperature = (1 / (A + (B * log(Rth)) +
                        (C * pow((log(Rth)), 3))));  // Temperature in kelvin

    temperature = temperature - 273.15;  // Temperature in degree celsius
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" degree celsius.");

    char tempStr[50];
    snprintf(tempStr, 50, "%d", (int)temperature);
    client.publish(topic4, tempStr);
    // delay(500);
  }
}  // thermistorTemp()

#include <WiFi.h>
#include <PubSubClient.h>
#include <Keypad.h>
#include <ESP32Servo.h>
#include <ezButton.h>

//WiFi Network credentials
const char* ssid = "jntes";
const char* password = "mivi9960";

//MQTT Broker Settings
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char* mqtt_username = "idpgrp3";
const char* mqtt_password = "turtlebot";
const char* mqtt_topic_tablenum = "table_num";
const char* mqtt_topic_docking = "docking";

//Keypad Setup
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {33, 25, 26, 27};
byte colPins[COLS] = {14, 12, 13};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

//Limit Switch Setup
ezButton limitSwitch(15);

//Servo Motor Setup
Servo servo;
int servoPin = 4;

//Initialize WiFi & MQTT client
WiFiClient espClient;
PubSubClient client(mqtt_server, mqtt_port, espClient);

unsigned long previousMillis = 0;
unsigned long interval = 30000;
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (5)
char msg[MSG_BUFFER_SIZE];

int tableNum = 0;
bool robot_arrived = false;
bool docking = false;

void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);
  Serial.println();
  Serial.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  //Loop until we're reconnected
  //client.connect returns a boolean value to let us know if the connection was successful.
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    /*clientId += String(random(0xffff), HEX);*/
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT Broker");
    } 
    else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      //Wait 5 seconds before retrying
      delay(2000);
    }
  }
}

void constantPublish() {
  //Publish Docking Status every 2 seconds
  unsigned long currentMillis = millis();
  if (currentMillis - lastMsg > 2000) {
    lastMsg = currentMillis;
    //Read Docking Status
    limitSwitch.loop();
    int state = limitSwitch.getState();
    if (state == HIGH) {
      docking = true;
    }
    else {
      docking = false;
    }
    //String dockingS = "Docking Status: "+String(docking)+" ";
    //Serial.println(dockingS);

    snprintf(msg, MSG_BUFFER_SIZE, "%d", docking);
    //Serial.print("Docking publish message: ");
    //Serial.println(msg);
    client.publish(mqtt_topic_docking, msg);
  }
}

void setup() {
  Serial.begin(9600);
	servo.attach(servoPin, 771, 2740);
	/*[Calibration] using min/max pulse widths of 771us and 2740us.
	  Different servos require different min/max settings.
    For an accurate 0 to 180 sweep */
  limitSwitch.setDebounceTime(50);
  servo.write(120);                 //set servo to initial position
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(65535);
}

void loop() {
  //Reconnect to MQTT broker if disconnected
  while (!client.connected()) {
    reconnect();
  }
  //Reconnect to WiFi if disconnected
  unsigned long currentMillis = millis();
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    //if WiFi is down, try reconnecting
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }

  //Check for status of robot's arrival
  while (!robot_arrived) {
    constantPublish();
    limitSwitch.loop();
    int state = limitSwitch.getState();
    if (state == HIGH) {
      //MQTT can only transmit strings
      docking = true;
      String dockingS = "Docking Status: "+String(docking)+" ";
      Serial.println(dockingS);
      //Serial.println(client.state()); //0 : MQTT_CONNECTED - the client is connected
      reconnect();
      delay(1000);
      //Publish to the MQTT Broker
      // unsigned long now = millis();
      // if (now - lastMsg > 10000) {
      //   lastMsg = now;
      if (client.publish(mqtt_topic_docking, String(dockingS).c_str())) {
        boolean rc = client.publish(mqtt_topic_docking, String(dockingS).c_str());
        Serial.println("Publishing Status: "+String(rc)+" ");
        Serial.println("Robot is receiving signal...");
        delay(2000);
      }
      else {
        Serial.println("Docking status failed to send. Reconnecting to MQTT Broker and trying again");
        reconnect();
        delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
        client.publish(mqtt_topic_docking, String(dockingS).c_str());
      }
      Serial.println("Limit switch activated! Robot is here, waiting for keypad input...");
      robot_arrived = true;
    }
    else {
      Serial.println("Robot has not arrived, waiting for robot...");
      delay(1000);      
    }
  }

  while (robot_arrived) {
    constantPublish();
    //Wait for valid input from keypad
    char key = keypad.getKey();
    while (key == NO_KEY || !(key >= '1' && key <= '6')) {
      constantPublish();
      key = keypad.getKey();
    }

    // Store keypad input if it's a valid table number
    if (key >= '1' && key <= '6') {
      tableNum = key - '0';
      //MQTT can only transmit strings
      String tableNumS = "Table No. for delivery: "+String(tableNum)+" ";
      Serial.println(tableNumS);
      //Serial.println(client.state()); //0 : MQTT_CONNECTED - the client is connected
      reconnect();
      delay(1000);
      //Publish to the MQTT Broker
      // unsigned long now = millis();
      // if (now - lastMsg > 10000) {
      //   lastMsg = now;
      if (client.publish(mqtt_topic_tablenum, String(tableNumS).c_str())) {
        boolean rc = client.publish(mqtt_topic_tablenum, String(tableNumS).c_str());
        Serial.println("Publishing Status: "+String(rc)+" ");
        Serial.println("Robot is receiving table number...");
        delay(1000);
        
        //Tilt bucket to transfer load can to robot
        Serial.println("Tilting...");
        delay(3000);
		    servo.write(20);
		    delay(3000);
        Serial.println("Returning to original position...");
		    servo.write(120);
        delay(2000);

        delay(1000);

        Serial.println("Can has been transferred. Robot is ready to deliver.");
        limitSwitch.loop();
        int state = limitSwitch.getState();
        while (state == HIGH) {
          constantPublish();
          Serial.println("Waiting for Robot to deliver...");
          delay(2000);
          limitSwitch.loop();
          state = limitSwitch.getState();
        }
        docking = false;
        String dockingS = "Docking Status: "+String(docking)+" ";
        Serial.println("Docking status: "+String(docking)+" ");
        //Serial.println(client.state()); //0 : MQTT_CONNECTED - the client is connected
        reconnect();
        delay(1000);
        //Publish to the MQTT Broker
        if (client.publish(mqtt_topic_docking, String(dockingS).c_str())) {
          boolean rc = client.publish(mqtt_topic_docking, String(dockingS).c_str());
          Serial.println("Publishing Status: "+String(rc)+" ");
          Serial.println("Docking status is sent!");
        }
        else {
          Serial.println("Docking status failed to send. Reconnecting to MQTT Broker and trying again");
          reconnect();
          delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
          client.publish(mqtt_topic_docking, String(dockingS).c_str());
        }
        Serial.println("Limit switch deactivated! Robot is delivering, waiting for next order.");
        robot_arrived = false;
      }
      //Again, client.publish will return a boolean value depending on whether it succeded or not.
      //If the message failed to send, we will try again, as the connection may have broken.
      else {
        Serial.println("Table No. failed to send. Reconnecting to MQTT Broker and trying again");
        reconnect();
        delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
        client.publish(mqtt_topic_tablenum, String(tableNumS).c_str());
      }        
    }
  }
}

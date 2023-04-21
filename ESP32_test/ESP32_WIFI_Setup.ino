#include <WiFi.h>

const char* ssid = "jntes";
const char* password = "mivi9960";

void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  delay(1000);
}

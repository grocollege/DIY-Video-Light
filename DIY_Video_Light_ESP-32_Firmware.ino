// Include required libraries
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi Settings - Replace these variables with your own information
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* mqtt_server = "YOUR_MQTT_SERVER_IP_ADDRESS";

// Initialise espClient to enable wifi and 
// communication with the MQTT server
WiFiClient espClient;
PubSubClient client(espClient);

// DEFINE CONSTANTS
// Constants for pin assignments
// Pin for On-Off Switch, 26 corresponds to GPIO26
const int swPin = 26;
// Pin for Brightness PWM Output, 16 corresponds to GPIO16
const int pwmPin = 16;
// Pin for Thermocouple1 input, 34 corresponds to GPIO34  
const int tcPin1 = 34;
// Pin for Thermocouple2 input, 35 corresponds to GPIO35  
const int tcPin2 = 35;

// Constants for Pusle Width Modulation (PWM)
const int freq = 2500;
const int ledChannel = 0;
const int resolution = 8;

// Constant for number of temperature readings to be averaged
const int numReadings = 10;

// INITIALISE VARIABLES
// Variables used for the timimg of the temperature measurements
long lastMsg = 0;
char msg[50];
int value = 0;

// Variables for temperature measurements
// Variable for measured thermocouple output voltage value
float tcValue = 0;

// Variables for Thermocouple 1
// Calculated temperature value   
float temperature1 = 0;  
// Array of readings from the analog input 
float readings1[numReadings];    
// Index of the current reading   
int readIndex1 = 0;              
// Running total
float total1 = 0;
// Average                  
float average1 = 0;

// Variables for Thermocouple 2
// Calculated temperature value   
float temperature2 = 0;
// Array of readings from the analog input 
float readings2[numReadings];
// Index of the current reading
int readIndex2 = 0;
// Running total
float total2 = 0;
// Average
float average2 = 0;

// Variable for PWM duty cycle
int dutyCycle = 0;


void setup() {
  // Set the pin for the On/Off switch as output
  pinMode(swPin, OUTPUT);
  // Begin serial communications
  Serial.begin(115200);
  // Call function to setup wifi commumications
  setup_wifi();
  // Connect to the MQTT Server
  client.setServer(mqtt_server, 1883);
  // Set the callback function
  client.setCallback(callback);
  // Configure LED PWM
  ledcSetup(ledChannel, freq, resolution);
  // Attach the channel to the GPIO to be controlled    
  ledcAttachPin(pwmPin, ledChannel);    
  // Allow the hardware to stabilise 
  delay(1500);
  // Initialize all the temperature reading arrays to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings1[thisReading] = 0;
    readings2[thisReading] = 0;
  }
}

// Connect the ESP-32 to the wifi network using the supplied credentials
void setup_wifi() {
  delay(10);
  
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Triggered every time the MQTT server publishes a message
// on a topic that the ESP-32 is subscribed to
void callback(char* topic, byte* message, unsigned int length) {
  // Display the message on the serial monitor
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // ON/OFF SWITCH MESSAGES
  // If a message is received on the topic esp32/output, 
  // the if the message is "on", make output pin high
  // and if the message is "off", make output pin low
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(swPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(swPin, LOW);
    }
  }

  // BRIGHTNESS MESSAGES
  // If a message is received on the topic esp32/brightness,
  // ouptut PWM at the duty cycle determined by the message.
  if(String(topic) == "esp32/brightness") {
    Serial.print("Changing brightness to ");
    Serial.println(messageTemp);
    // Turn the message into an integer
    dutyCycle = messageTemp.toInt();
    // A duty cycle in the range 0% to 35% has no effect on brightness
    dutyCycle = dutyCycle + 35;
    // Map the duty cycle (35% to 135%) to a range of 0 to 255  
    dutyCycle = map(dutyCycle, 135, 0, 0, 255);
    // Output PWM at the duty cyle
    ledcWrite(ledChannel, dutyCycle);
  }
}

// Reconnect to MQTT server if connection is lost
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
      client.subscribe("esp32/brightness");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Function to calculate an accurate thermocouple voltage,
// read on the analog input pin of the ESP32.
// Required because the ESP32 analog to digital converter
// is non-linear
double ReadVoltage(byte pin){
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if(reading < 1 || reading >= 4095) return 0;
  if(reading < 1 || reading > 4095) return 0;
  return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
}

void loop() {
  // Check that the MQTT server is connected
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Temperature readings are made every 5 milliseconds
  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

  // TEMPERATURE READINGS

  // THERMOCOUPLE 1
  // Calculate running average of voltage readings
    
  // Subtract the last reading from the running total
  total1 = total1 - readings1[readIndex1];
  // Read a voltage of the thermocouple using
  // accurate reading function
  readings1[readIndex1] = ReadVoltage(tcPin1);
  // Add the reading to the running total
  total1 = total1 + readings1[readIndex1];
  // Go to the next position in the array
  readIndex1 = readIndex1 + 1; 
  // if we're at the end of the array ...
  if (readIndex1 >= numReadings) {
    // ...wrap around to the beginning:
    readIndex1 = 0;
  }
  // Calculate the average value of voltage readings
  average1 = total1 / numReadings;
  
  // Calculate the thermocouple temperature using
  // the fuction supplied with the AD8495 thermocouple amplifier
  temperature1 = (average1-1.250)/0.005;
     
  // Convert the temperature value to a char array
  char tempString1[8];
  dtostrf(temperature1, 1, 2, tempString1);
  // Display the temperature on the serial monitor 
  Serial.print("Temperature 1: ");
  Serial.println(temperature1);
  Serial.println(tempString1);
  // Publish the value to the MQTT server
  // on the topic "esp32/temperature1" 
  client.publish("esp32/temperature1", tempString1);

  // THERMOCOUPLE 2
  // Calculate running average of voltage readings
    
  // Subtract the last reading from the running total
  total2 = total2 - readings2[readIndex2];
  // Read a voltage of the thermocouple using
  // accurate reading function
  readings2[readIndex2] = ReadVoltage(tcPin2);
  // Add the reading to the running total
  total2 = total2 + readings2[readIndex2];
  // Go to the next position in the array
  readIndex2 = readIndex2 + 1;
   // if we're at the end of the array...
  if (readIndex2 >= numReadings) {
    // ...wrap around to the beginning:
    readIndex2 = 0;
  }

  // Calculate the average value of voltage readings
  average2 = total2 / numReadings; // calculate the average
  
  // Calculate the thermocouple temperature using
  // the fuction supplied with the AD8495 thermocouple amplifier
  temperature2 = (average2-1.250)/0.005;
     
  // Convert the temperature value to a char array
   char tempString2[8];
   dtostrf(temperature2, 1, 2, tempString2);
  // Display the temperature on the serial monitor 
   Serial.print("Temperature 2: ");
   Serial.println(temperature2);
   Serial.println(tempString2);
  // Publish the value to the MQTT server
  // on the topic "esp32/temperature1" 
  client.publish("esp32/temperature2", tempString2);

  // delay between readings to improve stability
  delay(1000);
   }
 }

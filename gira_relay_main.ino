/*
 * Gira relay integration
 * Sends and receives values via MQTT
 * 
 * Copyright K.Schmolders 03/2020
 */

// wifi credentials stored externally and .gitignore
 //all wifi credential and MQTT Server importet through wifi_credential.h
 #include "wifi_credentials.h"
 #include <Arduino.h>

 //required for MQTT
 #include <ESP8266WiFi.h>
 //required for OTA updater
 #include <WiFiClient.h>
 #include <ESP8266WebServer.h>
 #include <ESP8266mDNS.h>
 #include <ESP8266HTTPUpdateServer.h>
 //end OTA requirements
 #include <PubSubClient.h>
 
 
 //TODO check SONOFF Pins
 //SONOF Pinout
 // 14 = lowest on pin array (furthest away from button)
 // 0 = button
 // 13 = led SONOFF
 //2 LED NODEMCU
 uint16_t RELAY_IN_PIN = 14; 
 uint16_t LED_PIN = 2; //13
 uint16_t BUTTON_PIN = 0;

 
 
 //timer
 int timer_update_state_count;
 int timer_update_state = 60000; //update status via MQTT every minute
  
 //MQTT (see also wifi_credentials)
 WiFiClient espClient;
 PubSubClient client(espClient);
 
 const char* inTopic = "cmnd/gira_bell/#";
 const char* outTopic = "stat/gira_bell/";
 const char* mqtt_id = "gira_bell";
 
 //flag to notify rining happend in main
 int ringing;
 //timer for rining counting on mqtt
 int ring_timer;
 //duration until ring cancellation
 int ring_duration = 3000; //rining mqtt for 3s

 //OTA
 ESP8266WebServer httpServer(80);
 ESP8266HTTPUpdateServer httpUpdater;

 //WebServer
 const char* sw_version = "gira_relay_main v2";

//Declarations
void send_bell_status();

//interrupt function to handle gira ring
ICACHE_RAM_ATTR void ring_on() {
    if (ringing==0){
        //only ring when not already in ringing mode (1,2)
        ringing = 1;
    }
}
  //web server to provide sw version
  void handle_root() {
   //print sw version on root
   String html_code = String("<p>SW: ");
   html_code += String(sw_version);
   html_code += "</p><p>Ring Status: ";
   html_code += String(ringing);
   html_code += "</p>";
   html_code += "<form action=\"/restart\" method=\"POST\"><input type=\"submit\" value=\"Restart\"></form>";
   html_code += "<form action=\"/led\" method=\"POST\"><input type=\"submit\" value=\"Toggle LED\"></form>";
   html_code += "<form action=\"/ring\" method=\"POST\"><input type=\"submit\" value=\"Ring\"></form>";
   html_code += "<form action=\"/status\" method=\"POST\"><input type=\"submit\" value=\"Send Status\"></form>";


   httpServer.send(200, "text/html", html_code);
  }

  void handle_restart() {                          // If a POST request is made to URI /reset
    Serial.println("Restarting...");
         
    httpServer.sendHeader("Location","/");        // Add a header to respond with a new location for the browser to go to the home page again
    httpServer.send(303);                         // Send it back to the browser with an HTTP status 303 (See Other) to redirect
    ESP.restart(); 
  }

  void handleNotFound(){
    httpServer.send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
  }

  void handle_led() {                          
    digitalWrite(LED_PIN,!digitalRead(LED_PIN));
         
    httpServer.sendHeader("Location","/");        
    httpServer.send(303);                         
  }
  void handle_ring() {                          
    //do ring emulation
    ring_on();
    httpServer.sendHeader("Location","/");        
    httpServer.send(303);                         
  }
  void handle_status() {                          
    //send status
    send_bell_status();
    httpServer.sendHeader("Location","/");        
    httpServer.send(303);                         
  }

 void setup_wifi() {
   delay(10);
   // We start by connecting to a WiFi network
   Serial.println();
   Serial.print("Connecting to ");
   Serial.println(ssid);
   WiFi.persistent(false);
   WiFi.mode(WIFI_OFF);
   WiFi.mode(WIFI_STA);
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
   }
     
   Serial.println("");
   Serial.println("WiFi connected");
   Serial.println("IP address: ");
   Serial.println(WiFi.localIP());
 
   httpUpdater.setup(&httpServer);
   httpServer.begin();
   httpServer.on("/", HTTP_GET, handle_root);     
   httpServer.on("/restart", HTTP_POST, handle_restart);  
   httpServer.on("/led", HTTP_POST, handle_led);
   httpServer.on("/ring", HTTP_POST, handle_ring);
   httpServer.on("/status", HTTP_POST, handle_status);
   httpServer.onNotFound(handleNotFound);

 }
 
 //callback function for MQTT client
 void callback(char* topic, byte* payload, unsigned int length) {
   payload[length]='\0'; // Null terminator used to terminate the char array
   String message = (char*)payload;
 
   Serial.print("Message arrived on topic: [");
   Serial.print(topic);
   Serial.print("]: ");
   Serial.println(message);
   
   //get last part of topic 
   char* cmnd = "test";
   char* cmnd_tmp=strtok(topic, "/");
 
   while(cmnd_tmp !=NULL) {
     cmnd=cmnd_tmp; //take over last not NULL string
     cmnd_tmp=strtok(NULL, "/"); //passing Null continues on string
     //Serial.println(cmnd_tmp);    
   }
 
    if (!strcmp(cmnd, "status")) {
        Serial.print("Received status request. sending status");
        send_bell_status();
    }
    else if (!strcmp(cmnd, "reset")) {
        Serial.print(F("Reset requested. Resetting..."));
        //software_Reset();
    }
 }



//sends module status via MQTT
void send_bell_status()
 {
   char outTopic_status[50];
   char msg[50];
   //IP Address
   strcpy(outTopic_status,outTopic);
   strcat(outTopic_status,"ip_address");
    WiFi.localIP().toString().toCharArray(msg,50);
   client.publish(outTopic_status,msg ); 

  //Ring 0
    strcpy(outTopic_status,outTopic);
    strcat(outTopic_status,"ring");
    client.publish(outTopic_status,"0"); 
 }
 
 //send Ring via MQTT
 void sendRingOn(){
    
    char outTopic_status[50];
    char msg[50];

    //bell status
    strcpy(outTopic_status,outTopic);
    strcat(outTopic_status,"ring");
    //dtostrf(garage_door_status,1,0,msg); 
    client.publish(outTopic_status,"1" ); 
 
 }
 //send Ring via MQTT
 void sendRingOff(){
    
    char outTopic_status[50];
    char msg[50];

    //Bell status
    strcpy(outTopic_status,outTopic);
    strcat(outTopic_status,"ring");
    client.publish(outTopic_status,"0" ); 
 
 }
 
 void reconnect() {
   // Loop until we're reconnected
   
   while (!client.connected()) {
     Serial.print("Attempting MQTT connection...");
     // Attempt to connect
     if (client.connect(mqtt_id)) {
       Serial.println("connected");
       
       client.publish(outTopic, "gira bell relay booted");
       
       
       // ... and resubscribe
       client.subscribe(inTopic);
 
     } else {
       Serial.print("failed, rc=");
       Serial.print(client.state());
       Serial.println(" try again in 5 seconds");      
       delay(5000);
     }
   }
 }
 



 void setup() {
   // Status message will be sent to the PC at 115200 baud
   Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
   Serial.println("Gira Bell Module");
   //INIT TIMERS
   timer_update_state_count=millis();


   pinMode(RELAY_IN_PIN, INPUT);
   //interrupt on relay pin. FALLING since using pull_up to 3.3. relay closes to GND. 
   attachInterrupt(digitalPinToInterrupt(RELAY_IN_PIN), ring_on, FALLING);
   pinMode(BUTTON_PIN, INPUT);
   attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), ring_on, RISING);
    pinMode(LED_PIN, OUTPUT);
    //LED is reverse. HIGH = OFF
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);

    ringing=0;
 
   //WIFI and MQTT
   setup_wifi();                   // Connect to wifi 
   client.setServer(mqtt_server, 1883);
   client.setCallback(callback);
 
    send_bell_status();
 }
 
 
 void loop() {
   if (!client.connected()) {
     reconnect();
   }
   client.loop();
   
   
   //http Updater for OTA
   httpServer.handleClient(); 
 
    if (ringing==1) {
        //alert on gira bus happend
        //set timer 
        ring_timer=millis();
        //next state for flag
        ringing=2;
        //send MQTT ring
        sendRingOn();
        digitalWrite(LED_PIN, LOW); //LOW=ON

    }
    if(millis()-ring_timer > ring_duration && ringing==2) {
        //reset flag
        ringing=0; 
        //send MQTT cancel
        sendRingOff();
        digitalWrite(LED_PIN, HIGH); //HIGH = OFF

    }
    //send status update via MQTT as configured
    if(millis()-timer_update_state_count > timer_update_state) {
      //addLog_P(LOG_LEVEL_INFO, PSTR("Serial Timer triggerd."));
      timer_update_state_count=millis();
      send_bell_status();
    }
   
 }
 
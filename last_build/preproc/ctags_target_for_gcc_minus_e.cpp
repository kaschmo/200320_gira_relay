# 1 "/Users/karsten/Documents/00_Project_Support/10_arduino/Projects/200320_gira_relay/gira_relay_main.ino"
# 1 "/Users/karsten/Documents/00_Project_Support/10_arduino/Projects/200320_gira_relay/gira_relay_main.ino"
/*
 * Gira relay integration
 * Sends and receives values via MQTT
 * 
 * GPIO In for 24V signal of gira bus coupler
 * Copyright K.Schmolders 03/2020
 */

// wifi credentials stored externally and .gitignore
 //all wifi credential and MQTT Server importet through wifi_credential.h
# 12 "/Users/karsten/Documents/00_Project_Support/10_arduino/Projects/200320_gira_relay/gira_relay_main.ino" 2

 //required for MQTT
# 15 "/Users/karsten/Documents/00_Project_Support/10_arduino/Projects/200320_gira_relay/gira_relay_main.ino" 2
 //required for OTA updater
# 17 "/Users/karsten/Documents/00_Project_Support/10_arduino/Projects/200320_gira_relay/gira_relay_main.ino" 2
# 18 "/Users/karsten/Documents/00_Project_Support/10_arduino/Projects/200320_gira_relay/gira_relay_main.ino" 2
# 19 "/Users/karsten/Documents/00_Project_Support/10_arduino/Projects/200320_gira_relay/gira_relay_main.ino" 2
# 20 "/Users/karsten/Documents/00_Project_Support/10_arduino/Projects/200320_gira_relay/gira_relay_main.ino" 2
 //end OTA requirements
# 22 "/Users/karsten/Documents/00_Project_Support/10_arduino/Projects/200320_gira_relay/gira_relay_main.ino" 2
# 23 "/Users/karsten/Documents/00_Project_Support/10_arduino/Projects/200320_gira_relay/gira_relay_main.ino" 2


 //TODO check SONOFF Pins
 //SONOF Pinout
 // 14 = lowest on pin array (furthest away from button)
 // 0 = button
 // 13 = led
 uint16_t RELAY_IN_PIN = 14;
 uint16_t LED_PIN = 13;
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

   while(cmnd_tmp !=__null) {
     cmnd=cmnd_tmp; //take over last not NULL string
     cmnd_tmp=strtok(__null, "/"); //passing Null continues on string
     //Serial.println(cmnd_tmp);    
   }

    if (!strcmp(cmnd, "status")) {
        Serial.print("Received status request. sending status");
        send_status();
    }
    else if (!strcmp(cmnd, "reset")) {
        Serial.print(((reinterpret_cast<const __FlashStringHelper *>((__extension__({static const char __c[] __attribute__((section(".irom.text"))) = ("Reset requested. Resetting..."); &__c[0];}))))));
        //software_Reset();
    }
 }



//sends module status via MQTT
void send_status()
 {
   char outTopic_status[50];
   char msg[50];
   //IP Address
   strcpy(outTopic_status,outTopic);
   strcat(outTopic_status,"ip_address");

   //ESP IP
   WiFi.localIP().toString().toCharArray(msg,50);
   client.publish(outTopic_status,msg );
 }

 //send Ring via MQTT
 void sendRingOn(){

    char outTopic_status[50];
    char msg[50];

    //GARAGE Door status
    strcpy(outTopic_status,outTopic);
    strcat(outTopic_status,"ring");
    //dtostrf(garage_door_status,1,0,msg); 
    client.publish(outTopic_status,"1" );

 }
 //send Ring via MQTT
 void sendRingOff(){

    char outTopic_status[50];
    char msg[50];

    //GARAGE Door status
    strcpy(outTopic_status,outTopic);
    strcat(outTopic_status,"ring");
    //dtostrf(garage_door_status,1,0,msg); 
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

//interrupt function to handle gira ring
void ring_on() {
    ringing = 1;
}

//interrupt function to handle gira ring
void ring_off() {
    //not needed. rining flag is cancelled in loop
}

 void setup() {
   // Status message will be sent to the PC at 115200 baud
   Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
   Serial.println("Gira Bell Module");
   //INIT TIMERS
   timer_update_state_count=millis();


   //TODO assign interrupt
   pinMode(RELAY_IN_PIN, 0x00);
   attachInterrupt((((RELAY_IN_PIN) < 16)? (RELAY_IN_PIN) : -1), ring_on, 0x01);
   pinMode(BUTTON_PIN, 0x00);
   attachInterrupt((((BUTTON_PIN) < 16)? (BUTTON_PIN) : -1), ring_on, 0x01);
    pinMode(LED_PIN, 0x01);
    ringing=0;

   //WIFI and MQTT
   setup_wifi(); // Connect to wifi 
   client.setServer(mqtt_server, 1883);
   client.setCallback(callback);


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
        digitalWrite(LED_PIN, 0x1);

    }
    if(millis()-ring_timer > ring_duration && ringing==2) {
        //reset flag
        ringing=0;
        //send MQTT cancel
        sendRingOff();
        digitalWrite(LED_PIN, 0x0);

    }

 }

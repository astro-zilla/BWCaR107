#include <WiFiNINA.h>
#include <ArduinoJson.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

#include "z_secrets.h"

char ssid[] = SECRET_SSID;  // your WPA2 enterprise network SSID (name)
char user[] = SECRET_USER;  // your WPA2 enterprise username
char pass[] = SECRET_PASS;  // your WPA2 enterprise password
char identity[] = SECRET_IDENTITY;

int keyIndex = 0;                       // your network key index number (needed only for WEP)

IPAddress server(10,248,151,151);
int port = 53282;

// wifi setup
int status = WL_IDLE_STATUS;
WiFiClient client;
byte mac[6]; //84:CC:A8:2B:E1:74 -- m4qjzq6z7adbuqfg

// time for ping calcs
long time = 0;

//motor speeds
int L_speed;
int R_speed;

// servo angles
int pincer_ang;
int arm_ang;

// motor stuff
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor_L = AFMS.getMotor(3);
Adafruit_DCMotor *motor_R = AFMS.getMotor(4);

Servo pincer;
Servo arm;

DynamicJsonDocument daedalus(256);

// ###REMOVE BEFORE FIRE###
void printWifiStatus() {
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your board's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

void test() {
    Serial.println("TESTING");

}

void reconnect() {
    Serial.println("retrying connection to server");
    if (client.connect(server,port)) {
        Serial.print("connected to: ");
        Serial.println(server);
    }
}

void setup() {

    Serial.begin(9600);     //Initialize serial and wait for port to open: ###REMOVE BEFORE FIRE###

    Serial.println("initialising...");

    // check for motor shield
    if (!AFMS.begin()) {
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }

    motor_L->setSpeed(0);
    motor_R->setSpeed(0);

    motor_L->run(RELEASE);
    motor_R->run(RELEASE);

    pincer.attach(9,730,3350); // pin,min,max (us pulse)
    arm.attach(10,730,3350);

    // check for the WiFi module: ###REMOVE BEFORE FIRE###
    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("Communication with WiFi module failed!");
        // hang
        while (true);
    }

    // check firmware is up to date ###REMOVE BEFORE FIRE###
    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
        Serial.println("Please upgrade the firmware");
    }

    // attempt to connect to WiFi network:
    WiFi.setTimeout(5*1000);
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
        // status = WiFi.begin(ssid, pass);
        status = WiFi.beginEnterprise(ssid, user, pass, identity);
    }
    Serial.println("Connected to WiFi");
    printWifiStatus(); // ###REMOVE BEFORE FIRE###

    //int err = WiFi.hostByName('dell-g5.eduroam.jc',server);
    //if ( err == 1 ){ err = WiFi.hostByName('dell-g5.wireless.private.cam.ac.uk',server); }
    //if ( err == 1 ){ Serial.println("Error resolving name"); }
    Serial.print("Attempting to connect to IP: ");
    Serial.println(server);
    if (client.connect(server,port)) {
        Serial.print("connected to: ");
        Serial.println(server);
    }
}

void loop() {
    //start 3 second timeout

    // TIMER IE DOES NOT WORK --- 2:30AM ELLIS SAYS USE AN EXTERNAL RC CIRCUIT OR 555 TIMER AS AN EXTERNAL IE

    // deserialise JSON data over WiFi from Daedalus on controller
    while (client.available()) {

        DeserializationError error = deserializeJson(daedalus, client);
        if (error){
            Serial.print("DeserializationError");
        }
        Serial.println("client");
    }
    Serial.println("loop");


    time = daedalus["time"];

    L_speed = daedalus["motors"][0];
    R_speed = daedalus["motors"][1];
    pincer_ang = daedalus["servos"][0];
    arm_ang = daedalus["servos"][1];

    pincer.write(pincer_ang); // only steps of 1 deg
    arm.write(arm_ang);

    //use bit manipulation here for better code
    motor_L->setSpeed(abs(L_speed));
    motor_R->setSpeed(abs(R_speed));
    if (R_speed>0) {
        motor_R->run(FORWARD);
    }
    else if (R_speed<0) {
        motor_R->run(BACKWARD);
    }
    if (L_speed>0) {
        motor_L->run(FORWARD);
    }
    else if (L_speed<0) {
        motor_L->run(BACKWARD);
    }




    // create JSON object for sensor data
    DynamicJsonDocument theseus(256);
    theseus["time"] = time;
    theseus["ultrasonic0"] = 0;
    theseus["ultrasonic0"] = 0;
    theseus["IR0"] = 0;
    theseus["IR1"] = 0;
    theseus["IR2"] = 0;
    theseus["IR3"] = 0;

    // output JSON object as string to Daedalus
    serializeJson(theseus,client);
    client.write('\n');

    // simple reconnection algorithm, detection of disconnect could be improved
    // also add interrupt that causes reconnect
    if (!client.connected()) {
        reconnect();
    }
}


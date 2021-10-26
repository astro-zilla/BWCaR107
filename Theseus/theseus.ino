#define echoPin 2
#define trigPin 3
#define servoPin0 9
#define servoPin1 10
#define motorPinL 3
#define motorPinR 4
#define magnetometerPin A0

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

IPAddress server(10,248,152,107);
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
int angle_0;
int angle_1;

// sensor data
int ultrasonic;
int magnetometer;

// motor stuff
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor_L = AFMS.getMotor(motorPinL);
Adafruit_DCMotor *motor_R = AFMS.getMotor(motorPinR);

Servo servo_0;
Servo servo_1;

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

int ping(int trigger, int echo) {
    digitalWrite(trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);

    return pulseIn(echo, HIGH) * 0.034 / 2;
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

    servo_0.attach(servoPin0,730,3350); // pin,min,max (us pulse)
    servo_1.attach(servoPin1,730,3350);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

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
    }


    time = daedalus["time"];

    L_speed = daedalus["motors"][0];
    R_speed = daedalus["motors"][1];
    angle_0 = daedalus["servos"][0];
    angle_1 = daedalus["servos"][1];

    // write servo angles: todo in steps of 1 deg
    servo_0.write(angle_0);
    servo_1.write(angle_1);

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

    // get sensor data
    ultrasonic = 0; // ping(trigPin,echoPin);
    magnetometer = 0; // analogRead(magnetometerPin);

    // create JSON object for sensor data
    DynamicJsonDocument theseus(256);
    theseus["time"] = time;
    theseus["ultrasonic"] = ultrasonic;
    theseus["magnetometer"] = magnetometer;

    // output JSON object as string to Daedalus
    serializeJson(theseus,client);
    client.write('\n');

    // simple reconnection algorithm, detection of disconnect could be improved
    // also add interrupt that causes reconnect
    if (!client.connected()) {
        reconnect();
    }
}


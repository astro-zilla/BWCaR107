#include <WiFiNINA.h>
#include <ArduinoJson.h>

#include "z_secrets.h"

char ssid[] = SECRET_SSID;              // your network SSID (name)
char pass[] = SECRET_PASS;              // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                       // your network key index number (needed only for WEP)

IPAddress server(172,20,10,11);         // dell-g5 over Orang
int port = 53282;

int status = WL_IDLE_STATUS;
WiFiClient client;

long time = 0;

void setup() {

    Serial.begin(9600);     //Initialize serial and wait for port to open: ###REMOVE BEFORE FIRE###
    while (!Serial) {}      // wait for serial port to connect. Needed for native USB port only
    Serial.println("initialising...");

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
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
        status = WiFi.begin(ssid, pass);

        // wait 10 seconds for connection:S
        // delay(5000);
    }
    Serial.println("Connected to WiFi");
    printWifiStatus(); // ###REMOVE BEFORE FIRE###

    if (client.connect(server,port)) {
        Serial.print("connected to: ");
        Serial.println(server);
    }
}

void loop() {
    // deserialise JSON data over WiFi from Daedalus on controller
    DynamicJsonDocument daedalus(256);
    DeserializationError error = deserializeJson(daedalus, client);
    if (error)
        return;

    time = daedalus["time"];
    Serial.println(time);

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
        Serial.println();
        Serial.println("retrying connection to server");
        if (client.connect(server,port)) {
            Serial.print("connected to: ");
            Serial.println(server);
        }
    }
}

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

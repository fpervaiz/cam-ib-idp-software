#include <SPI.h>
#include <PubSubClient.h>
#include <WiFiNINA.h>

#define const_topic_bot_serial "/idp/bot/serial"
#define const_topic_bot_cmd "/idp/bot/cmd"
#define const_topic_bot_stt "/idp/bot/stt"
#define const_topic_bot_stage "/idp/bot/stage"

char ssid[] = "IDP_L101";
char pass[] = ">r063W83";
int status = WL_IDLE_STATUS;
IPAddress mqtt_server(192, 168, 137, 1);

WiFiClient net;
PubSubClient mqc(net);

// Wifi functions

void printWifiData()
{
    // print your board's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
    Serial.println(ip);

    // print your MAC address:
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.print("MAC address: ");
    printMacAddress(mac);
}

void printCurrentNet()
{
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print the MAC address of the router you're attached to:
    byte bssid[6];
    WiFi.BSSID(bssid);
    Serial.print("BSSID: ");
    printMacAddress(bssid);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.println(rssi);

    // print the encryption type:
    byte encryption = WiFi.encryptionType();
    Serial.print("Encryption Type:");
    Serial.println(encryption, HEX);
    Serial.println();
}

void printMacAddress(byte mac[])
{
    for (int i = 5; i >= 0; i--)
    {
        if (mac[i] < 16)
        {
            Serial.print("0");
        }
        Serial.print(mac[i], HEX);
        if (i > 0)
        {
            Serial.print(":");
        }
    }
    Serial.println();
}

void setupWifi()
{
    // check for the WiFi module:
    if (WiFi.status() == WL_NO_MODULE)
    {
        Serial.println("Communication with WiFi module failed!");
        // don't continue
        while (true)
            ;
    }

    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION)
    {
        Serial.println("Please upgrade the firmware");
    }

    // attempt to connect to Wifi network:
    while (status != WL_CONNECTED)
    {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        // Connect to WPA/WPA2 network:
        status = WiFi.begin(ssid, pass);

        // wait 10 seconds for connection:
        delay(10000);
    }

    // you're connected now, so print out the data:
    Serial.print("You're connected to the network");
    printCurrentNet();
    printWifiData();
}

// MQTT functions

void onMessageReceived(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);

    Serial.print("] ");
    for (int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    String strTopic = String((char*)topic);
    int strLen = strTopic.length() + 1; 
    char buf[strLen];
    strTopic.toCharArray(buf, strLen);

    mqc.publish(const_topic_bot_serial, "Recieved message");
    mqc.publish(const_topic_bot_serial, buf);

    if (strcmp(buf, const_topic_bot_cmd) == 0) {
        Serial.println("command topic");
    }
    else if (strcmp(buf, const_topic_bot_stt) == 0) {
        Serial.println("state topic");
    }
    else {
        Serial.println("unrecognised topic");
    }
}

void connectMqtt()
{
    while (!mqc.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (mqc.connect("arduinoClient"))
        {
            Serial.println("connected");
            // Once connected, publish an announcement...
            mqc.publish("/idp/bot/stt", "Arduino connected.");
            // ... and resubscribe
            mqc.subscribe("/idp/bot/cmd");
            mqc.subscribe("/idp/bot/stt");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqc.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void setupMqtt()
{
    mqc.setServer(mqtt_server, 1883);
    mqc.setCallback(onMessageReceived);

    connectMqtt();
}

void setup()
{
    //Initialize serial and wait for port to open:
    Serial.begin(9600);
    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    setupWifi();
    setupMqtt();
}

void loop()
{
    // check the network connection once every 10 seconds:
    /*
    delay(10000);
    printCurrentNet();

    if (!mqc.connected())
    {
        connectMqtt();
    }
    mqc.publish("/idp/bot/stt", "Hello!");
    */

    mqc.loop();
}
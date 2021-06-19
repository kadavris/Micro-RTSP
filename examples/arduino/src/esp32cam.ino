//NOTE! This example is intended for the ESP32-CAM from Ai-Thinker. Other ESP32-based camera boards are supported,
//but will require modification to this example. This is just a bare-bones example of Micro-RTSP, no web server
//or other features are provided. Although this uses the Arduino framework it is intended to be built with
//PlatformIO.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <OV2640Streamer.h>
#include <CRtspSession.h>
#include "time.h"

//device specific settings
#define DEVICE_NAME "ESP32CAM1"
#define ENABLE_OTA
//#define ENABLE_MQTT
//#define IGNORE_NEW_CLIENTS //if defined, new connections will be ignore; if undefined, old connections are dropped for new ones

//WIFI settings
#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_PASSWORD"

//common camera settings, see below for other camera options
#define CAM_FRAMESIZE FRAMESIZE_VGA //enough resolution for a simple security cam
#define CAM_FRAMEBUFFERS 2 //need double-buffers for reasonable framerate
#define CAM_QUALITY 12 //0 to 63, with higher being lower-quality (and less bandwidth), 12 seems reasonable
#define CAM_FRAMERATE 10 //frames per second, only going to get about 10fps with the ESP32
#define CAM_VERTICAL_FLIP 1 //1 to enable
#define CAM_HORIZONTAL_MIRROR 1 //1 to enable

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;

#ifdef ENABLE_OTA
//Arduino OTA Config
#define OTA_NAME DEVICE_NAME
#define OTA_PORT 3232 //default
#define OTA_PASS NULL

#include <ArduinoOTA.h>
#endif

#ifdef ENABLE_MQTT
#include <PubSubClient.h>
WiFiClient esp_client;
PubSubClient mqtt_client(esp_client);

//MQTT Config
#define MQTT_SERVER "192.168.1.163"   //DNS resolving on EPS32 arduino is broken "homeassistant.localdomain"
#define MQTT_PORT 1883
#define MQTT_USER "mqtt"
#define MQTT_PASS "mqtt"
#define MQTT_CLIENT DEVICE_NAME
#define MQTT_LED_TOPIC "home/" DEVICE_NAME "/set/led"
#define MQTT_WILL_TOPIC "home/" DEVICE_NAME "/connected"
#define MQTT_WILL_MSG "false"
#define MQTT_LED_STATE_TOPIC "home/" DEVICE_NAME "/state/led"
#define MQTT_CONNECT_RETRY 30000 //try to reconnect every thirty seconds only

//LED PWM properties
const int ledPin = 4;
const int ledFreq = 1000;
const int ledChannel = 1; //0 is used by the camera
const int ledResolution = 8;
unsigned long mqtt_last_reconnect = 0;
#endif

OV2640 cam;
CStreamer *streamer = NULL;
CRtspSession *session = NULL;
WiFiClient client;
WiFiServer rtspServer(8554);
const uint32_t msecPerFrame = (1000 / CAM_FRAMERATE);
uint32_t lastFrameTime = 0;

#ifdef ENABLE_MQTT
void mqtt_client_callback(char* topic, uint8_t* payload, unsigned int length)
{
  char payload_string[8];
  char payload_string_len = length;
  int intensity = 0;

  //constrain the length of the payload
  if (payload_string_len >= sizeof(payload_string))
  {
    payload_string_len = sizeof(payload_string) - 1;
  }

  //convert to a C-string, and then convert to a number
  memset(payload_string, 0, sizeof(payload_string));
  memcpy(payload_string, payload, payload_string_len);
  intensity = atoi((char*)payload_string);

  //set LED
  //digitalWrite(4, intensity);
  ledcWrite(ledChannel, intensity);
}

void mqtt_reconnect()
{
  if (mqtt_client.connected() == false)
  {
    if ((millis() - MQTT_CONNECT_RETRY) >= mqtt_last_reconnect)
    {
      if (mqtt_client.connect(MQTT_CLIENT, MQTT_USER, MQTT_PASS, MQTT_WILL_TOPIC, 1, true, MQTT_WILL_MSG) == true)
      {
        Serial.println(F("MQTT Connected!"));
        mqtt_client.subscribe(MQTT_LED_TOPIC);
        mqtt_client.publish(MQTT_WILL_TOPIC, "true", true);
      }

      mqtt_last_reconnect = millis();
    }
  }
}
#endif

// cppcheck-suppress unusedFunction
void setup()
{
    Serial.begin(115200);

    //turn off the camera LED
#ifdef ENABLE_MQTT
    //pinMode(4, OUTPUT);
    //digitalWrite(4, 0); //off
    ledcSetup(ledChannel, ledFreq, ledResolution);
    ledcAttachPin(ledPin, ledChannel);
    ledcWrite(ledChannel, 0);
#else
    pinMode(4, OUTPUT);
    digitalWrite(4, 0); //off
#endif

    camera_config_t config = esp32cam_aithinker_config;
    config.frame_size = CAM_FRAMESIZE;
    config.jpeg_quality = CAM_QUALITY; 
    config.fb_count = CAM_FRAMEBUFFERS;
    cam.init(config);

    //setup other camera options
    sensor_t * s = esp_camera_sensor_get();
    //s->set_brightness(s, 0);     // -2 to 2
    //s->set_contrast(s, 0);       // -2 to 2
    //s->set_saturation(s, 0);     // -2 to 2
    //s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 0);       // 0 = disable , 1 = enable //auto white-balance seems to be more trouble than its worth
    //s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    //s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    //s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    //s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    //s->set_ae_level(s, 0);       // -2 to 2
    //s->set_aec_value(s, 300);    // 0 to 1200
    //s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    //s->set_agc_gain(s, 0);       // 0 to 30
    //s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    //s->set_bpc(s, 0);            // 0 = disable , 1 = enable
    //s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    //s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    //s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, CAM_HORIZONTAL_MIRROR); // 0 = disable , 1 = enable
    s->set_vflip(s, CAM_VERTICAL_FLIP);       // 0 = disable , 1 = enable
    //s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    //s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    
    IPAddress ip;

    Serial.print(F("WiFi start..."));
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(DEVICE_NAME);
    WiFi.setSleep(false); //helps stability and performance
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(F("."));
    }

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    ip = WiFi.localIP();
    Serial.println(F("WiFi connected"));
    Serial.println("");
    Serial.println(ip);
    Serial.print(F("Stream Link: rtsp://"));
    Serial.print(ip);
    Serial.println(F(":8554/mjpeg/1"));

#ifdef ENABLE_OTA
    //Setup OTA
    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.setHostname(OTA_NAME);
    ArduinoOTA.setPassword(OTA_PASS);
    ArduinoOTA.begin();
#endif

#ifdef ENABLE_MQTT
    //Setup MQTT
    mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt_client.setCallback(mqtt_client_callback);
    mqtt_reconnect();
#endif

    lastFrameTime = millis();
    rtspServer.begin();
}

// cppcheck-suppress unusedFunction
void loop()
{
#ifdef ENABLE_OTA
    static unsigned long lastOTA = 0;
    const unsigned long msecOTA = 2000; //2 seconds
    unsigned long now = millis();
    if((now > (lastOTA + msecOTA)) || (now < lastOTA))
    {
        ArduinoOTA.handle(); //doing this periodically seems to really help performance
        lastOTA = now;
    }
#endif

#ifdef ENABLE_MQTT
    mqtt_reconnect();
    mqtt_client.loop();
#endif

    //There are two connection schemes to support a single client. The first is a
    //first-come, only served style. This means the first client to connect is given
    //the stream, and all future clients are ignored until the first client disconnects.
    //The second style is last-come, only served. This means it will accept new clients
    //and disconnect the old client. Only the newest client will be the one with the stream.
    //This has been proven to be needed for Agent DVR, which seems to sometimes connect twice
    //to the same camera, but only ever displays the second connection. Disconnecting the old
    //client prevents the first zombie connection from robbing the real client from the stream.
#ifdef IGNORE_NEW_CLIENTS
    // If we have an active client connection, just service that until gone
    if(streamer)
    {
        session->handleRequests(0); // we don't use a timeout here,
        // instead we send only if we have new enough frames

        uint32_t now = millis();
        if((now > (lastFrameTime + msecPerFrame)) || (now < lastFrameTime))
        {
            session->broadcastCurrentFrame(now);
            lastFrameTime = now;
        }

        if(session->m_stopped)
        {
            delete session; //will close the client
            delete streamer;
            session = NULL;
            streamer = NULL;
        }
    }
    else
    {
        //no current client, see if we got a new one
        client = rtspServer.accept();

        if(client)
        {
            //got a new client, start a session
            streamer = new OV2640Streamer(&client, cam);
            session = new CRtspSession(&client, streamer);
        }
    }
#else
    //see if a new client connected
    WiFiClient new_client = rtspServer.accept();

    if(new_client)
    {
        //if we had a session already, close it
        if (streamer)
        {
            delete session; //will close the client
            delete streamer;
            session = NULL;
            streamer = NULL;
        }

        //service just the new client
        client = new_client;
        streamer = new OV2640Streamer(&client, cam);
        session = new CRtspSession(&client, streamer);
    }

    //if we have a current session
    if(streamer)
    {
        session->handleRequests(0); // we don't use a timeout here,
        // instead we send only if we have new enough frames

        uint32_t now = millis();
        if((now > (lastFrameTime + msecPerFrame)) || (now < lastFrameTime))
        {
            session->broadcastCurrentFrame(now);
            lastFrameTime = now;
        }

        if(session->m_stopped)
        {
            delete session; //will close the client
            delete streamer;
            session = NULL;
            streamer = NULL;
        }
    }
#endif
}

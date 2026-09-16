/*
 AMB82 Mini rover camera: streams H.264 video over RTSP to the Raspberry Pi 3,
 which runs person detection and drives the ESP32.

 Network (USE_PI_HOTSPOT below):
   1: joins the Pi 3's Wi-Fi hotspot (./pi.sh hotspot) at a fixed address, so
      the Pi always finds the stream at rtsp://10.42.0.50:554
   0: joins home Wi-Fi; the Serial Monitor prints rtsp://<address>:554

 Arduino IDE, Tools menu (Realtek AmebaPro2 package 4.1.0):
   Board:           AMB82-MINI
   Camera Options:  must match the sensor printed on your camera module
                    (JXF37, GC2053, ...) or the video will not start
   Auto Flash Mode: Enable. If upload still waits for the board: hold the
                    download button, press reset, release, then upload.
 Serial Monitor at 115200 shows the Wi-Fi state and the stream URL.

 Based on the SDK example Multimedia/StreamRTSP/VideoOnly.
*/

#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"

// 1: join the Pi 3's RoverCam hotspot at the fixed address 10.42.0.50 (on the rover)
// 0: join the Wi-Fi below and take an address from its router (testing at home);
//    the Serial Monitor prints the stream URL to use
#define USE_PI_HOTSPOT 0

#if USE_PI_HOTSPOT
char ssid[] = "RoverCam";  // must match HOTSPOT_SSID / HOTSPOT_PASS in pi.sh
char pass[] = "rovercam123";
#else
char ssid[] = "You Know What's Paradoxical :)";
char pass[] = "9@homewifi.com";
#endif

// Pi hotspot (NetworkManager shared mode) is 10.42.0.1/24; used when USE_PI_HOTSPOT is 1
IPAddress camIP(10, 42, 0, 50);
IPAddress piIP(10, 42, 0, 1);
IPAddress subnet(255, 255, 255, 0);

#define CHANNEL 0

// 640x360 keeps the sensor's 16:9 view without distortion, and is small enough
// for the Pi 3 to decode while it runs YOLO. 15 fps is well above what the
// Pi 3 can detect at, so no useful frames are lost.
#define STREAM_FPS 15
#define STREAM_BPS (1 * 1024 * 1024)

// Width/height form: the VIDEO_WVGA preset constant leaves the size at 0x0 in
// this SDK (4.1.0), which starts RTSP but never sends a frame.
VideoSetting config(640, 360, STREAM_FPS, VIDEO_H264, 0);
RTSP rtsp;
StreamIO videoStreamer(1, 1);  // 1 video input -> 1 RTSP output

unsigned long lastCheckMs = 0;

void connectWiFi()
{
#if USE_PI_HOTSPOT
    WiFi.config(camIP, piIP, piIP, subnet);
#endif
    int status = WL_IDLE_STATUS;
    while (status != WL_CONNECTED) {
        Serial.print("Connecting to Wi-Fi ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);
        delay(2000);
    }
    Serial.print("Wi-Fi connected, IP ");
    Serial.print(WiFi.localIP());
    Serial.print(", RSSI ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
}

void setup()
{
    Serial.begin(115200);
    connectWiFi();

    config.setBitrate(STREAM_BPS);

    Camera.configVideoChannel(CHANNEL, config);
    Camera.videoInit();

    rtsp.configVideo(config);
    rtsp.begin();

    videoStreamer.registerInput(Camera.getStream(CHANNEL));
    videoStreamer.registerOutput(rtsp);
    if (videoStreamer.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    Camera.channelBegin(CHANNEL);

    delay(1000);
    Camera.printInfo();
    Serial.print("Stream ready: rtsp://");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(rtsp.getPort());
}

void loop()
{
    // The stream runs on its own; just keep Wi-Fi up and report its health.
    if (millis() - lastCheckMs > 5000) {
        lastCheckMs = millis();
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Wi-Fi lost, reconnecting");
            connectWiFi();
        } else {
            Serial.print("streaming, RSSI ");
            Serial.print(WiFi.RSSI());
            Serial.println(" dBm");
        }
    }
    delay(100);
}

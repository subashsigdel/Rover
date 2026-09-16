/*
 AMB82 Mini as a USB camera for the rover's Raspberry Pi 3 - no Wi-Fi at all.

 The board appears to the Pi as a normal USB camera (UVC), so the video travels
 down the USB cable that already powers it: no router, no RTSP, no reconnects,
 and the least delay. On the Pi:
     ./pi.sh run --source usb --port auto ...

 The SDK's USB camera mode always sends H.264 (UVCD::configVideo forces it), so
 the Pi decodes it with ffmpeg; 640x360 is the same size the Wi-Fi sketch sends.

 Keep rover_cam.ino for the Wi-Fi setup; this is the wired alternative.

 Arduino IDE, Tools menu (Realtek AmebaPro2 package 4.1.0):
   Board:           AMB82-MINI
   Camera Options:  must match the sensor on your camera module (JXF37, GC2053, ...)
   Auto Flash Mode: Enable

 Based on the SDK example USB/UVC_Device.
*/

#include "StreamIO.h"
#include "VideoStream.h"
#include "UVCD.h"

#define CHANNEL 0

#define STREAM_W   640
#define STREAM_H   360
#define STREAM_FPS 15
#define STREAM_BPS (1 * 1024 * 1024)

// Start from the USB preset: configVideoChannel only copies the USB-specific
// encoder settings (GOP, rate control, static buffers) for this preset. Then
// bring the size down from 1080p, which a Pi 3 cannot decode in real time.
VideoSetting config(USB_UVCD_STREAM_PRESET);
Video camera_uvcd;
UVCD usb_uvcd;
StreamIO videoStreamer(1, 1);  // 1 video input -> 1 USB camera output

void setup()
{
    Serial.begin(115200);

    config._resolution = VIDEO_CUSTOM;
    config._w = STREAM_W;
    config._h = STREAM_H;
    config._fps = STREAM_FPS;
    config._bps = STREAM_BPS;

    camera_uvcd.configVideoChannel(CHANNEL, config);
    camera_uvcd.videoInit(CHANNEL);

    usb_uvcd.configVideo(config);

    videoStreamer.registerInput(camera_uvcd.getStream(CHANNEL));
    videoStreamer.registerOutput(usb_uvcd);
    if (videoStreamer.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    camera_uvcd.channelBegin(CHANNEL);
    usb_uvcd.begin(camera_uvcd.getStream(CHANNEL), videoStreamer.linker, CHANNEL);

    Serial.print("USB camera ready: ");
    Serial.print(STREAM_W);
    Serial.print("x");
    Serial.print(STREAM_H);
    Serial.print(" H264 @ ");
    Serial.print(STREAM_FPS);
    Serial.println(" fps");
}

void loop()
{
    delay(1000);
}

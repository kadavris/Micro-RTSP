# Micro-RTSP

This is a small library which can be used to serve up RTSP streams from
resource constrained MCUs.  It lets you trivially make a $10 open source
RTSP video stream camera.

# Usage

This library works for ESP32/arduino targets but also for most any POSIX-ish platform.

## Example arduino/ESP32 usage

This library will work standalone, but it is _super_ easy to use if your app is platform.io based.
Just "pio lib install Micro-RTSP" to pull the latest version from their library server.  If you want to use the OV2640
camera support you'll need to be targeting the espressif32 platform in your project.

See the [example platform.io app](/examples/arduino).  It should build and run on virtually any of the $10
ESP32-CAM boards (such as M5CAM) with some modifications. The example is targeted for the AI-Thinker ESP32-CAM.
The relevant bit of the code is included below.  In short:
1. Listen for a TCP connection on the RTSP port with accept()
2. When a connection comes in, create a CRtspSession and OV2640Streamer camera streamer objects.
3. While the connection remains, call session->handleRequests(0) to handle any incoming client requests.
4. Every 100ms or so (10 FPS) call session->broadcastCurrentFrame() to send new frames to any clients.

```
void loop()
{
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
            delete session;
            delete streamer;
            session = NULL;
            streamer = NULL;
        }
    }
    else
    {
        client = rtspServer.accept();

        if(client)
        {
            streamer = new OV2640Streamer(&client, cam);
            session = new CRtspSession(&client, streamer);
        }
    }
}
```
## Example esp-idf usage
Also included is an example using the Espressif IoT Development Framework. See the [example esp-idf README](/examples/esp-idf/README.md) for further information.

## Example posix/linux usage

There is a small standalone example [here](/test/RTSPTestServer.cpp).  You can build it by following [these](/test/README.md) directions.  The usage of the two key classes (CRtspSession and SimStreamer) are very similar to to the ESP32 usage.

## Supporting new camera devices

Supporting new camera devices is quite simple.  See OV2640Streamer for an example and implement streamImage()
by reading a frame from your camera.

# Structure and design notes

# Issues and sending pull requests

Please report issues and send pull requests.  I'll happily reply. ;-)

# Credits

The server code was initially based on a great 2013 [tutorial](https://www.medialan.de/usecase0001.html) by Medialan.

# License

Copyright 2018 S. Kevin Hester-Chow, kevinh@geeksville.com (MIT License)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

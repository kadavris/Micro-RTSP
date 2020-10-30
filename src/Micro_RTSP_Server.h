/*
  Micro_RTSP_Server - wrapper around Micro-RTSP
  to further simplify rtsp servicing with ESP32-CAM
  Copyright 2020 Andrej Pakhutin. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef _Micro_RTSP_Server_H_
#define _Micro_RTSP_Server_H_

#include "config.h"
#include "Arduino.h"
#include "CRtspSession.h"
#include "OV2640.h"
#include "OV2640Streamer.h"
#include "SimStreamer.h"
#include "WiFiClient.h"
#include "WiFiServer.h"
#include <list>

// helper class to manipulate clients
class Micro_RTSP_Client
{
private:
protected:
    CStreamer *_streamer;
    CRtspSession *_session;
    WiFiClient *_wifi_client;
    int _id;
    int _errors; //consecutive errors on stream.
    bool _active; // still good or ready to be destroyed.
    uint32_t _last_frame_time; // last time we pushed a frame to client
    uint32_t _last_cmd_time;  // last time we got a valid command from client
    void stop(); // prepare to die

public:
    Micro_RTSP_Client( WiFiClient *, CStreamer *, int );
    Micro_RTSP_Client( Micro_RTSP_Client && );
    ~Micro_RTSP_Client();

    uint8_t active();
    bool processCommands();
    bool streamFrame();

    friend class Micro_RTSP_Server;
};

//====================================
class Micro_RTSP_Server
{
  private:
    OV2640 *_cam;
    WiFiServer *_wifi_server;
    CStreamer *_streamer;

    // using lists here ensures no pointer loss on most operations
    std::list<Micro_RTSP_Client> rtsp_clients;
    std::list<WiFiClient *> wifi_clients;
    int _max_clients;
    int _port;

  public:
    Micro_RTSP_Server( OV2640 *cam, uint16_t port = 554, uint8_t max_rtsp_clients = 2 );
    ~Micro_RTSP_Server();

    int activeClients() { return rtsp_clients.size(); }
    void begin( uint16_t port = 0 );
    int port(); // return configured listening port
    bool run(); // do all stuff that is needed and return after single loop
    void stop();
};

#endif /* _Micro_RTSP_Server_H_ */

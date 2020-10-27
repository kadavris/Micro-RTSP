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
#include "Micro_RTSP_Server.h"

Micro_RTSP_Client::Micro_RTSP_Client( WiFiClient *wclient, OV2640 *cam, int id )
{
    _wifi_client = wclient;
    _streamer = new OV2640Streamer( _wifi_client, *cam );    // our streamer for UDP/TCP based RTP transport
    _session = new CRtspSession( _wifi_client, _streamer ); // our threads RTSP session and state
    _id = id;
    _errors = 0;
    _active = true;

    #ifdef DEBUG_MICRO_RTSP_SERVER
    Serial.print( "+ RTSP: New session " ); Serial.print( _id );
    Serial.print( " from " ); Serial.println( _wifi_client->remoteIP().toString().c_str() );
    #endif
}

//===================================
Micro_RTSP_Client::Micro_RTSP_Client( Micro_RTSP_Client && old )
{
    stop();
    _wifi_client = old._wifi_client; // wifi client will be queried for removal by server classs, so we left it in old too
    _streamer = old._streamer; 
    _session = old._session;
    _id = old._id;
    _errors = 0;
    _active = true;

    old._active = false;
    old._id = -1;
    old._session = NULL;
    old._streamer = NULL;
}

//===================================
Micro_RTSP_Client::~Micro_RTSP_Client()
{
    stop();
}

//===================================
void Micro_RTSP_Client::stop()
{
    if ( ! _active )
        return;

    #ifdef DEBUG_MICRO_RTSP_SERVER
    Serial.print( "- RTSP: stopping session ID " ); Serial.println( _id );
    #endif

    if ( _session )
        delete _session;
    if ( _streamer )
        delete _streamer;

    _active = false;
}

//===================================
uint8_t Micro_RTSP_Client::active()
{
    if ( ! _active )
        return 0;

    if ( _session->m_stopped || ! _wifi_client->connected() )
    {
        stop();

        return 0;
    }
    
    return _wifi_client->connected();
}

//===================================
bool Micro_RTSP_Client::processCommands()
{
    if( ! active() )
        return false;

    bool problem;

    try
    {
        problem = ! _session->handleRequests( 0 );
    }
    catch(const std::exception& e)
    {
        Serial.println( e.what() );
        problem = true;
    }
    
    if( problem )
    {
        if ( ++_errors > 10 )
        {
            #ifdef DEBUG_MICRO_RTSP_SERVER
            Serial.println( "" );
            Serial.print( _id ); Serial.println( ": Too many errors in processCommands()" );
            #endif

            stop();
            return false;
        }
    }
    else
    {
        _errors = 0;
    }
    
    return true;
}

//===================================
bool Micro_RTSP_Client::streamFrame()
{
    if( ! active() )
        return false;

    bool problem = false;

    try
    {
        if( _session->m_streaming ) // PLAY command received
        {
            _session->broadcastCurrentFrame( millis() );

            #ifdef DEBUG_MICRO_RTSP_SERVER
            Serial.print( _id );
            #endif
        }
    }
    catch(const std::exception& e)
    {
        Serial.println( e.what() );
        problem = true;
    }
    
    if( problem )
    {
        if ( ++_errors > 10 )
        {
            #ifdef DEBUG_MICRO_RTSP_SERVER
            Serial.println( "" );
            Serial.print( _id ); Serial.println( ": Too many errors in streamFrame()" );
            #endif
            stop();
            return false;
        }
    }
    else
    {
        _errors = 0;
    }
    
    return true;
}

//===================================
//===================================
//===================================
Micro_RTSP_Server::Micro_RTSP_Server( OV2640 *cam, uint16_t port, uint8_t max_rtsp_clients )
{
    _wifi_server = new WiFiServer( port );
    _cam = cam;
    _port = port;
    _max_clients = max_rtsp_clients;
}

//===================================
Micro_RTSP_Server::~Micro_RTSP_Server()
{
    stop();
    delete _wifi_server;
}

//===================================
bool Micro_RTSP_Server::run()
{
    uint32_t msecPerFrame = 100;
    static uint32_t lastimage = millis();
    static uint32_t last_cmd = millis();

    // If we have an active client connection, just service that until gone
    uint32_t now = millis();

    // looks like if the library reads stream too fast it receives fragments of commands
    // and discards them mindlessly. trying to call processor every 1 sec to ensure most of cmd data came in full
    if( now > last_cmd + 1000 || now < lastimage )
    { 
        for( auto it = rtsp_clients.begin(); it != rtsp_clients.end(); ++it )
        {
            (*it).processCommands();
        }

        last_cmd = now;
    }

    // check if it is a time to server another frame
    if( now > lastimage + msecPerFrame || now < lastimage ) // handle clock rollover
    { 
        for( auto it = rtsp_clients.begin(); it != rtsp_clients.end(); ++it )
        {
            (*it).streamFrame();
        }

        lastimage = now;

        // check if we are overrunning our max frame rate
        now = millis();

        #ifdef DEBUG_MICRO_RTSP_SERVER
        if( now > lastimage + msecPerFrame )
        {
            Serial.print( "+ RTSP: warning exceeding max frame rate of " );
            Serial.print( now - lastimage );
            Serial.println( " ms" );
        }
        #endif

        // looking for inactive ones
        for( auto it = rtsp_clients.begin(); it != rtsp_clients.end(); ++it )
        {
            if ( ! (*it).active() )
            {
                #ifdef DEBUG_MICRO_RTSP_SERVER
                Serial.print( "--- RTSP: Erasing inactive client ID " ); Serial.println( (*it)._id );
                #endif

                wifi_clients.remove( *( (*it)._wifi_client ) );

                rtsp_clients.erase( it );
                break; // if there are other disconnects they will be handled on next iteration
            }
        }

        // and adding new
        if ( _wifi_server->hasClient() )
        {
            wifi_clients.push_back( _wifi_server->available() );

            if ( wifi_clients.back().fd() != -1 && rtsp_clients.size() < _max_clients )
                rtsp_clients.push_back( Micro_RTSP_Client( &(wifi_clients.back()), _cam, rtsp_clients.size() ) );
            else
                wifi_clients.pop_back();
        }
    }

    return true;
}

//===================================
void Micro_RTSP_Server::begin( uint16_t port )
{
    if ( port )
        _port = port;

    _wifi_server->begin( port );
    run();
}

//===================================
void Micro_RTSP_Server::stop()
{
    rtsp_clients.clear();
    wifi_clients.clear();
    _wifi_server->end();
}

//===================================
int Micro_RTSP_Server::port()
{
    return _port;
}

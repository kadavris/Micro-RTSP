# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.c esp32cam.cpp ../../../src/CRtspSession.cpp ../../../src/CStreamer.cpp ../../../src/OV2640.cpp ../../../src/OV2640Streamer.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "../../../src")
set(COMPONENT_EMBED_TXTFILES ../server_certs/ca_cert.pem)

register_component()

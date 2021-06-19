#!/usr/bin/python3
import http.server
import os
import re
import socket
import ssl
import sys
from threading import Thread
import time
import paho.mqtt.client as mqtt # pip install paho-mqtt

use_ssl = False # SSL doesn't seem to work...

connected = False
status = ""

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    global connected
    print("Connected with return code: " + str(rc))
    time.sleep(1)
    client.subscribe("devices/" + device + "/ota/status")
    time.sleep(1)
    connected = True

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global status
    status =  msg.payload.decode("utf-8")

server_cert = '-----BEGIN CERTIFICATE-----\n' \
              'MIIDXTCCAkWgAwIBAgIJAP4LF7E72HakMA0GCSqGSIb3DQEBCwUAMEUxCzAJBgNV\n'\
              'BAYTAkFVMRMwEQYDVQQIDApTb21lLVN0YXRlMSEwHwYDVQQKDBhJbnRlcm5ldCBX\n'\
              'aWRnaXRzIFB0eSBMdGQwHhcNMTkwNjA3MDk1OTE2WhcNMjAwNjA2MDk1OTE2WjBF\n'\
              'MQswCQYDVQQGEwJBVTETMBEGA1UECAwKU29tZS1TdGF0ZTEhMB8GA1UECgwYSW50\n'\
              'ZXJuZXQgV2lkZ2l0cyBQdHkgTHRkMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIB\n'\
              'CgKCAQEAlzfCyv3mIv7TlLkObxunKfCdrJ/zgdANrsx0RBtpEPhV560hWJ0fEin0\n'\
              'nIOMpJSiF9E6QsPdr6Q+eogH4XnOMU9JE+iG743N1dPfGEzJvRlyct/Ck8SswKPC\n'\
              '9+VXsnOdZmUw9y/xtANbURA/TspvPzz3Avv382ffffrJGh7ooOmaZSCZFlSYHLZA\n'\
              'w/XlRr0sSRbLpFGY0gXjaAV8iHHiPDYLy4kZOepjV9U51xi+IGsL4w75zuMgsHyF\n'\
              '3nJeGYHgtGVBrkL0ZKG5udY0wcBjysjubDJC4iSlNiq2HD3fhs7j6CZddV2v845M\n'\
              'lVKNxP0kO4Uj4D8r+5USWC8JKfAwxQIDAQABo1AwTjAdBgNVHQ4EFgQU6OE7ssfY\n'\
              'IIPTDThiUoofUpsD5NwwHwYDVR0jBBgwFoAU6OE7ssfYIIPTDThiUoofUpsD5Nww\n'\
              'DAYDVR0TBAUwAwEB/zANBgkqhkiG9w0BAQsFAAOCAQEAXIlHS/FJWfmcinUAxyBd\n'\
              '/xd5Lu8ykeru6oaUCci+Vk9lyoMMES7lQ+b/00d5x7AcTawkTil9EWpBTPTOTraA\n'\
              'lzJMQhNKmSLk0iIoTtAJtSZgUSpIIozqK6lenxQQDsHbXKU6h+u9H6KZE8YcjsFl\n'\
              '6vL7sw9BVotw/VxfgjQ5OSGLgoLrdVT0z5C2qOuwOgz1c7jNiJhtMdwN+cOtnJp2\n'\
              'fuBgEYyE3eeuWogvkWoDcIA8r17Ixzkpq2oJsdvZcHZPIZShPKW2SHUsl98KDemu\n'\
              'y0pQyExmQUbwKE4vbFb9XuWCcL9XaOHQytyszt2DeD67AipvoBwVU7/LBOvqnsmy\n'\
              'hA==\n'\
              '-----END CERTIFICATE-----\n'

server_key = '-----BEGIN PRIVATE KEY-----\n'\
             'MIIEvAIBADANBgkqhkiG9w0BAQEFAASCBKYwggSiAgEAAoIBAQCXN8LK/eYi/tOU\n'\
             'uQ5vG6cp8J2sn/OB0A2uzHREG2kQ+FXnrSFYnR8SKfScg4yklKIX0TpCw92vpD56\n'\
             'iAfhec4xT0kT6Ibvjc3V098YTMm9GXJy38KTxKzAo8L35Veyc51mZTD3L/G0A1tR\n'\
             'ED9Oym8/PPcC+/fzZ999+skaHuig6ZplIJkWVJgctkDD9eVGvSxJFsukUZjSBeNo\n'\
             'BXyIceI8NgvLiRk56mNX1TnXGL4gawvjDvnO4yCwfIXecl4ZgeC0ZUGuQvRkobm5\n'\
             '1jTBwGPKyO5sMkLiJKU2KrYcPd+GzuPoJl11Xa/zjkyVUo3E/SQ7hSPgPyv7lRJY\n'\
             'Lwkp8DDFAgMBAAECggEAfBhAfQE7mUByNbxgAgI5fot9eaqR1Nf+QpJ6X2H3KPwC\n'\
             '02sa0HOwieFwYfj6tB1doBoNq7i89mTc+QUlIn4pHgIowHO0OGawomeKz5BEhjCZ\n'\
             '4XeLYGSoODary2+kNkf2xY8JTfFEcyvGBpJEwc4S2VyYgRRx+IgnumTSH+N5mIKZ\n'\
             'SXWNdZIuHEmkwod+rPRXs6/r+PH0eVW6WfpINEbr4zVAGXJx2zXQwd2cuV1GTJWh\n'\
             'cPVOXLu+XJ9im9B370cYN6GqUnR3fui13urYbnWnEf3syvoH/zuZkyrVChauoFf8\n'\
             '8EGb74/HhXK7Q2s8NRakx2c7OxQifCbcy03liUMmyQKBgQDFAob5B/66N4Q2cq/N\n'\
             'MWPf98kYBYoLaeEOhEJhLQlKk0pIFCTmtpmUbpoEes2kCUbH7RwczpYko8tlKyoB\n'\
             '6Fn6RY4zQQ64KZJI6kQVsjkYpcP/ihnOY6rbds+3yyv+4uPX7Eh9sYZwZMggE19M\n'\
             'CkFHkwAjiwqhiiSlUxe20sWmowKBgQDEfx4lxuFzA1PBPeZKGVBTxYPQf+DSLCre\n'\
             'ZFg3ZmrxbCjRq1O7Lra4FXWD3dmRq7NDk79JofoW50yD8wD7I0B7opdDfXD2idO8\n'\
             '0dBnWUKDr2CAXyoLEINce9kJPbx4kFBQRN9PiGF7VkDQxeQ3kfS8CvcErpTKCOdy\n'\
             '5wOwBTwJdwKBgDiTFTeGeDv5nVoVbS67tDao7XKchJvqd9q3WGiXikeELJyuTDqE\n'\
             'zW22pTwMF+m3UEAxcxVCrhMvhkUzNAkANHaOatuFHzj7lyqhO5QPbh4J3FMR0X9X\n'\
             'V8VWRSg+jA/SECP9koOl6zlzd5Tee0tW1pA7QpryXscs6IEhb3ns5R2JAoGAIkzO\n'\
             'RmnhEOKTzDex611f2D+yMsMfy5BKK2f4vjLymBH5TiBKDXKqEpgsW0huoi8Gq9Uu\n'\
             'nvvXXAgkIyRYF36f0vUe0nkjLuYAQAWgC2pZYgNLJR13iVbol0xHJoXQUHtgiaJ8\n'\
             'GLYFzjHQPqFMpSalQe3oELko39uOC1CoJCHFySECgYBeycUnRBikCO2n8DNhY4Eg\n'\
             '9Y3oxcssRt6ea5BZwgW2eAYi7/XqKkmxoSoOykUt3MJx9+EkkrL17bxFSpkj1tvL\n'\
             'qvxn7egtsKjjgGNAxwXC4MwCvhveyUQQxtQb8AqGrGqo4jEEN0L15cnP38i2x1Uo\n'\
             'muhfskWf4MABV0yTUaKcGg==\n'\
             '-----END PRIVATE KEY-----\n'


def get_my_ip():
    s1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s1.connect(('8.8.8.8', 80))
    my_ip = s1.getsockname()[0]
    s1.close()
    return my_ip


def start_https_server(ota_image_dir, server_ip, server_port, server_file=None, key_file=None):
    os.chdir(ota_image_dir)

    if server_file is None:
        server_file = os.path.join(ota_image_dir, 'server_cert.pem')
        cert_file_handle = open(server_file, 'w+')
        cert_file_handle.write(server_cert)
        cert_file_handle.close()

    if key_file is None:
        key_file = os.path.join(ota_image_dir, 'server_key.pem')
        key_file_handle = open('server_key.pem', 'w+')
        key_file_handle.write(server_key)
        key_file_handle.close()

    httpd = http.server.HTTPServer((server_ip, server_port), http.server.SimpleHTTPRequestHandler)

    if use_ssl == True:
        httpd.socket = ssl.wrap_socket(httpd.socket,
                                       keyfile=key_file,
                                       certfile=server_file, server_side=True)
    httpd.serve_forever()


if __name__ == '__main__':
    # Usage: example_test.py <server_port> <device>
    this_dir = os.path.dirname(os.path.realpath(__file__))
    bin_dir = os.path.join(this_dir, "./build")
    port = int(sys.argv[1])
    host_ip = get_my_ip()
    device = sys.argv[2]
    thread1 = Thread(target=start_https_server, args=(bin_dir, host_ip, port))
    thread1.daemon = True
    thread1.start()
    if use_ssl == True:
        ota_url = 'https://' + host_ip + ':' + str(port) + '/esp32cam.bin'
    else:
        ota_url = 'http://' + host_ip + ':' + str(port) + '/esp32cam.bin'
    client = mqtt.Client("esp32cam_ota")
    client.on_connect = on_connect
    client.on_message = on_message
    client.username_pw_set("mqtt", "mqtt")
    client.connect("homeassistant.localdomain")
    client.loop_start()
    print("Waiting to connect to MQTT...")
    while connected == False:
        time.sleep(1)
    # publish to the device the server URI to get it to start OTA
    print("Asking " + device + " to update from: " + ota_url)
    client.publish("devices/" + device + "/ota/url", ota_url)
    # wait for a good status from the device

    while status == "":
        print('.', end='', flush=True)
        time.sleep(1)
    print("\nOTA finished with return code: " + status)


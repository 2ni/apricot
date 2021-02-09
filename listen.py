"""
mosquitto_sub -h eu.thethings.network -t '+/devices/+/up' -u '<appid>' -P '<key>' -v
"""

import paho.mqtt.client as mqtt
import json
from dateutil.parser import parse
from datetime import timezone
import sys

try:
    from credentials import appid, key
except ModuleNotFoundError:
    print("missing credentials.py with definition of apid='' and key=''")
    sys.exit(1)


def on_connect(mqttc, mosq, obj, rc):
    # subscribe to specific device in a specific app
    mqttc.subscribe('+/devices/+/up')


def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))


def on_message(mqttc, obj, msg):

    try:
        x = json.loads(msg.payload.decode('utf-8'))
        timestamp = parse(x['metadata']['time']).replace(tzinfo=timezone.utc).astimezone(tz=None)
        timestamp_str = timestamp.strftime("%Y-%m-%d %H:%M:%S")
        alt_data = str("%s" % x['payload_fields'])
        print(timestamp_str, alt_data)

    except Exception as e:
        print(e)
        pass


def on_publis(mosq, obj, mid):
    print("mid: " + str(mid))


mqttc = mqtt.Client()

# Assign event callbacks
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.username_pw_set(appid, key)
mqttc.connect("eu.thethings.network", 1883, 60)

try:
    mqttc.loop_forever()
except KeyboardInterrupt:
    mqttc.disconnect()
    sys.exit(0)

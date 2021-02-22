"""
mosquitto_sub -h eu.thethings.network -t '+/devices/+/up' -u '<appid>' -P '<key>' -v
"""

import paho.mqtt.client as mqtt
import json
from dateutil.parser import parse
from datetime import timezone
import sys
import re

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
        x = json.loads(msg.payload.decode("utf-8"))
        devid = x["dev_id"]
        counter = x["counter"]
        gateways = x["metadata"]["gateways"]
        gws_with_pos = [(x['rssi'], x['latitude'], x['longitude'], x['time']) for x in gateways if 'latitude' in x]
        gws_with_pos_time = [(x['rssi'], x['latitude'], x['longitude'], x['time']) for x in gateways if 'latitude' in x and x['time']]
        timestamp = parse(x["metadata"]["time"]).replace(tzinfo=timezone.utc).astimezone(tz=None)
        timestamp_str = timestamp.strftime("%Y-%m-%d %H:%M:%S")
        data = str("%s" % x["payload_fields"])
        my_rssi = 0
        for gw in gateways:
            if gw["gtw_id"] == "eui-58a0cbfffe01bc78":
                my_rssi = gw["rssi"]

        print("{timestamp}: ({devid}:{counter},{datarate}|{frq}|{my_rssi}) {data} | gws ({no_gws_pos_time}/{no_gws_pos}/{no_gws}): {gws_with_pos}".format(
            timestamp=timestamp_str,
            devid=devid,
            counter=counter,
            datarate=re.findall("^(.*)BW", x["metadata"]["data_rate"])[0],
            frq=x["metadata"]["frequency"],
            data=data,
            no_gws=len(gateways),
            no_gws_pos=len(gws_with_pos),
            no_gws_pos_time=len(gws_with_pos_time),
            gws_with_pos=gws_with_pos,
            my_rssi=my_rssi
        ))
        sys.stdout.flush()

    except Exception as e:
        print(e)
        pass


def on_publish(mosq, obj, mid):
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

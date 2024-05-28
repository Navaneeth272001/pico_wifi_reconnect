import time
from umqtt.simple import MQTTClient
import network

from pimoroni_i2c import PimoroniI2C
from breakout_sgp30 import BreakoutSGP30
import rp2
import utime as time

# WLAN-Konfiguration
wlanSSID = 'xyr-69934'
wlanPW = '8bin-vqpr-gol0-7q3u'
rp2.country('CH')

# MQTT Configuration
mqttBroker = '192.168.1.134'
mqttClient = 'your_client_id'  # Set an appropriate client ID
mqttUser = ''
mqttPW = ''
mqttTopicAir = b"zurich/Air"
mqttTopiceCO2 = b"zurich/eCO2"
mqttTopicTVOC = b"zurich/TVOC"
mqttTopicqual = b"zurich/qual"
mqttTopicH2 = b"zurich/H2"
mqttTopicEthanol = b"zurich/Ethanol"

# Function: WLAN-Verbindung herstellen
def wlanConnect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('WLAN-Verbindung herstellen:', wlanSSID)
        wlan.connect(wlanSSID, wlanPW)
        for i in range(10):  # Try for 10 seconds
            if wlan.isconnected():
                break
            print('.', end='')
            time.sleep(1)
    if wlan.isconnected():
        print('\nWLAN-Verbindung hergestellt / WLAN-Status:', wlan.status())
    else:
        print('\nKeine WLAN-Verbindung / WLAN-Status:', wlan.status())

# Function: MQTT connection
def mqttConnect():
    client = MQTTClient(mqttClient, mqttBroker, user=mqttUser, password=mqttPW)
    client.connect()
    print('MQTT-Verbindung hergestellt')
    return client

PINS_PICO_EXPLORER = {"sda": 20, "scl": 21}  # Default i2c pins for Pico Explorer

i2c = PimoroniI2C(**PINS_PICO_EXPLORER)
sgp30 = BreakoutSGP30(i2c)

print("SGP30 initialised - about to start measuring without waiting")
sgp30.start_measurement(False)
id = sgp30.get_unique_id()
print("Started measuring for id 0x", '{:04x}'.format(id[0]), '{:04x}'.format(id[1]), '{:04x}'.format(id[2]), sep="")

j = 0
data = []

wlanConnect()

while True:
    if wlan.isconnected():
        j += 1
        air_quality = sgp30.get_air_quality()
        eCO2 = air_quality[BreakoutSGP30.ECO2]
        TVOC = air_quality[BreakoutSGP30.TVOC]
        air_quality_raw = sgp30.get_air_quality_raw()
        H2 = air_quality_raw[BreakoutSGP30.H2]
        ETHANOL = air_quality_raw[BreakoutSGP30.ETHANOL]

        print(f"{j}: CO2 {eCO2}, TVOC {TVOC}, raw {H2} {ETHANOL}")

        myValueAir = str(air_quality)
        myValueeCO2 = str(eCO2)
        myValueTVOC = str(TVOC)
        myValuequal = str(air_quality_raw)
        myValueH2 = str(H2)
        myValueEthanol = str(ETHANOL)

        try:
            client = mqttConnect()
            client.publish(mqttTopicAir, myValueAir)
            client.publish(mqttTopiceCO2, myValueeCO2)
            client.publish(mqttTopicTVOC, myValueTVOC)
            client.publish(mqttTopicqual, myValuequal)
            client.publish(mqttTopicH2, myValueH2)
            client.publish(mqttTopicEthanol, myValueEthanol)
            print(f"An Topic {mqttTopicAir} gesendet: {myValueAir}")
            client.disconnect()
            print('MQTT-Verbindung beendet')
        except Exception as e:
            print(f"MQTT-Verbindung fehlgeschlagen: {e}")

        time.sleep(10.0)
    else:
        print("Verbindung verloren. Versuche, erneut zu verbinden...")
        wlanConnect()
        time.sleep(5)
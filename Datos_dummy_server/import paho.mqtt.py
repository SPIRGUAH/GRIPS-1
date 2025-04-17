import paho.mqtt.client as mqtt
import time
import random

# Configuración del broker MQTT
# broker = "srs-copper.duckdns.org" # broker chan
# broker = "192.168.199.105" # broker S2P
broker = "192.168.1.141" # Broker wifi casa isma

port = 1883

def generate_random_value(min_val, max_val):
    return round(random.uniform(min_val, max_val), 6)

def generate_random_integer(min_val, max_val):
    return random.randint(min_val, max_val)

# Conexión al broker
client = mqtt.Client()
client.connect(broker, port, 60)

# Publicación periódica de datos
try:
    while True:
        altitude = generate_random_value(0, 600000)
        rssi3 = generate_random_integer(-120, -30)
        rssi2 = generate_random_integer(-120, -30)
        rssi1 = generate_random_integer(-120, -30)
        hdop = generate_random_value(0.5, 5.0)
        ext_temperature_ours = generate_random_value(-127, 79)
        temperature = generate_random_value(-79, 79)
        pressure = generate_random_value(1, 1099)
        humidity = generate_random_value(1, 99)
        baro_altitude = generate_random_value(0, 600000)
        cp10sec = generate_random_integer(0, 1000)
        latitude = round(random.uniform(-90, 90), 5)
        longitude = round(random.uniform(-180, 180), 5)
        
        # Publicar datos en los topics correspondientes
        client.publish("globo/altitude", str(altitude))
        client.publish("globo/RSSI3", str(rssi3))
        client.publish("globo/RSSI2", str(rssi2))
        client.publish("globo/RSSI1", str(rssi1))
        client.publish("globo/hdop", str(hdop))
        client.publish("globo/ext_temperature", str(ext_temperature_ours))
        client.publish("globo/temperature", str(temperature))
        client.publish("globo/pressure", str(pressure))
        client.publish("globo/humidity", str(humidity))
        client.publish("globo/baro_altitude", str(baro_altitude))
        client.publish("globo/CP10Sec", str(cp10sec))
        client.publish("globo/latitude", str(latitude))
        client.publish("globo/longitude", str(longitude))
        
        print(f"Datos publicados:")
        print(f"Altitude: {altitude} m, RSSI1: {rssi1} dBm, RSSI2: {rssi2} dBm, RSSI3: {rssi3} dBm, HDOP: {hdop}, Ext Temp: {ext_temperature_ours} C, Temp: {temperature} C, Pressure: {pressure} hPa, Humidity: {humidity} %, Baro Altitude: {baro_altitude} m, CP10Sec: {cp10sec}, Latitude: {latitude}, Longitude: {longitude}")
        
        time.sleep(10)  # Esperar 10 segundos

except KeyboardInterrupt:
    print("Publicación detenida.")
    client.disconnect()

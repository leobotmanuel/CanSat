import serial
from time import sleep, time
from picamera import PiCamera
import random

camara = PiCamera()
a = 0
x = ""

ser = serial.Serial(
	port = '/dev/ttyAMA0',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 1
)

with open("/home/pi/datos_cansat_argonautex.csv", "a") as log:
	log.write("palabra_clave,temperatura_bme,presion_bme,humedad_bme,altitud_bme,duv,latitud,longitud,velocidad,altitud_gps,co2,gases_volatiles,acx,acy,acz,girox,giroy,giroz,magx,magy,magz,IR")
	log.close()

while True:
	x = ser.readline()
	a = random.randint(0,700)
	camara.capture("/home/pi/fotos/imagen" + str(a) + ".jpg")
        with open("/home/pi/datos_cansat_argonautex.csv", "a") as log:
            log.write(x + "\n")
            log.close()
        sleep(3)

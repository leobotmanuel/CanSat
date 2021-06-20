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
	log.write("clave,temperaturaBME,presionBME,humedadBME,altitudBME,DUV,CO2,GasesVolatiles,AcelerometroX,AcelerometroY,AcelerometroZ,GiroscopioX,GiroscopioY,GiroscopioZ,MagnetometroX,MagnetometroY,MagnetometroZ,presAlt, tempAlt, altAlt,bateria,IR,LatitudGPS,LongitudGPS,velocidadGPS,altitudGPS")
	log.close()

while True:
	x = ser.readline()
	a = random.randint(0,700)
	camara.capture("/home/pi/fotos/imagen" + str(a) + ".jpg")
        with open("/home/pi/datos_cansat_argonautex.csv", "a") as log:
            log.write(x + "\n")
            log.close()
        sleep(3)

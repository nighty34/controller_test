from controller import RCReceiver
from constants import config

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from board import SCL, SDA

import busio
import time

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

receiver = RCReceiver(config)

servo7 = servo.Servo(pca.channels[0])

for i in range(180):
    servo7.angle = i
    time.sleep(0.03)
for i in range(180):
    servo7.angle = 180 - i
    time.sleep(0.03)

# You can also specify the movement fractionally.
fraction = 0.0
while fraction < 1.0:
    servo7.fraction = fraction
    fraction += 0.01
    time.sleep(0.03)

pca.deinit()

for i in range (1, 30):
    result = receiver.run()
    print(f"Steering: {result[0]}, Throttle: {result[1]}")

    time.sleep(0.3)

receiver.shutdown()
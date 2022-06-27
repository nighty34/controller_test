from controller import RCReceiver
from constants import config
import time

receiver = RCReceiver(config)

for i in range (1, 30):
    result = receiver.run()
    print(f"Steering: {result[0]}, Throttle: {result[1]}")
    time.sleep(0.3)

receiver.shutdown()
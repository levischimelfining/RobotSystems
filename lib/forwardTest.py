from picarx import Picarx
from utils import reset_mcu
import time
reset_mcu()

if __name__ == "__main__":
    px = Picarx()
    px.set_dir_servo_angle(-1)
    time.sleep(0.01)
    px.forward(50)
    time.sleep(1)
    px.stop()




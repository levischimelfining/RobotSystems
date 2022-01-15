from picarx_improved import Picarx
from utils import reset_mcu
import time
reset_mcu()

if __name__ == "__main__":
    px = Picarx()
    px.forward(50)
    time.sleep(1)
    px.stop()




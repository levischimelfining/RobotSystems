import sys
from numpy import sign
sys.path.append(r'/home/levi/picar-x/lib')
from utils import reset_mcu
reset_mcu()
from adc import ADC
from picarx_improved import Picarx
import time


class Sensing:
    def __init__(self):
        self.chn_0 = ADC("A0")
        self.chn_1 = ADC("A1")
        self.chn_2 = ADC("A2")

    def sensor_reading(self):
        adc_value_list = []
        adc_value_list.append(self.chn_0.read())
        adc_value_list.append(self.chn_1.read())
        adc_value_list.append(self.chn_2.read())
        return adc_value_list


class Interpretation:
    def __init__(self):
        self.sensitivity = int(
            input("Enter sensitivity for range of variation in color of line and surroundings [10-50]: ") or "20")
        self.polarity = str(
            input("Enter polarity for the line compared to surroundings [darker or lighter]: ") or "lighter")

        if self.polarity == "lighter":
            self.polarity = 1
        elif self.polarity == "darker":
            self.polarity = -1

    def interpret(self, adc_list):

        if self.polarity != 1 or self.polarity != -1:
            return RuntimeError("Invalid polarity")

        if abs(adc_list[2]-adc_list[0]) > self.sensitivity:
            output = self.polarity * sign(adc_list[2] - adc_list[0])*(self.sensitivity + min(adc_list))/max(adc_list)
            if output > 1:
                output = 1
            elif output < -1:
                output = -1
        else:
            output = 0

        return output


class Controller:

    def __init__(self):
        self.scaling_factor = int(input("Enter the scaling factor for the angle [1-35]" or "35"))

    def control(self, output):
        px = Picarx()
        angle = self.scaling_factor * output
        px.set_dir_servo_angle(angle)
        return angle


if __name__ == "__main__":

    while True:
        sensor = Sensing()
        sensor_output = sensor.sensor_reading()
        interpreter = Interpretation()
        interpreter_output = interpreter.interpret(adc_list=sensor_output)
        controller_output = Controller()
        controller_output.control(output=interpreter_output)
        time.sleep(0.5)

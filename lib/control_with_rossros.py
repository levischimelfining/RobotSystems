from numpy import sign
from adc import ADC
from picarx_improved import Picarx
from utils import reset_mcu
from ultrasonic import Ultrasonic
from pin import Pin

import concurrent.futures
import time
import sys
import rossros

sys.path.append(r'/home/levi/picar-x/lib')
reset_mcu()


class GrayscaleSensorBus(rossros.Bus):
    pass


class UltrasonicSensorBus(rossros.Bus):
    pass


class GrayscaleInterpreterBus(rossros.Bus):
    pass


class UltrasonicInterpreterBus(rossros.Bus):
    pass


def grayscale_sensor_function():
    while True:
        chn_0 = ADC("A0")
        chn_1 = ADC("A1")
        chn_2 = ADC("A2")
        adc_value_list = [chn_0.read(), chn_1.read(), chn_2.read()]
        return adc_value_list


def ultrasonic_sensor_function():
    trig_pin = Pin("D2")
    echo_pin = Pin("D3")
    sonar = Ultrasonic(trig_pin, echo_pin)
    return sonar.read()


def grayscale_interpreter_function(sensor_values):
    sensitivity = 10
    polarity = 1

    if abs(sensor_values[2] - sensor_values[0]) > sensitivity:
        interpreter_value = -polarity * sign(sensor_values[2] - sensor_values[0]) * (
                    sensitivity + min(sensor_values)) / max(
            sensor_values)
        if interpreter_value > 1:
            interpreter_value = 1
        elif interpreter_value < -1:
            interpreter_value = -1
    else:
        interpreter_value = 0
    return interpreter_value


def ultrasonic_interpreter_function(distance):
    if 0 < distance < 300:
        return True
    else:
        return False


def controller_function(interpreter_values):
    scaling_factor = 35
    px.forward(30)
    while True:
        if interpreter_values[1]:
            angle = scaling_factor * interpreter_values[0]
            px.set_dir_servo_angle(angle)
        else:
            px.stop()


if __name__ == "__main__":

    px = Picarx()
    sensor_delay = 0.01
    grayscale_sensor_values_bus = GrayscaleSensorBus()
    ultrasonic_sensor_values_bus = UltrasonicSensorBus()
    interpreter_delay = 0.05
    grayscale_interpreter_bus = GrayscaleInterpreterBus()
    ultrasonic_interpreter_bus = UltrasonicInterpreterBus()
    controller_delay = 0.1
    producer_consumer_list = [rossros.Producer(grayscale_sensor_function, grayscale_sensor_values_bus, 0.01),
                              rossros.Producer(ultrasonic_sensor_function, ultrasonic_sensor_values_bus, 0.01),
                              rossros.ConsumerProducer(grayscale_interpreter_function, grayscale_sensor_values_bus,
                                                       grayscale_interpreter_bus, 0.05),
                              rossros.ConsumerProducer(ultrasonic_interpreter_function, ultrasonic_sensor_values_bus,
                                                       ultrasonic_interpreter_bus, 0.05),
                              rossros.Consumer(controller_function,
                                               (grayscale_interpreter_bus, ultrasonic_interpreter_bus), 0.1)]

    rossros.runConcurrently(producer_consumer_list)

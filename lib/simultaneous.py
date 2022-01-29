from numpy import sign
from readerwriterlock import rwlock
from adc import ADC
from picarx_improved import Picarx
import concurrent.futures
import time
import sys
sys.path.append(r'/home/levi/picar-x/lib')
from utils import reset_mcu
reset_mcu()


# Create basic structure of message bus
class MessageBus:

    def __init__(self):
        self.message = []
        self.lock = rwlock.RWLockWriteD()

    def write(self, message):
        with self.lock.gen_wlock():
            self.message = message

    def read(self):
        with self.lock.gen_rlock():
            message = self.message
        return message


# Create sensor function that writes the ADC values to the sensor_value_bus
def sensor_function(sensor_values_bus, sensor_delay):
    while True:
        chn_0 = ADC("A0")
        chn_1 = ADC("A1")
        chn_2 = ADC("A2")
        adc_value_list = [chn_0.read(), chn_1.read(), chn_2.read()]
        sensor_values_bus.write(message=adc_value_list)
        time.sleep(sensor_delay)


# Create interpreter function that writes output from [-1,1] to the interpreter_bus based on read values from sensor bus
def interpreter_function(sensor_values_bus, interpreter_bus, interpreter_delay):
    sensitivity = 10
    polarity = 1

    while True:
        adc_list = sensor_values_bus.read
        if abs(adc_list[2]-adc_list[0]) > sensitivity:
            output = -polarity * sign(adc_list[2] - adc_list[0])*(sensitivity + min(adc_list))/max(adc_list)
            if output > 1:
                output = 1
            elif output < -1:
                output = -1
        else:
            output = 0

        interpreter_bus.write(message=output)
        time.sleep(interpreter_delay)


# Create controller function that maps read values from interpreter bus to an actual steering angle
def controller_function(interpreter_bus, controller_delay):
    px = Picarx()
    px.forward(30)
    scaling_factor = 35
    interpreter_output = interpreter_bus.read
    angle = scaling_factor * interpreter_output
    px.set_dir_servo_angle(angle)
    time.sleep(controller_delay)


if __name__ == "__main__":

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:

        sensor_delay = 0.01
        sensor_values_bus = MessageBus()
        interpreter_delay = 0.05
        interpreter_bus = MessageBus()
        controller_delay = 0.1
        controller_bus = MessageBus()

        eSensor = executor.submit(sensor_function, sensor_values_bus, sensor_delay)
        eInterpreter = executor.submit(interpreter_function, sensor_values_bus, interpreter_bus, interpreter_delay)
        eController = executor.submit(controller_function, interpreter_bus, controller_delay)

    eSensor.result()

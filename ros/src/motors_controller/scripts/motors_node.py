#!/usr/bin/env python
from functools import partial

import rospy
from motors_controller.srv import SetSpeed, SetSpeedResponse


def node_main():
    rospy.init_node('motors_controller', anonymous=True)
    # params = rospy.get_param('~')
    gpio_interface = GpioInterface()
    set_speed_service = rospy.Service('set_speed', SetSpeed, partial(set_speed, gpio_interface))
    rospy.spin()
    set_speed_service.shutdown()


def set_speed(gpio_interface, request):
    if request.stop:
        gpio_interface.stop()
    else:
        if request.left.speed_percent > 0:
            gpio_interface.enable1.start(request.left.speed_percent)
            if request.left.direction_forward:
                gpio_interface.input1.set()
                gpio_interface.input2.unset()
            else:
                gpio_interface.input1.unset()
                gpio_interface.input2.set()
        if request.right.speed_percent > 0:
            gpio_interface.enable2.start(request.right.speed_percent)
            if request.right.direction_forward:
                gpio_interface.input3.set()
                gpio_interface.input4.unset()
            else:
                gpio_interface.input3.unset()
                gpio_interface.input4.set()
        gpio_interface.stop_in(request.duration_ms)
    return SetSpeedResponse(result=True)


class FakeGpio(object):

    class FakeObject(object):

        def __init__(self, name):
            self.name = name

        def __call__(self, *args):
            call_name = '%s(%s)' % (self.name, ','.join(map(str, args)))
            rospy.loginfo('Fake call: ' + call_name)
            return FakeGpio.FakeObject(call_name)

        def __getattr__(self, attr):
            attr_name = self.name + '.' + attr
            return FakeGpio.FakeObject(attr_name)

        def __str__(self):
            return self.name

    def __getattr__(self, attr):
        return FakeGpio.FakeObject(attr)


try:
    import RPi.GPIO as gpio
except ImportError:
    rospy.logwarn('GPOI module not found. Faking it')
    gpio = FakeGpio()


class GpioInterface(object):

    def __init__(self):
        gpio.setmode(gpio.BOARD)
        self.timer = None
        self.enable1 = PwmPin(19)
        self.input1 = BinPin(21)
        self.input2 = BinPin(23)
        self.enable2 = PwmPin(22)
        self.input3 = BinPin(24)
        self.input4 = BinPin(26)

    def stop(self, event=None):
        if self.timer is not None:
            self.timer.shutdown()
        self.timer = None
        self.enable1.stop()
        self.input1.unset()
        self.input2.unset()
        self.enable2.stop()
        self.input3.unset()
        self.input4.unset()

    def stop_in(self, duration_ms):
        if self.timer is not None:
            self.timer.shutdown()
        self.timer = rospy.Timer(rospy.Duration.from_sec(duration_ms / 1000), self.stop, oneshot=True)


class PwmPin(object):

    def __init__(self, pin, frequency=5):
        gpio.setup(pin, gpio.OUT)
        self.pin = pin
        self.frequency = frequency
        self.pwm = gpio.PWM(pin, frequency)
        self.pwm.stop()

    def start(self, percent):
        rospy.loginfo('PIN %s start PWM at %s%% (freq %sHz)', self.pin, percent, self.frequency)
        self.pwm.start(percent)

    def stop(self):
        rospy.loginfo('PIN %s stop PWM', self.pin)
        self.pwm.stop()


class BinPin(object):

    def __init__(self, pin):
        self.pin = pin
        gpio.setup(pin, gpio.OUT)
        gpio.output(self.pin, 0)

    def set(self):
        rospy.loginfo('PIN %s set to HIGH', self.pin)
        gpio.output(self.pin, 1)

    def unset(self):
        rospy.loginfo('PIN %s set to LOW', self.pin)
        gpio.output(self.pin, 0)


if __name__ == '__main__':
    try:
        node_main()
    except rospy.ROSInterruptException:
        pass

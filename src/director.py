import machine
from machine import Pin, PWM, Timer, ADC
from esp32 import RMT
import math
import time
import json
import uasyncio as asyncio

# import tinypico as TinyPICO
# from dotstar import DotStar
# from machine import SPI

STOP = 'stop'
FWD = 'fwd'
BWD = 'bwd'
MAX = 'max'
MIN = 'min'
LAST = 'last'
RANGE = 'range'
SPEED_1 = 'slowest'
SPEED_2 = 'slow'
SPEED_3 = 'fast'
SPEED_4 = 'fastest'
MAIN = 'main'
STAND = 'stand'
PIVOT = 'pivot'
CROUCH = 'crouch'
INIT = 'init'
RMT_CH_TURNER = 0
RMT_CH_WALKER = 1
RMT_CLOCK_DIV = 80
RMT_CYCLE = 20000

#### HUZZAH32 ####
# PIN_POWER = 13
# PIN_SERVO_REAR = 4
# PIN_SERVO_FRONT = 14
# PIN_HALL_LEFT = 15
# PIN_HALL_FRONT = 33
# PIN_HALL_RIGHT = 12
# PIN_HALL_REAR = 27

#### TINYPICO ####
PIN_POWER = 4
PIN_SERVO_REAR = 33
PIN_SERVO_FRONT = 25
PIN_HALL_LEFT = 26
PIN_HALL_FRONT = 27
PIN_HALL_RIGHT = 14
PIN_HALL_REAR = 15


class Director:

    ######################  HALL  ###################################################

    class HallSensor:
        def __init__(self):
            self.left = Pin(PIN_HALL_LEFT, Pin.IN, pull=Pin.PULL_UP)
            self.front = Pin(PIN_HALL_FRONT, Pin.IN, pull=Pin.PULL_UP)
            self.right = Pin(PIN_HALL_RIGHT, Pin.IN, pull=Pin.PULL_UP)
            self.rear = Pin(PIN_HALL_REAR, Pin.IN, pull=Pin.PULL_UP)

            self.pins = (self.left, self.front,
                         self.right, self.rear)

        def get_pose_fwd(self, pin):
            if pin == self.left:
                return STAND
            elif pin == self.front:
                return PIVOT
            elif pin == self.right:
                return CROUCH
            elif pin == self.rear:
                return MAIN

        def get_pose_bwd(self, pin):
            if pin == self.left:
                return PIVOT
            elif pin == self.front:
                return CROUCH
            elif pin == self.right:
                return MAIN
            elif pin == self.rear:
                return STAND

        def get_next_pose_fwd(self, pose):
            if pose is STAND:
                return MAIN
            elif pose is MAIN:
                return CROUCH
            elif pose is CROUCH:
                return PIVOT
            elif pose is PIVOT:
                return STAND

        def get_next_pose_bwd(self, pose):
            if pose is STAND:
                return PIVOT
            elif pose is PIVOT:
                return CROUCH
            elif pose is CROUCH:
                return MAIN
            elif pose is MAIN:
                return STAND

    ######################  WALKER  ###################################################

    class Walker:
        def __init__(self, hall_sensor, turner):
            self.FREQ = 50

            self.DUTY_FWD_MAX = 1950
            self.DUTY_FWD_MIN = 1650
            self.DUTY_IDLE = 1450
            self.DUTY_BWD_MIN = 1320
            self.DUTY_BWD_MAX = 1020

            self.hall_sensor = hall_sensor

            self.servo = RMT(RMT_CH_WALKER, pin=Pin(
                PIN_SERVO_REAR), clock_div=80)

            self.speed = 0
            self.direction = None
            self.active = True

            self.current_duty = self.DUTY_IDLE

        def set_speed(self, speed):
            self.speed = speed
            self._start()

            # print("SET SPEED: "+str(self.speed))

        def start_fwd(self, timing=None):
            # if self.active:
            #     try:
            #         self.servo.init()
            #     except Exception as e:
            #         print(e)
            print("START_FWD")
            self.direction = FWD
            self._start()

        def start_bwd(self, timing=None):
            # if self.active:
            #     try:
            #         self.servo.init()
            #     except Exception as e:
            #         print(e)
            print("START_BWD")
            self.direction = BWD
            self._start()

        def _start(self):
            print("DIRECTION: "+str(self.direction))

            if self.direction == FWD:
                target_duty = (self.DUTY_FWD_MAX - self.DUTY_FWD_MIN) * \
                    self.speed + self.DUTY_FWD_MIN
            elif self.direction == BWD:
                target_duty = self.DUTY_BWD_MIN - \
                    (self.DUTY_BWD_MIN - self.DUTY_BWD_MAX) * self.speed

            target_duty = int(target_duty)

            print("CURRENT: "+str(self.current_duty))
            print("TARGET: "+str(target_duty))

            resolution = 24
            increment = abs(self.current_duty - target_duty) // resolution
            acc_table = []
            for i in range(resolution):
                if self.current_duty < target_duty:
                    pulse = self.current_duty + increment * i
                else:
                    pulse = self.current_duty - increment * i
                cycle = RMT_CYCLE - pulse
                acc_table.append(pulse)
                acc_table.append(cycle)

            print(acc_table)

            self.servo.loop(0)
            self.servo.write_pulses((0, RMT_CYCLE))
            # time.sleep(0.5)
            print("START WINDUP")
            self.servo.write_pulses((acc_table))

            while(not self.servo.wait_done(timeout=3000)):
                print("WAIT_DONE")
                time.sleep_ms(50)

            # time.sleep(2)

            print("WINDUP DONE")
            self.current_duty = target_duty
            self.resume()

        def resume(self):
            self.servo.loop(1)
            self.servo.write_pulses(
                (self.current_duty, RMT_CYCLE - self.current_duty))

        def stop(self, timing=None):
            # if not self.active:
            #     try:
            #         self.servo.deinit()
            #     except Exception as e:
            #         print(e)
            #     self.active = True
            self.servo.loop(0)
            self.current_duty = self.DUTY_IDLE

    ######################  TURNER  ###################################################
    class Turner:

        def __init__(self):
            self.FREQ = 50

            self.duty_center = 900  #
            self.duty_cw = 400  # 69
            self.duty_ccw = -400  # 23

            self.resolution = 24
            self.full_duty_range = abs(self.duty_cw - self.duty_ccw)

            self.servo = RMT(RMT_CH_TURNER, pin=Pin(
                PIN_SERVO_FRONT), clock_div=RMT_CLOCK_DIV)

            self.extent = 1
            self.current_duty = self.duty_center
            self.target_duty = None

            self.z = True
            # while(True):
            self.ccw()
            time.sleep(0.3)
            self.cw()
            time.sleep(0.5)
            self.center()
            time.sleep(0.5)

        def set_extent(self, extent):
            self.extent = extent

        def center(self):
            self.target_duty = self.duty_center
            self._turn()

        def cw(self, timer=None):
            self.target_duty = math.ceil(
                self.duty_center + self.duty_cw * self.extent)
            self._turn()

        def ccw(self, timer=None):
            self.target_duty = math.floor(
                self.duty_center + self.duty_ccw * self.extent)
            self._turn()

        def _turn(self):
            turn_range = abs(self.current_duty - self.target_duty)
            nr_cycles = self.resolution * turn_range // self.full_duty_range

            # print("CURRENT: "+str(self.current_duty))
            # print("TARGET: "+str(self.target_duty))
            # print("CYCLES: "+str(nr_cycles))
            # print("RANGE: "+str(turn_range))

            turn_table = []
            for i in range(nr_cycles):
                x = 1 / nr_cycles * i
                a = x ** 2 * (3 - 2 * x)
                pulse_offset = math.ceil(turn_range * a)
                if self.current_duty < self.target_duty:
                    pulse = self.current_duty + pulse_offset
                else:
                    pulse = self.current_duty - pulse_offset

                turn_table.append(pulse)
                turn_table.append(RMT_CYCLE - pulse)

            # print(turn_table)

            if turn_table:
                self.servo.write_pulses(turn_table)

            while(not self.servo.wait_done(timeout=1000)):
                time.sleep_ms(100)

            self.current_duty = self.target_duty
            # print("DONE")

            if self.z:
                print("Z!")
                # for i in range(turn_range):
                #     if start_duty < self.target_duty:
                #         self.current_duty = start_duty + turn_table[i]

                #     elif start_duty > self.target_duty:
                #         self.current_duty = start_duty - turn_table[i]

                #     self.servo.duty(self.current_duty)
                #     time.sleep_ms(10)

            else:
                pass
                # print("not")
                # while(True):
                #     if self.current_duty == self.target_duty:
                #         return

                #     elif self.current_duty < self.target_duty:
                #         self.current_duty = self.current_duty + 1

                #     elif self.current_duty > self.target_duty:
                #         self.current_duty = self.current_duty - 1

                #     self.servo.duty(self.current_duty)
                #     time.sleep_ms(7)

                ###########################  INIT ######################################################

    def __init__(self):
        # Configure SPI for controlling the DotStar
        # Internally we are using software SPI for this as the pins being used are not hardware SPI pins

        # spi = SPI(sck=Pin(TinyPICO.DOTSTAR_CLK), mosi=Pin(
        #     TinyPICO.DOTSTAR_DATA), miso=Pin(TinyPICO.SPI_MISO))

        # Create a DotStar instance
        # Just one DotStar, half brightness

        # dotstar = DotStar(spi, 1, brightness=0.05)

        # Turn on the power to the DotStar

        # TinyPICO.set_dotstar_power(True)

        # Create a colour wheel index int

        # color_index = 0
        # dotstar[0] = (0, 255, 0, 0.3)

        self.pin_power = Pin(PIN_POWER, Pin.OUT)
        self.pin_power.value(0)

        self.pin_bat = Pin
        self.adc_bat = ADC(Pin(35))
        self.adc_bat.atten(ADC.ATTN_11DB)

        self.hall_sensor = self.HallSensor()
        self.turner = self.Turner()
        self.walker = self.Walker(self.hall_sensor, self.turner)

        self.pin_power.value(1)

        self.idle = True

        self.sleep_timer = Timer(0)
        self.sleep_count = 0
        self.sleep_after_seconds = 60 * 5

        self.start_sleep_countdown()

        for pin in self.hall_sensor.pins:
            pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
                    handler=self.irq_hall)

        self.current_command = None

        self.pose = None

        self.script = None

        self.script_wake = {
            INIT: (self.turner.center, self.walker.start_fwd, ),
            MAIN: (self.turner.center, ),
            STAND: (self.walker.stop, ),
            PIVOT: (self.turner.center, ),
            CROUCH: (self.turner.center, ),
        }

        self.script_sleep = {
            INIT: (self.turner.center, self.walker.start_bwd, ),
            MAIN: (self.turner.center, ),
            STAND: (self.turner.center, ),
            PIVOT: (self.turner.center, ),
            CROUCH: (self.walker.stop, ),
        }

        self.script_stop = {
            "name": "script_stop",
            INIT: (self.turner.center, ),
            MAIN: None,
            STAND: (self.walker.stop, self.power_off),
            PIVOT: None,
            CROUCH: (self.walker.stop, self.power_off),
        }

        self.script_fwd = {
            "name": "script_fwd",
            INIT: (self.turner.center, self.walker.start_fwd, ),
            MAIN: None,
            STAND: None,
            PIVOT: None,
            CROUCH: None
        }

        self.script_bwd = {
            "name": "script_bwd",
            INIT: (self.turner.center, self.walker.start_bwd, ),
            MAIN: None,
            STAND: None,
            PIVOT: None,
            CROUCH: None
        }

        self.script_pivot_cw = {
            INIT: (self.turner.center, self.walker.start_fwd, ),
            MAIN: (self.walker.stop, self.turner.ccw, self.walker.start_bwd, ),
            STAND: None,
            PIVOT: (self.walker.stop, self.turner.cw, self.walker.start_fwd, ),
            CROUCH: None,
        }

        self.script_pivot_ccw = {
            INIT: (self.turner.center, self.walker.start_fwd, ),
            MAIN: (self.walker.stop, self.turner.cw, self.walker.start_bwd, ),
            STAND: None,
            PIVOT: (self.walker.stop, self.turner.ccw, self.walker.start_fwd, ),
            CROUCH: None,
        }

        self.script_turn_cw_fwd = {
            INIT: (self.turner.center, self.walker.start_fwd, ),
            MAIN: (self.walker.stop, self.turner.ccw, self.walker.start_fwd, ),
            STAND: None,
            PIVOT: (self.walker.stop, self.turner.cw, self.walker.start_fwd, ),
            CROUCH: None
        }

        self.script_turn_ccw_fwd = {
            INIT: (self.turner.center, self.walker.start_fwd, ),
            MAIN: (self.walker.stop, self.turner.cw, self.walker.start_fwd, ),
            STAND: None,
            PIVOT: (self.walker.stop, self.turner.ccw, self.walker.start_fwd, ),
            CROUCH: None
        }

        self.script_turn_cw_bwd = {
            INIT: (self.turner.center, self.walker.start_bwd, ),
            MAIN: (self.turner.ccw, ),
            STAND: None,
            PIVOT: (self.turner.cw, ),
            CROUCH: None
        }

        self.script_turn_ccw_bwd = {
            INIT: (self.turner.center, self.walker.start_bwd, ),
            MAIN: (self.turner.cw, ),
            STAND: None,
            PIVOT: (self.turner.ccw, ),
            CROUCH: None
        }

        # self.walker.stop()

    def power_on(self):
        self.pin_power.value(0)

    def power_off(self):
        self.pin_power.value(1)

    async def main_loop(self):
        script = None
        pose = None
        sequence = None

        while(True):

            if script is not self.script:
                script = self.script
                if script:
                    sequence = script[INIT]
                    # print("NEW SCRIPT: "+str(script["name"]))

            elif pose is not self.pose:
                pose = self.pose
                if script and pose:
                    sequence = script[pose]

            if sequence:
                for action in sequence:
                    print(action)
                    action()
                sequence = None

            await asyncio.sleep(0)

    def start_sleep_countdown(self):
        self.sleep_timer.init(mode=Timer.PERIODIC,
                              period=1000, callback=self.sleep_countdown)

    def stop_sleep_countdown(self):
        self.sleep_timer.deinit()
        self.sleep_count = 0

    def sleep_countdown(self, tim):
        if not self.idle:
            self.sleep_count = 0

        elif self.sleep_count < self.sleep_after_seconds:
            self.sleep_count = self.sleep_count + 1
        else:
            print("GOING TO DEEPSLEEP...")
            self.script = self.script_sleep
            time.sleep(3)
            machine.deepsleep()

    def irq_hall(self, pin):
        # print("DIRECTION: "+str(self.walker.direction))
        if pin.value():
            self.irq_hall_rising(pin)
        else:
            self.irq_hall_falling(pin)

    def irq_hall_falling(self, pin):
        if self.walker.direction is not FWD:
            return

        # if not self.pose:
        #     self.pose = self.hall_sensor.get_pose_fwd(pin)
        #     return

        # detected_pose = self.hall_sensor.get_pose_fwd(pin)
        # if detected_pose is self.hall_sensor.get_next_pose_fwd(self.pose):
        #     self.pose = detected_pose

        self.pose = self.hall_sensor.get_pose_fwd(pin)
        # print("FWD POSE: "+str(self.pose))

    def irq_hall_rising(self, pin):
        if self.walker.direction is not BWD:
            return

        # if not self.pose:
        #     self.pose = self.hall_sensor.get_pose_bwd(pin)
        #     return

        # detected_pose = self.hall_sensor.get_pose_bwd(pin)
        # if detected_pose is self.hall_sensor.get_next_pose_bwd(self.pose):
        #     self.pose = detected_pose

        self.pose = self.hall_sensor.get_pose_bwd(pin)
        # print("BWD POSE: "+str(self.pose))

    def recieve_command(self, c):

        self.current_command = c

        s = c['speed_ratio']
        t = c['turn_ratio']

        self.turner.set_extent(t)

        last_script = self.script

        if c['idle']:
            self.idle = True
            self.start_sleep_countdown()

            self.script = self.script_stop
            self.walker.speed = s

        else:
            self.idle = False
            self.stop_sleep_countdown()
            self.power_on()

            if c['direction'] == 'forward':
                self.script = self.script_fwd

            elif c['direction'] == 'backward':
                self.script = self.script_bwd

            elif c['direction'] == 'forward-left':
                self.script = self.script_turn_ccw_fwd

            elif c['direction'] == 'forward-right':
                self.script = self.script_turn_cw_fwd

            elif c['direction'] == 'backward-right':
                self.script = self.script_turn_ccw_bwd

            elif c['direction'] == 'backward-left':
                self.script = self.script_turn_cw_bwd

            elif c['direction'] == 'pivot-left':
                self.script = self.script_pivot_ccw

            elif c['direction'] == 'pivot-right':
                self.script = self.script_pivot_cw

            if last_script is self.script and s != self.walker.speed:
                self.walker.set_speed(s)

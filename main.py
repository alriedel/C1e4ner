from ev3dev.ev3 import GyroSensor
from ev3dev.ev3 import TouchSensor
from ev3dev.ev3 import LargeMotor
import threading
from time import sleep

class MotorThread(threading.Thread):
    """ Simple thread dealing with driving motors """
    def __init__(self, left_motor_out, right_motor_out):
        self.left_motor  = LargeMotor(left_motor_out)
        self.right_motor = LargeMotor(right_motor_out)
        self.running     = True
        self.speed_left  = 0
        self.speed_right = 0
        threading.Thread.__init__(self)

    def run(self):
        print ("Motor thread running!")
        while self.running:
            self.left_motor.run_direct(duty_cycle_sp=self.speed_left)
            self.right_motor.run_direct(duty_cycle_sp=self.speed_right)

    def stop(self):
        print ("Motor thread stopping!")
        self.running = False
        self.left_motor.stop()
        self.right_motor.stop()

    def set_speed(self, speed):
        self.speed_left  = speed[0]
        self.speed_right = speed[1]

class CalibrationError(Exception):
    def __init__(self, msg, value = None):
        self.msg   = msg
        self.value = value

class SensorWatcher(threading.Thread):
    def __init__(self, event_receiver, watcher_fun, watch_frequency = 0.2):
        self.event_receiver  = event_receiver
        self.watch_frequency = watch_frequency
        self.running         = True
        self.watcher_fun     = watcher_fun
        threading.Thread.__init__(self)

    def stop(self):
        self.running = False

    def run(self):
        while self.running:
            sleep(self.watch_frequency)
            self.watcher_fun(self)

class TouchWatcher(SensorWatcher):
    def __init__(self, event_receiver, watch_frequency = 0.1):
        self.ts         = TouchSensor()
        self.last_value = 0
        SensorWatcher.__init__(self, event_receiver, TouchWatcher.watcher_fun, watch_frequency)

    def watcher_fun(self):
        self.current_value = self.ts.value()
        if self.last_value != self.current_value:
            if self.last_value == 0:
                self.event_receiver.button_pressed()
            else:
                self.event_receiver.button_released()
            self.last_value = self.current_value
            
class GyroWatcher(SensorWatcher):
    def __init__(self, event_receiver, watch_frequency = 0.2):
        self.gs         = GyroSensor()
        self.last_angle = 0
        self.fix_drift()
        SensorWatcher.__init__(self, event_receiver, GyroWatcher.watcher_fun, watch_frequency)
    
    def fix_drift(self):
        self.gs.mode = 'GYRO-CAL'
        sleep(1)
        self.gs.mode = 'GYRO-ANG'
        self.measures = 4
        for i in range(self.measures):
            sleep(0.5)
            if self.gs.value() != 0:
                raise CalibrationError("Angle is drifting ", self.gs.value())

    def watcher_fun(self):
        self.current_angle = self.gs.value()
        if self.current_angle != self.last_angle:
            self.event_receiver.angle_changed(self.current_angle)
            self.last_angle = self.current_angle


class SensorEventHandler(threading.Thread):
    """ Simple FIFO Event Processor """
    def __init__(self, motor_thread):
        self.e        = threading.Event()
        self.active_q = 0
        self.event_q  = ([], [])
        self.motor_thread  = motor_thread
        self.is_running    = False
        #self.initial_angle = 0
        threading.Thread.__init__(self)

    def angle_changed(self, new_angle):
        self.event_q[self.active_q].append(("angle_changed", new_angle))        
        self.e.set()

    def button_pressed(self):
        self.event_q[self.active_q].append(("button_pressed", ))
        self.e.set()        

    def button_released(self):
        self.event_q[self.active_q].append(("button_released", ))
        self.e.set()

    def handle_button_release(self):
        if self.is_running:
            motor_thread.set_speed(0, 0)
            self.is_running = False
        else:
            motor_thread.set_speed(-80, -80)
            self.is_running = True

    def handle_angle_change(angle):
        if self.is_running:
            if self.
            
    def run(self):
        while True:
            self.e.wait()
            self.passive_q = self.active_q
            self.active_q  = (self.active_q + 1) % 2
            for event in self.event_q[self.passive_q]:
                print ("Processing", event)
                if event[0] == "button_released":
                    handle_button_release(self)
                elif event[0] == "angle_changed":
                    handle_angle_change(event[1])
            list.clear(self.event_q[self.passive_q])
                

def main():
    motor_thread = MotorThread()
    motor_thread.start()
    sensor_event_handler = SensorEventHandler(motor_thread)
    sensor_event_handler.start()
    gyro_watcher = GyroWatcher(sensor_event_handler)
    gyro_watcher.start()
    touch_watcher = TouchWatcher(sensor_event_handler)
    touch_watcher.start()
    print ("Started.")

if __name__ == "__main__":
    main()

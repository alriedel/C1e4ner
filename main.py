from ev3dev.ev3 import GyroSensor
import threading
from time import sleep

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
    def __init__(self):
        self.e        = threading.Event()
        self.active_q = 0
        self.event_q  = ([], [])
        threading.Thread.__init__(self)

    def angle_changed(self, new_angle):
        self.event_q[self.active_q].append(("angle_changed", new_angle))
        self.e.set()

    def run(self):
        while True:
            self.e.wait()
            self.passive_q = self.active_q
            self.active_q  = (self.active_q + 1) % 2
            for event in self.event_q[self.passive_q]:
                print ("Processing", event)
            list.clear(self.event_q[self.passive_q])
                

def main():
    sensor_event_handler = SensorEventHandler()
    sensor_event_handler.start()
    gyro_watcher = GyroWatcher(sensor_event_handler)
    gyro_watcher.start()
    print ("Started.")

if __name__ == "__main__":
    main()

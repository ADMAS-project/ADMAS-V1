"""
This additions are done by CANITEY
GOAL:
    make a sensing for the time span of blink, yawn, eye direction
"""
import time


class Timer:
    def __init__(self, duration=3, callback=None):
        self.duration_ = duration
        self.now_ = time.time()
        self.current_ = None
        self.callback_ = self.default_callback if callback == None else callback

    def default_callback(self):
        print("passed")

    def sense(self) -> bool : 
        self.current_ = time.time()
        if self.current_ - self.now_ >= self.duration_ :
            self.callback_()
            self.now_ = time.time()
            return True
        return False


class GlobalTimer:
    def __init__(self):
        self.timers_ = dict()

    def init_if_none(self, timer: str, duration=3, callback=None):
        timer_name = timer.lower()
        if self.timers_.get(timer_name) == None or self.timers_[timer_name] == None:
            self.timers_[timer_name] = Timer(duration=duration, callback=callback)
        return self.timers_[timer_name]
    
    def reset_all_timers(self):
        for timer in self.timers_:
            self.timers_[timer] = None

    def reset_except(self, timer: str):
        for t in self.timers_:
            if timer.lower() == t.lower():
                continue
            self.timers_[t] = None


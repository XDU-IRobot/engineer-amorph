import RPi.GPIO as GPIO
import time

class Button:
    def __init__(self, pin, debounce_delay=0.05):
        self.pin = pin
        self.debounce_delay = debounce_delay
        self.last_state = None
        self.last_change_time = 0
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def is_pressed(self):
        current_state = GPIO.input(self.pin) == GPIO.LOW
        current_time = time.time()
        
        # 如果状态改变了
        if current_state != self.last_state:
            # 检查是否超过debounce延迟
            if current_time - self.last_change_time >= self.debounce_delay:
                self.last_state = current_state
                self.last_change_time = current_time
                return current_state
        
        return self.last_state if self.last_state is not None else current_state
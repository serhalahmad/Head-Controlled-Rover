import RPi.GPIO as GPIO
import time

class Buzzer:
    def __init__(self, buzzer:int):
        """
        :param buzzer: buzzer pin BCM
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.buzzer=buzzer
        GPIO.setup(self.buzzer,GPIO.OUT)
#        print('Initialized buzzer')
    
    def turn_on(self):
        GPIO.output(self.buzzer,True)
#        print('Turned buzzer on')
    
    def turn_off(self):
        GPIO.output(self.buzzer,False)
#        print('Turned buzzer off')
        
def main():
    buzzer=Buzzer(26)
    for x in range(10):
        buzzer.turn_on()
        time.sleep(0.5)
        buzzer.turn_off()
        time.sleep(0.5)
    
if __name__ == '__main__':
    main()


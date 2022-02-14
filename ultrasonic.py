
import RPi.GPIO as GPIO
import time


class Ultrasonic:
    def __init__(self, trig: int, echo: int, frequency = 0.00001):
        """
        :param trig: trigger pin BCM
        :param echo: echo pin BCM
        :param frequency: frequency of sending signal
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.trig = trig
        self.echo = echo
        self.frequency = frequency
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.output(self.trig, False)
        time.sleep(2)

    def get_distance(self):
        GPIO.output(self.trig, True)
        time.sleep(self.frequency)
        GPIO.output(self.trig, False)

        while GPIO.input(self.echo) == 0:
            pulse_start = time.time()

        while GPIO.input(self.echo) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150
        distance = round(distance, 2)
#        print("Distance:", distance, "cm")
        return distance


def main():
	ultra = Ultrasonic(24,25)
	while True:	
		distance=ultra.get_distance()
		print(str(distance))

if __name__=='__main__':
	main()

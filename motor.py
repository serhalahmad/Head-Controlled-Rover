import RPi.GPIO as GPIO


class Motors:
    def __init__(self, en1: int, en2: int, in1: int, in2: int, in3: int, in4: int, duty_cycle: int):
        """
        :param en1: Enable 1 on motor drive
        :param en2: Enable 2 on motor drive
        :param in1: Input 1 on motor drive
        :param in2: Input 2 on motor drive
        :param in3: Input 3 on motor drive
        :param in4: Input 4 on motor drive
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.en1 = en1
        self.en2 = en2
        self.in1 = in1
        self.in2 = in2
        self.in3 = in3
        self.in4 = in4
        self.duty_cycle = duty_cycle

        GPIO.setup(en1, GPIO.OUT)
        GPIO.setup(en2, GPIO.OUT)
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.setup(in3, GPIO.OUT)
        GPIO.setup(in4, GPIO.OUT)
        self.pwm1 = GPIO.PWM(self.en1, 100)
        self.pwm1.start(0)
        self.pwm2 = GPIO.PWM(self.en2, 100)
        self.pwm2.start(0)

    def motor1(self, forward: bool, on: bool):
        """
        :param forward: True: moves motor forward / False: moves motor backward
        :param on: True: turns motor on / False: turns motor off
        :return:
        """
        if on:
            GPIO.output(self.in1, forward)
            GPIO.output(self.in2, not forward)
            self.pwm1.ChangeDutyCycle(self.duty_cycle)
            print('Motor 1 running')
        else:
            GPIO.output(self.in1, 1)
            GPIO.output(self.in2, 1)
            print('Motor 1 stopped')

    def motor2(self, forward: bool, on: bool):
        """
        :param forward: True: moves motor forward / False: moves motor backward
        :param on: True: turns motor on / False: turns motor off
        :return:
        """
        if on:
            GPIO.output(self.in3, forward)
            GPIO.output(self.in4, not forward)
            self.pwm2.ChangeDutyCycle(self.duty_cycle)
            print('Motor 2 running')
        else:
            GPIO.output(self.in3, 1)
            GPIO.output(self.in4, 1)
            print('Motor 2 Stopped')

    def turn_left(self):
        self.motor1(True,True)
        self.motor2(False,True)
        
    def turn_right(self):
        self.motor1(False,True)
        self.motor2(True,True)
        
    def move_forward(self):
        self.motor1(True,True)
        self.motor2(True,True)
    
    def stop(self):
        self.motor1(True,False)
        self.motor2(True,False)
        

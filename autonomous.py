import RPi.GPIO as GPIO
import time

# Define GPIO pins
TRIG = 23
ECHO = 24
SERVO = 21
IN1 = 17
IN2 = 18
IN3 = 22
IN4 = 27
ENA = 25
ENB = 26

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)

# Setup Ultrasonic Sensor
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Setup Servo
GPIO.setup(SERVO, GPIO.OUT)

# Setup Motor Driver
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Initialize PWM
pwm_left_motor = GPIO.PWM(ENA, 1000)
pwm_right_motor = GPIO.PWM(ENB, 1000)
servo_pwm = GPIO.PWM(SERVO, 50)

pwm_left_motor.start(0)
pwm_right_motor.start(0)
servo_pwm.start(7.5)

def measure_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.1)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    pulse_start = time.time()
    pulse_end = time.time()

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)

def move_forward(speed=70):
    pwm_left_motor.ChangeDutyCycle(speed)
    pwm_right_motor.ChangeDutyCycle(speed)
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)

def stop():
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, False)
    pwm_left_motor.ChangeDutyCycle(0)
    pwm_right_motor.ChangeDutyCycle(0)

def turn_left(speed=70, duration=1):
    pwm_left_motor.ChangeDutyCycle(speed)
    pwm_right_motor.ChangeDutyCycle(speed)
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)
    time.sleep(duration)
    stop()

def turn_right(speed=70, duration=1):
    pwm_left_motor.ChangeDutyCycle(speed)
    pwm_right_motor.ChangeDutyCycle(speed)
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)
    time.sleep(duration)
    stop()

def set_servo_angle(angle):
    duty_cycle = (angle / 18.0) + 2.5
    duty_cycle = max(2.5, min(duty_cycle, 12.5))
    servo_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)

def scan_for_gap():
    directions = {"right": 0, "left": 0}

    for angle in range(0, 91, 30):
        set_servo_angle(angle)
        directions["right"] = max(directions["right"], measure_distance())

    set_servo_angle(0)
    time.sleep(0.5)

    for angle in range(0, -91, -30):
        set_servo_angle(180 + angle)
        directions["left"] = max(directions["left"], measure_distance())

    set_servo_angle(0)

    if directions["right"] > directions["left"]:
        return "right"
    elif directions["left"] > directions["right"]:
        return "left"
    else:
        return "stop"

def decide_movement():
    set_servo_angle(0)
    front_distance = measure_distance()

    if front_distance > 20:
        move_forward()
    else:
        stop()
        time.sleep(0.5)
        direction = scan_for_gap()

        if direction == "right":
            turn_right()
        elif direction == "left":
            turn_left()
        else:
            stop()

def main():
    try:
        while True:
            decide_movement()
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop()
        GPIO.cleanup()
        print("Program terminated.")

if __name__ == "__main__":
    main()

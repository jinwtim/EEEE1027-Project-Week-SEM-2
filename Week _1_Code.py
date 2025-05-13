import RPi.GPIO as GPIO
import time

# === Motor Control Pins ===
IN1 = 17  # Left motor forward
IN2 = 22  # Left motor backward
IN3 = 23  # Right motor forward
IN4 = 24  # Right motor backward

# === Encoder Pins ===
EncoderR_GPIO = 20  # Right wheel encoder
EncoderL_GPIO = 21  # Left wheel encoder

# === Encoder Parameters ===
PPR = 20  # Pulses per revolution
wheelCircumference = 22  # cm per wheel revolution

# === Global Variables ===
countR = 0  # Right wheel pulse counter
countL = 0  # Left wheel pulse counter
total_distanceR = 0  # Total distance for right wheel
total_distanceL = 0  # Total distance for left wheel

# === Setup GPIO ===
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(EncoderR_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(EncoderL_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# === Create PWM Objects ===
pwm_left_fwd = GPIO.PWM(IN1, 100)
pwm_left_bwd = GPIO.PWM(IN2, 100)
pwm_right_fwd = GPIO.PWM(IN3, 100)
pwm_right_bwd = GPIO.PWM(IN4, 100)

# Start PWM at 0% (stopped)
pwm_left_fwd.start(0)
pwm_left_bwd.start(0)
pwm_right_fwd.start(0)
pwm_right_bwd.start(0)


# === Encoder Callback Functions ===
def encoder_callbackR(channel):
    global countR
    countR += 0.5


def encoder_callbackL(channel):
    global countL
    countL += 0.5


# Attach interrupt to count pulses
GPIO.add_event_detect(EncoderR_GPIO, GPIO.RISING, callback=encoder_callbackR, bouncetime=1)
GPIO.add_event_detect(EncoderL_GPIO, GPIO.RISING, callback=encoder_callbackL, bouncetime=1)


# === Motor Control Functions ===
def forward(distance_target, speed):
    """ Move forward for a specific distance (cm) and return average distance traveled. """
    global countR, countL, total_distanceR, total_distanceL
    countR = 0  # Reset right wheel pulse counter
    countL = 0  # Reset left wheel pulse counter

    pwm_left_fwd.ChangeDutyCycle(speed)
    pwm_right_fwd.ChangeDutyCycle(speed)
    pwm_left_bwd.ChangeDutyCycle(0)
    pwm_right_bwd.ChangeDutyCycle(0)

    while ((countR / PPR * wheelCircumference) < distance_target and
           (countL / PPR * wheelCircumference) < distance_target):
        time.sleep(0.01)

    stop_motors()

    # Calculate distances for both wheels
    distanceR = (countR / PPR) * wheelCircumference
    distanceL = (countL / PPR) * wheelCircumference
    avg_distance = (distanceR + distanceL) / 2  # Average distance

    # Update total distances
    total_distanceR += distanceR
    total_distanceL += distanceL

    print(f"Forward: Left={distanceL:.2f} cm, Right={distanceR:.2f} cm, Avg={avg_distance:.2f} cm")
    return avg_distance


def reverse(distance_target, speed):
    """ Move backward for a specific distance (cm) and return average distance traveled. """
    global countR, countL, total_distanceR, total_distanceL
    countR = 0  # Reset right wheel pulse counter
    countL = 0  # Reset left wheel pulse counter

    pwm_left_bwd.ChangeDutyCycle(speed)
    pwm_right_bwd.ChangeDutyCycle(speed)
    pwm_left_fwd.ChangeDutyCycle(0)
    pwm_right_fwd.ChangeDutyCycle(0)

    while ((countR / PPR * wheelCircumference) < distance_target and
           (countL / PPR * wheelCircumference) < distance_target):
        time.sleep(0.01)

    stop_motors()

    # Calculate distances for both wheels
    distanceR = (countR / PPR) * wheelCircumference
    distanceL = (countL / PPR) * wheelCircumference
    avg_distance = (distanceR + distanceL) / 2  # Average distance

    # Update total distances
    total_distanceR += distanceR
    total_distanceL += distanceL

    print(f"Reverse: Left={distanceL:.2f} cm, Right={distanceR:.2f} cm, Avg={avg_distance:.2f} cm")
    return avg_distance


def turn_left(deg, speed):
    """ Turn left by a specified degree. """
    global countR, countL, total_distanceR, total_distanceL
    countR = 0  # Reset right wheel pulse counter
    countL = 0  # Reset left wheel pulse counter

    distance_target = 0.22 * deg

    pwm_left_bwd.ChangeDutyCycle(speed)
    pwm_right_fwd.ChangeDutyCycle(speed)
    pwm_left_fwd.ChangeDutyCycle(0)
    pwm_right_bwd.ChangeDutyCycle(0)

    while ((countR / PPR * wheelCircumference) < distance_target and
           (countL / PPR * wheelCircumference) < distance_target):
        time.sleep(0.01)

    stop_motors()

    # Calculate distances for both wheels
    distanceR = (countR / PPR) * wheelCircumference
    distanceL = (countL / PPR) * wheelCircumference
    avg_distance = (distanceR + distanceL) / 2  # Average distance

    # Update total distances
    total_distanceR += distanceR
    total_distanceL += distanceL

    print(f"Forward: Left={distanceL:.2f} cm, Right={distanceR:.2f} cm, Avg={avg_distance:.2f} cm")
    return avg_distance

    # time.sleep(deg / 360 * 1)  # Adjust for proper rotation timing
    # stop_motors()


def turn_right(deg, speed):
    """ Turn right by a specified degree. """
    global countR, countL, total_distanceR, total_distanceL
    countR = 0  # Reset right wheel pulse counter
    countL = 0  # Reset left wheel pulse counter

    distance_target = 0.22 * deg

    pwm_left_fwd.ChangeDutyCycle(speed)
    pwm_right_bwd.ChangeDutyCycle(speed)
    pwm_left_bwd.ChangeDutyCycle(0)
    pwm_right_fwd.ChangeDutyCycle(0)

    while ((countR / PPR * wheelCircumference) < distance_target and
           (countL / PPR * wheelCircumference) < distance_target):
        time.sleep(0.01)

    stop_motors()

    distanceR = (countR / PPR) * wheelCircumference
    distanceL = (countL / PPR) * wheelCircumference
    avg_distance = (distanceR + distanceL) / 2  # Average distance

    # Update total distances
    total_distanceR += distanceR
    total_distanceL += distanceL

    print(f"Forward: Left={distanceL:.2f} cm, Right={distanceR:.2f} cm, Avg={avg_distance:.2f} cm")
    return avg_distance

    # time.sleep(deg / 360 * 1)
    # stop_motors()


def stop_motors():
    """ Stop all motors. """
    pwm_left_fwd.ChangeDutyCycle(0)
    pwm_left_bwd.ChangeDutyCycle(0)
    pwm_right_fwd.ChangeDutyCycle(0)
    pwm_right_bwd.ChangeDutyCycle(0)


# === Example Usage ===
print("Moving Forward 50 cm at 75% speed")
avg_forward = forward(100, 100)

print("Moving Reverse 30 cm at 50% speed")
avg_reverse = reverse(100, 100)

print("Turning Right")
avg_right = turn_right(90, 100)

time.sleep(1)

print("Turning Left")
avg_left = turn_left(85, 100)

# Calculate final total average distance
# final_avg_distance = (total_distanceR + total_distanceL) / 2
# print(f"Final Total Average Distance Traveled: {final_avg_distance:.2f} cm")

# Cleanup GPIO at the very end
GPIO.cleanup()

import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# === GPIO Motor Pins ===
LEFT_MOTOR_FORWARD = 17
LEFT_MOTOR_BACKWARD = 22
RIGHT_MOTOR_FORWARD = 23
RIGHT_MOTOR_BACKWARD = 24

# === Initialize GPIO ===
GPIO.setmode(GPIO.BCM)
GPIO.setup([LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD,
            RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD], GPIO.OUT)

# === PWM Motor Control ===
LEFT_PWM_FWD = GPIO.PWM(LEFT_MOTOR_FORWARD, 100)
LEFT_PWM_BWD = GPIO.PWM(LEFT_MOTOR_BACKWARD, 100)
RIGHT_PWM_FWD = GPIO.PWM(RIGHT_MOTOR_FORWARD, 100)
RIGHT_PWM_BWD = GPIO.PWM(RIGHT_MOTOR_BACKWARD, 100)

LEFT_PWM_FWD.start(0)
LEFT_PWM_BWD.start(0)
RIGHT_PWM_FWD.start(0)
RIGHT_PWM_BWD.start(0)

# === Speed Control Parameters ===
MAX_SPEED = 100  # Maximum motor speed (% duty cycle)
MIN_SPEED = 0  # Minimum motor speed (% duty cycle)
TURN_SPEED = 80  # Speed during sharp turns
BASE_SPEED = 40  # Normal cruising speed for PID control
# === PID Control Parameters ===
KP = 0.5  # Proportional gain
KI = 0  # Integral gain
KD = 5.2  # Derivative gain

# == Threshold Parameters ===
SHARP_TURN_THRESHOLD = 180  # Pixels from center to trigger sharp turn
TURN_DURATION = 0.6  # Seconds to maintain sharp turn before rechecking

# === System Variables ===
previous_error = 0
integral = 0
last_turn_time = 0
in_sharp_turn = False

# === Initialize PiCamera2 ===
picam2 = Picamera2()
picam2.preview_configuration.main.size = (320, 240)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.configure("preview")
picam2.start()
time.sleep(2)  # Camera warm-up time


def stop_motors():
    LEFT_PWM_FWD.ChangeDutyCycle(0)
    LEFT_PWM_BWD.ChangeDutyCycle(60)
    RIGHT_PWM_FWD.ChangeDutyCycle(0)
    RIGHT_PWM_BWD.ChangeDutyCycle(60)
    time.sleep(0.28)
    """Stop all motors"""
    LEFT_PWM_FWD.ChangeDutyCycle(0)
    LEFT_PWM_BWD.ChangeDutyCycle(0)
    RIGHT_PWM_FWD.ChangeDutyCycle(0)
    RIGHT_PWM_BWD.ChangeDutyCycle(0)


def preprocess_frame(frame):
    """Convert frame to HSV and apply color thresholding for line detection"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 100])
    mask = cv2.inRange(hsv, lower_black, upper_black)
    return mask, hsv


def find_line(thresh):
    """Detects the main line using contours and finds its center"""
    roi = thresh[160:240, :]  # Focus on lower portion of image
    contours, _ = cv2.findContours(roi, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 100:  # Minimum contour area
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                return cx
    return None


def execute_sharp_turn(error):
    """Execute a sharp turn maneuver"""
    global last_turn_time, in_sharp_turn
    print(error)

    if error > 0:  # Line is to the right - pivot left
        LEFT_PWM_FWD.ChangeDutyCycle(0)
        LEFT_PWM_BWD.ChangeDutyCycle(TURN_SPEED)
        RIGHT_PWM_FWD.ChangeDutyCycle(TURN_SPEED)
        RIGHT_PWM_BWD.ChangeDutyCycle(0)
    else:  # Line is to the left - pivot right
        LEFT_PWM_FWD.ChangeDutyCycle(TURN_SPEED)
        LEFT_PWM_BWD.ChangeDutyCycle(0)
        RIGHT_PWM_FWD.ChangeDutyCycle(0)
        RIGHT_PWM_BWD.ChangeDutyCycle(TURN_SPEED)

    last_turn_time = time.time()
    in_sharp_turn = True


def pid_control(error):
    """Normal PID control for smooth line following"""
    global previous_error, integral, in_sharp_turn

    # Reset integral if coming out of sharp turn
    if in_sharp_turn:
        integral = 0
        in_sharp_turn = False

    # Calculate PID terms
    integral += error
    derivative = error - previous_error
    correction = KP * error + KI * integral + KD * derivative

    # Dynamic speed adjustment
    base_speed = BASE_SPEED * (1 - 0.3 * (abs(error) / SHARP_TURN_THRESHOLD))
    base_speed = max(base_speed, MIN_SPEED)

    # Apply correction
    left_speed = base_speed - correction
    right_speed = base_speed + correction

    # Constrain speeds
    left_speed = max(min(left_speed, MAX_SPEED), MIN_SPEED)
    right_speed = max(min(right_speed, MAX_SPEED), MIN_SPEED)

    # Drive motors forward
    LEFT_PWM_FWD.ChangeDutyCycle(left_speed)
    LEFT_PWM_BWD.ChangeDutyCycle(0)
    RIGHT_PWM_FWD.ChangeDutyCycle(right_speed)
    RIGHT_PWM_BWD.ChangeDutyCycle(0)

    previous_error = error
    return correction


def control_motors(cx, frame_width):
    """Main control logic deciding between PID and sharp turns"""
    center_x = frame_width // 2

    if cx is None:
        stop_motors()
        return

    error = cx - center_x
    abs_error = abs(error)
    current_time = time.time()

    # Check if we should still be in sharp turn mode
    if in_sharp_turn and (current_time - last_turn_time) < TURN_DURATION:
        return

    # Check for sharp turn condition
    if abs_error > SHARP_TURN_THRESHOLD:
        execute_sharp_turn(error)
    else:
        correction = pid_control(error)
        # Debug info
        print(f"PID Control | Error: {error:4.1f} | Correction: {correction:4.1f} | "
              f"L: {MAX_SPEED - correction:4.1f}% | R: {MAX_SPEED + correction:4.1f}%")


try:
    while True:
        # Capture frame from camera
        frame = picam2.capture_array()

        # Preprocess frame (convert to HSV and threshold)
        mask, hsv = preprocess_frame(frame)

        # Find the line
        cx = find_line(mask)

        # Control motors based on line position
        control_motors(cx, frame.shape[1])

        # Display images (optional)
        cv2.imshow("RGB Preview", frame)
        cv2.imshow("Thresholded", mask)

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Cleanup
    cv2.destroyAllWindows()
    LEFT_PWM_FWD.stop()
    LEFT_PWM_BWD.stop()
    RIGHT_PWM_FWD.stop()
    RIGHT_PWM_BWD.stop()
    GPIO.cleanup()
    picam2.stop()

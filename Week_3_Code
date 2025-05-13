import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import math

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
MAX_SPEED = 100
MIN_SPEED = 0
BASE_SPEED = 30
SLOW_SPEED = 10

# === PID Control Parameters ===
KP = 1.5
KI = 0.00
KD = 15

# === ROI Parameters ===
MAX_ROI_HEIGHT = 240
MIN_ROI_HEIGHT = 80

# === System Variables ===
previous_error = 0
integral = 0

# === Shape Detection Parameters ===
arrow_minHSV = np.array([100, 150, 50])  # Adjust these values for your blue arrow
arrow_maxHSV = np.array([140, 255, 255])
shape_counter = 0
final_shape_text = ""

# === Initialize PiCamera2 ===
picam2 = Picamera2()
picam2.preview_configuration.main.size = (320, 240)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 60
picam2.configure("preview")
picam2.start()
time.sleep(2)

# === Prompt User for Color Selection ===
VALID_COLORS = {"red", "green", "blue", "yellow"}
selected_colors = []

print("Please choose two colors from the following: red, green, blue, yellow")
while len(selected_colors) < 2:
    color = input(f"Enter color {len(selected_colors) + 1}: ").strip().lower()
    if color in VALID_COLORS and color not in selected_colors:
        selected_colors.append(color)
    else:
        print("Invalid or duplicate color. Please try again.")

print(f"Selected colors: {selected_colors}")


def stop_motors():
    LEFT_PWM_FWD.ChangeDutyCycle(0)
    LEFT_PWM_BWD.ChangeDutyCycle(0)
    RIGHT_PWM_FWD.ChangeDutyCycle(0)
    RIGHT_PWM_BWD.ChangeDutyCycle(0)


def no_line():
    LEFT_PWM_FWD.ChangeDutyCycle(0)
    LEFT_PWM_BWD.ChangeDutyCycle(50)
    RIGHT_PWM_FWD.ChangeDutyCycle(0)
    RIGHT_PWM_BWD.ChangeDutyCycle(50)
    time.sleep(0.5)


def preprocess_frame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 100])
    lower_yellow = np.array([75, 77, 0])
    upper_yellow = np.array([105, 255, 255])
    lower_red = np.array([104, 72, 114])
    upper_red = np.array([128, 255, 207])
    lower_green = np.array([30, 111, 88])
    upper_green = np.array([82, 253, 195])
    lower_blue = np.array([0, 120, 72])
    upper_blue = np.array([32, 204, 129])

    color_mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)

    if "red" in selected_colors:
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        color_mask |= mask_red
    if "green" in selected_colors:
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        color_mask |= mask_green
    if "blue" in selected_colors:
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        color_mask |= mask_blue
    if "yellow" in selected_colors:
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        color_mask |= mask_yellow

    mask_black = cv2.inRange(hsv, lower_black, upper_black)
    black_only_mask = mask_black & ~color_mask
    combined_mask = color_mask | black_only_mask

    return combined_mask, color_mask, black_only_mask


def find_line(combined_mask, color_mask, black_only_mask):
    speed_factor = BASE_SPEED / MAX_SPEED
    roi_height = MIN_ROI_HEIGHT + int((MAX_ROI_HEIGHT - MIN_ROI_HEIGHT) * speed_factor)
    roi_top = 240 - roi_height

    roi_combined = combined_mask[roi_top:240, :]
    roi_color = color_mask[roi_top:240, :]
    roi_black = black_only_mask[roi_top:240, :]

    color_contours, _ = cv2.findContours(roi_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if color_contours:
        largest = max(color_contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 100:
            M = cv2.moments(largest)
            if M["m00"] > 0:
                return int(M["m10"] / M["m00"])

    black_contours, _ = cv2.findContours(roi_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if black_contours:
        largest = max(black_contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 200:
            M = cv2.moments(largest)
            if M["m00"] > 0:
                return int(M["m10"] / M["m00"])

    return None


def pid_control(error):
    global previous_error, integral

    integral += error
    derivative = error - previous_error
    correction = KP * error + KI * integral + KD * derivative

    left_speed = BASE_SPEED - correction
    right_speed = BASE_SPEED + correction

    left_speed = np.clip(left_speed, MIN_SPEED, MAX_SPEED)
    right_speed = np.clip(right_speed, MIN_SPEED, MAX_SPEED)

    LEFT_PWM_FWD.ChangeDutyCycle(left_speed)
    LEFT_PWM_BWD.ChangeDutyCycle(0)
    RIGHT_PWM_FWD.ChangeDutyCycle(right_speed)
    RIGHT_PWM_BWD.ChangeDutyCycle(0)

    previous_error = error
    return correction


def detect_shape(cnt, frame, hsv_frame):
    global final_shape_text, shape_counter

    shape = "unknown"
    peri = cv2.arcLength(cnt, True)
    area = cv2.contourArea(cnt)

    if area < 5000 or len(cnt) < 3:
        return None

    epsilon = 0.03 * peri if area > 5000 else 0.05 * peri
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    num_vertices = len(approx)

    # First check for triangle (simplest shape)
    if num_vertices == 3:
        shape = "triangle"

    # Check for arrow specifically if it's blue
    elif 4 <= num_vertices <= 8:
        # Get color information to confirm if it's blue
        maskHSV = cv2.inRange(hsv_frame, arrow_minHSV, arrow_maxHSV)
        kernel = np.ones((5, 5), np.uint8)
        maskHSV = cv2.morphologyEx(maskHSV, cv2.MORPH_CLOSE, kernel)
        maskHSV = cv2.morphologyEx(maskHSV, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        x, y, w, h = cv2.boundingRect(cnt)
        x, y = max(x - 10, 0), max(y - 10, 0)
        w = min(w + 20, frame.shape[1] - x)
        h = min(h + 20, frame.shape[0] - y)
        arrow_region = maskHSV[y:y + h, x:x + w]

        if arrow_region.size > 0:
            blurIm = cv2.GaussianBlur(arrow_region, (9, 9), 0)
            corners = cv2.goodFeaturesToTrack(blurIm, 2, 0.7, 15)

            if corners is not None and len(corners) >= 2:
                corners = np.int0(corners)z
                x0, y0 = corners[0].ravel()
                x1, y1 = corners[1].ravel()
                x0, y0 = x0 + x, y0 + y
                x1, y1 = x1 + x, y1 + y

                cv2.circle(frame, (x0, y0), 5, (0, 0, 255), -1)
                cv2.circle(frame, (x1, y1), 5, (0, 0, 255), -1)

                am, bm = (x0 + x1) / 2, (y0 + y1) / 2
                cv2.circle(frame, (int(am), int(bm)), 3, (255, 0, 0), -1)

                (cx, cy), radius = cv2.minEnclosingCircle(cnt)
                cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 0, 255), 2)
                cv2.line(frame, (int(cx), int(cy)), (int(am), int(bm)), (255, 0, 0), 2)

                angle = math.degrees(math.atan2(bm - cy, am - cx))
                if -45 <= angle < 45:
                    shape = "arrow (right)"
                elif 45 <= angle < 135:
                    shape = "arrow (down)"
                elif -180 <= angle <= -135 or 135 <= angle <= 180:
                    shape = "arrow (left)"
                elif -135 < angle < -45:
                    shape = "arrow (up)"

    # If it wasn't an arrow, continue with regular shape detection
    if shape == "unknown":
        if num_vertices == 4:
            x, y, w, h = cv2.boundingRect(approx)
            ar = w / float(h)
            shape = "square" if 0.95 <= ar <= 1.05 else "rectangle"
        elif num_vertices == 5:
            shape = "pentagon"
        elif num_vertices == 6:
            shape = "hexagon"
        elif num_vertices > 6:
            circularity = (4 * math.pi * area) / (peri * peri)
            shape = "full circle" if circularity > 0.8 else "partial circle"

    # Only return known shapes (ignore "unknown")
    if shape == "unknown":
        return None

    # Shape persistence logic
    shape_text = shape if shape.startswith("arrow") else shape
    if final_shape_text != shape_text:
        shape_counter += 1
    else:
        shape_counter = 0
    if shape_counter >= 5:
        final_shape_text = shape_text
        return shape_text

    return None


try:
    while True:
        frame = picam2.capture_array()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Shape detection
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY, 11, 2)
        edges = cv2.Canny(thresh, 120, 100)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_shape = None
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                shape = detect_shape(cnt, frame, hsv_frame)
                if shape is not None:
                    detected_shape = shape
                    print(f"Detected shape: {shape}")
                    cv2.drawContours(frame, [cnt], -1, (0, 0, 255), 2)
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        cv2.putText(frame, final_shape_text, (cX, cY),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Handle detected shapes
        if detected_shape:
            if detected_shape.startswith("arrow"):
                print(f"Arrow detected: {detected_shape}")
                # Add arrow direction handling here
                stop_motors()
                time.sleep(2)
                LEFT_PWM_FWD.ChangeDutyCycle(50)
                LEFT_PWM_BWD.ChangeDutyCycle(0)
                RIGHT_PWM_FWD.ChangeDutyCycle(50)
                RIGHT_PWM_BWD.ChangeDutyCycle(0)
                time.sleep(0.5)

            else:
                print(f"Shape detected: {detected_shape}")
                stop_motors()
                time.sleep(2)
                LEFT_PWM_FWD.ChangeDutyCycle(50)
                LEFT_PWM_BWD.ChangeDutyCycle(0)
                RIGHT_PWM_FWD.ChangeDutyCycle(50)
                RIGHT_PWM_BWD.ChangeDutyCycle(0)
                time.sleep(0.5)
                continue

        # Normal line following
        combined_mask, color_mask, black_only_mask = preprocess_frame(frame)
        cx = find_line(combined_mask, color_mask, black_only_mask)

        if cx is not None:
            error = cx - 160
            pid_control(error)
        else:
            no_line()

        # Debug view
        cv2.imshow("Line View", combined_mask)
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    stop_motors()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    picam2.stop()

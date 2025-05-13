import cv2
import numpy as np


def nothing(x):
    pass

# Create a window
cv2.namedWindow('HSV Calibration')

# Create trackbars for HSV ranges
cv2.createTrackbar('H_min', 'HSV Calibration', 0, 179, nothing)
cv2.createTrackbar('H_max', 'HSV Calibration', 179, 179, nothing)
cv2.createTrackbar('S_min', 'HSV Calibration', 0, 255, nothing)
cv2.createTrackbar('S_max', 'HSV Calibration', 255, 255, nothing)
cv2.createTrackbar('V_min', 'HSV Calibration', 0, 255, nothing)
cv2.createTrackbar('V_max', 'HSV Calibration', 255, 255, nothing)

picam2 = Picamera2()
picam2.preview_configuration.main.size = (320, 240)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 60
picam2.configure("preview")
picam2.start()
time.sleep(2)

while True:
    frame = picam2.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # Get current trackbar positions
    h_min = cv2.getTrackbarPos('H_min', 'HSV Calibration')
    h_max = cv2.getTrackbarPos('H_max', 'HSV Calibration')
    s_min = cv2.getTrackbarPos('S_min', 'HSV Calibration')
    s_max = cv2.getTrackbarPos('S_max', 'HSV Calibration')
    v_min = cv2.getTrackbarPos('V_min', 'HSV Calibration')
    v_max = cv2.getTrackbarPos('V_max', 'HSV Calibration')

    # Create HSV range
    lower_green = np.array([h_min, s_min, v_min])
    upper_green = np.array([h_max, s_max, v_max])

    # Create mask
    mask = cv2.inRange(hsv, lower_green, upper_green)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Show results
    cv2.imshow('Original', frame)
    cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()

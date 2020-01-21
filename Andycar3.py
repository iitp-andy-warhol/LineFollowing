from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from adafruit_servokit import ServoKit


# Initializing
camera = PiCamera()
camera.resolution = (320, 240)
rawCapture = PiRGBArray(camera, size=(320, 240))
time.sleep(0.1)

x_last = 160
y_last = 120

start_time = time.time()
counter = 0

kit = ServoKit(channels=16)


# Motor control
def Motor_Steer(speed, steering, stop=False):
    if stop == True:
        kit.continuous_servo[0].throttle = 0
        kit.continuous_servo[1].throttle = 0
        return
    elif steering == 0:
        kit.continuous_servo[0].throttle = speed
        kit.continuous_servo[1].throttle = -1 * speed
        return
    elif steering > 0:
        steering = 100 - steering
        kit.continuous_servo[0].throttle = speed
        kit.continuous_servo[1].throttle = -1 * speed * steering / 100
        return
    elif steering < 0:
        steering = steering * -1
        steering = 100 - steering
        kit.continuous_servo[0].throttle = speed * steering / 100
        kit.continuous_servo[1].throttle = -1 * speed
        return

# Line following parameters
kp = 0.75  # off line
ap = 1.0  # off angle

# Video capture code
# videoFile1 = './video001.avi'
# fourcc = cv2.VideoWriter_fourcc(*'DIVX')
# out = cv2.VideoWriter(videoFile1, fourcc, 9.0, (320,240))


# Main Loop
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    counter += 1
    image = frame.array
    # out.write(image)
    roi = image[60:239, 0:319]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    #yellow_lower = np.array([22, 60, 200], np.uint8)
    yellow_lower = np.array([22, 0, 150], np.uint8)
    yellow_upper = np.array([60, 255, 255], np.uint8)
    yellow_line = cv2.inRange(hsv, yellow_lower, yellow_upper)

    red_lower = np.array([170, 120, 70], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)
    stop_sign = cv2.inRange(hsv, red_lower, red_upper)

    # kernel = np.ones((3, 3), np.uint8)
    # yellow_line = cv2.erode(yellow_line, kernel, iterations=3)
    # yellow_line = cv2.dilate(yellow_line, kernel, iterations=3)
    # stop_sign = cv2.erode(stop_sign, kernel, iterations=3)
    # stop_sign = cv2.dilate(stop_sign, kernel, iterations=5)
    contours_blk, hierarchy_blk = cv2.findContours(yellow_line.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, hierarchy_red = cv2.findContours(stop_sign.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Detect Stop sign
    if len(contours_red) > 0:
        address0_time = time.time() - start_time
        #print("stopsign: ", address0_time)

    # Detect Yellow line
    contours_blk_len = len(contours_blk)
    if contours_blk_len > 0:
        if contours_blk_len == 1:
            blackbox = cv2.minAreaRect(contours_blk[0])
        else:
            canditates = []
            off_bottom = 0
            for con_num in range(contours_blk_len):
                blackbox = cv2.minAreaRect(contours_blk[con_num])
                (x_min, y_min), (w_min, h_min), ang = blackbox
                box = cv2.boxPoints(blackbox)
                (x_box, y_box) = box[0]
                if y_box > 238:
                    off_bottom += 1
                canditates.append((y_box, con_num, x_min, y_min))
            canditates = sorted(canditates)
            if off_bottom > 1:
                canditates_off_bottom = []
                for con_num in range((contours_blk_len - off_bottom), contours_blk_len):
                    (y_highest, con_highest, x_min, y_min) = canditates[con_num]
                    total_distance = (abs(x_min - x_last) ** 2 + abs(y_min - y_last) ** 2) ** 0.5
                    canditates_off_bottom.append((total_distance, con_highest))
                canditates_off_bottom = sorted(canditates_off_bottom)
                (total_distance, con_highest) = canditates_off_bottom[0]
                blackbox = cv2.minAreaRect(contours_blk[con_highest])
            else:
                (y_highest, con_highest, x_min, y_min) = canditates[contours_blk_len - 1]
                blackbox = cv2.minAreaRect(contours_blk[con_highest])
        (x_min, y_min), (w_min, h_min), ang = blackbox
        x_last = x_min
        y_last = y_min
        if ang < -45:
            ang = 90 + ang
        if w_min < h_min and ang > 0:
            ang = (90 - ang) * -1
        if w_min > h_min and ang < 0:
            ang = 90 + ang
        setpoint = 160
        error = int(x_min - setpoint)
        ang = int(ang)


        # Draw PID factors
        box = cv2.boxPoints(blackbox)
        box = np.int0(box)
        #cv2.drawContours(image, [box], 0, (0, 0, 255), 3)
        #cv2.putText(image, str(ang), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        #cv2.putText(image, str(error), (10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        #cv2.line(image, (int(x_min), 190), (int(x_min), 230), (255, 0, 0), 3)

        # cv2.drawContours(image, contours_red, -1, (0,255,0), 3)


    # Show Image
    cv2.imshow("original with line", image)
    rawCapture.truncate(0)


    # Keyboard control
    key = cv2.waitKey(1) & 0xFF
    if key == ord("z"):
        mmode_flag = True
        print("M-mode On")
    elif key == ord("x"):
        mmode_flag = False
        print("M-mode Off")
    elif key == ord("q"):
        kit.continuous_servo[0].throttle = 0
        kit.continuous_servo[1].throttle = 0
        break


# out.release()
# cv2.destroyAllWindows()

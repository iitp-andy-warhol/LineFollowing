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


# Green Sign Detection_HY
def make_cropped(img):
    '''
    lower_orange = (0, 200, 200)
    upper_orange = (30, 255, 255)
    '''

    lower_green = (40, 100, 0)
    upper_green = (70, 255, 200)
    '''
    lower_blue = (0, 180, 55)
    upper_blue = (20, 255, 200)
    '''

    # img = cv2.imread(img_path, cv2.IMREAD_COLOR)
    # cv2.imwrite("img_original.jpeg", img)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_mask = cv2.inRange(img_hsv, lower_green, upper_green)
    img_green_only = cv2.bitwise_and(img, img, mask=img_mask)

    # cv2.imwrite("result_green_only.jpeg", img_green_only)
    img_gray = cv2.cvtColor(img_green_only, cv2.COLOR_BGR2GRAY)

    # blurred = cv2.GaussianBlur(img_gray, (5, 5), 0)

    # cv2.imwrite("result_blurred.jpeg", img_gray)
    ret, img_binary = cv2.threshold(img_gray, 20, 255, cv2.THRESH_BINARY)
    # cv2.imwrite("result_binary.jpeg", img_binary)

    # contours, _ = cv2.findContours(img_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours, _ = cv2.findContours(img_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    tmp_con = []
    for c in contours:
        area = cv2.contourArea(c)
        # print(area)
        x, y, w, h = cv2.boundingRect(c)
        '''
        img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),1)
        cv2.imwrite('rrrrr.jpeg', img)
        '''
        if (area < 2000):
            if (area > 100):  # 200
                if (w / h > 1.25):
                    tmp_con.append(c)

    contours = tmp_con
    max_area = -1
    max_con = 0
    # print(len(tmp_con))

    if (len(tmp_con) != 0):
        for c in contours:
            area = cv2.contourArea(c)
            # print(area)

            x, y, w, h = cv2.boundingRect(c)
            # img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),1)

            if (area > max_area):
                max_area = area
                max_con = c

        x, y, w, h = cv2.boundingRect(max_con)

        if (w / h > 1.25):
            # cropped = img[y:y+h, x:x+w]
            # cv2.imwrite('rrrrr.jpeg', cropped)
            # print("Hello")
            return 1

        else:
            return 0

    else:
        return 0


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

    
    # Detect Green sign
    result = make_cropped(image)
    if (result == 1):
        print(" # %d : Yes! Detected! \n")

    elif (result == 0):
        print(" # %d : NO!\n")

    
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

        # Move Motors
        if mmode_flag:
            Motor_Steer(0.4, (error * kp) + (ang * ap), True)
        else:
            Motor_Steer(0.4, (error * kp) + (ang * ap))
        
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

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import sys
import pickle


# for path in len(sys.argv)+1:
#     a = None
start = True
mmode_flag = False
ccw = True
stop0 = False
stop1 = False
stop2 = False
stop3 = False
stop4 = False
stop5 = False
stop6 = False
msg0 = False
msg1 = False
msg2 = False
msg3 = False
msg4 = False
msg5 = False
msg6 = False
turn0 = False
turn1 = False
turn2 = False
turn3 = False
turn4 = False
turn5 = False
turn6 = False

# path = [2, 3, 9, 0]
b = 0
for temp in range(len(sys.argv)-1):
    e = int(sys.argv[temp+1])
    if e == 9:
        if b == 1:
            turn1 = True
        elif b == 2:
            turn2 = True
        elif b == 3:
            turn3 = True
        elif b == 4:
            turn4 = True
        elif b == 5:
            turn5 = True
        elif b == 6:
            turn6 = True
        elif b == 0:
            turn0 = True
    else:
        if e == 1:
            stop1 = True
            msg1 = True
        elif e == 2:
            stop2 = True
            msg2 = True
        elif e == 3:
            stop3 = True
            msg3 = True
        elif e == 4:
            stop4 = True
            msg4 = True
        elif e == 5:
            stop5 = True
            msg5 = True
        elif e == 6:
            stop6 = True
            msg6 = True
        elif e == 0:
            stop0 = True
            msg0 = True
    b = np.copy(e)

camera = PiCamera()
camera.resolution = (320, 240)
rawCapture = PiRGBArray(camera, size=(320, 240))
time.sleep(0.1)

x_last = 160
y_last = 120

start_time = time.time()
counter = 0

from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)


# servo[0] -> left, 1 -> forward
# servo[1] -> right, -1 -> forward

def Motor_Steer(speed, steering, stop=False):
    global ccw
    if ccw:
        param = 1.0
    else:
        param = 0.8
    if stop == True:
        kit.continuous_servo[0].throttle = 0
        kit.continuous_servo[1].throttle = 0
        return
    elif steering == 0:
        kit.continuous_servo[0].throttle = speed
        kit.continuous_servo[1].throttle = -1 * speed*param
        return
    elif steering > 0:
        steering = 100 - steering
        kit.continuous_servo[0].throttle = speed
        kit.continuous_servo[1].throttle = -1 * speed * steering / 100*param
        return
    elif steering < 0:
        steering = steering * -1
        steering = 100 - steering
        kit.continuous_servo[0].throttle = speed * steering / 100
        kit.continuous_servo[1].throttle = -1 * speed*param
        return


def turn(ccw):
    if ccw:
        if address == 5 or 2:
            kit.continuous_servo[0].throttle = 1
            kit.continuous_servo[1].throttle = 1
            time.sleep(0.9)
        elif address == 1:
            kit.continuous_servo[0].throttle = -1
            kit.continuous_servo[1].throttle = -1
            time.sleep(1.0)
        else:
            kit.continuous_servo[0].throttle = 1
            kit.continuous_servo[1].throttle = 1
            time.sleep(1.15)
    else:
        kit.continuous_servo[0].throttle = -1
        kit.continuous_servo[1].throttle = -1
        time.sleep(1.12)


def change_dir(ccw):
    if ccw:
        ccw = False
    else:
        ccw = True
    return ccw


kp = 0.75  # off line
ap = 1.0  # off angle

# videoFile1 = './video001.avi'
# fourcc = cv2.VideoWriter_fourcc(*'DIVX')
# out = cv2.VideoWriter(videoFile1, fourcc, 9.0, (320,240))
address = 0
address0_time = 0
ang_list = []
stop_trigger = False
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    counter += 1
    image = frame.array
    # out.write(image)
    roi = image[60:239, 0:319]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    #yellow_lower = np.array([22, 60, 200], np.uint8)
    yellow_lower = np.array([22, 0, 150], np.uint8)
    yellow_upper = np.array([60, 255, 255], np.uint8)
    Blackline = cv2.inRange(hsv, yellow_lower, yellow_upper)
    # Blackline = cv2.inRange(image, (0, 0, 0), (75, 75, 75))

    red_lower = np.array([170, 120, 70], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)
    stopsign = cv2.inRange(hsv, red_lower, red_upper)
    
    kernel = np.ones((3, 3), np.uint8)
    #Blackline = cv2.erode(Blackline, kernel, iterations=3)
    #Blackline = cv2.dilate(Blackline, kernel, iterations=3)
    # stopsign = cv2.erode(stopsign, kernel, iterations=3)
    # stopsign = cv2.dilate(stopsign, kernel, iterations=5)
    contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, hierarchy_red = cv2.findContours(stopsign.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if not mmode_flag:
        if len(contours_red) > 0:
            print("stopsign: ", time.time() - start_time)
            stop_trigger = True
        if not (len(contours_red) > 0) and stop_trigger:
            print("address: LZ")
            address = 0
            stop_trigger = False

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

# address
        if not mmode_flag:
            if ccw:
                if address == 0 and ang > 40:
                    print("error: turn to ccw")
                    turn(not ccw)
                if address == 0 and ang < -60:
                    address = 101
                elif address == 101 and ang > -60:
                    print("address: 101")
                    address = 1
                    start = False
                elif address == 1 and ang > -10:
                    print("address: 102")
                    address = 2
                elif address == 2 and ang < -60:
                    print("address: 103")
                    address = 3
                elif address == 3 and ang > -30:
                    address = 34
                elif address == 34 and ang < -65:
                    print("address: 203")
                    address = 4
                elif address == 4 and ang > 20:
                    print("address: 202")
                    address = 5
                elif address == 5 and ang < -65:
                    print("address: 201")
                    address = 6
            else:
                if address == 0 and ang < -40:
                    print("error: turn to cw")
                    turn(not ccw)
                if address == 0 and ang > 65:
                    print("address: 201")
                    address = 6
                    start = False
                elif address == 6 and ang < -20:
                    print("address: 202")
                    address = 5
                elif address == 5 and ang > 65:
                    print("address: 203")
                    address = 4
                elif address == 4 and ang < 35:
                    address = 43
                elif address == 43 and ang > 65:
                    print("address: 103")
                    address = 3
                elif address == 3 and ang < 10:
                    print("address: 102")
                    address = 2
                elif address == 2 and ang > 65:
                    print("address: 101")
                    address = 1

        _, wh, _ = blackbox
        ww, hh = wh
        area = ww*hh
        #print(area)
        # stop
        if mmode_flag:
            Motor_Steer(0.4, (error * kp) + (ang * ap), True)
        elif area < 8500.0:
            Motor_Steer(0.4, (error * kp) + (ang * ap), True)
            print('obstacle: ', area)
        elif address == 0 and start and turn0:
            kit.continuous_servo[0].throttle = 1
            kit.continuous_servo[1].throttle = 1
            time.sleep(1.2)
            kit.continuous_servo[0].throttle = 0
            kit.continuous_servo[1].throttle = 0
            time.sleep(0.01)
            ccw = change_dir(ccw)
            turn0 = False
        elif address == 0 and stop0 and not start:
            Motor_Steer(0.4, (error * kp) + (ang * ap), True)
            if msg0:
                msg0 = False
        elif address == 1 and stop1:
            Motor_Steer(0.4, (error * kp) + (ang * ap), True)
            if msg1:
                print("After unloading, press [y]")
                msg1 = False
        elif address == 2 and stop2:
            Motor_Steer(0.4, (error * kp) + (ang * ap), True)
            if msg2:
                print("After unloading, press [y]")
                msg2 = False
        elif address == 3 and stop3:
            Motor_Steer(0.4, (error * kp) + (ang * ap), True)
            if msg3:
                print("After unloading, press [y]")
                msg3 = False
        elif address == 4 and stop4:
            Motor_Steer(0.4, (error * kp) + (ang * ap), True)
            if msg4:
                print("After unloading, press [y]")
                msg4 = False
        elif address == 5 and stop5:
            Motor_Steer(0.4, (error * kp) + (ang * ap), True)
            if msg5:
                print("After unloading, press [y]")
                msg5 = False
        elif address == 6 and stop6:
            Motor_Steer(0.4, (error * kp) + (ang * ap), True)
            if msg6:
                print("After unloading, press [y]")
                msg6 = False
        else:
            Motor_Steer(0.4, (error * kp) + (ang * ap))

        box = cv2.boxPoints(blackbox)
        box = np.int0(box)
    #cv2.drawContours(image, [box], 0, (0, 0, 255), 3)
        #cv2.putText(image, str(ang), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        #cv2.putText(image, str(error), (10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        #cv2.line(image, (int(x_min), 190), (int(x_min), 230), (255, 0, 0), 3)

    #cv2.drawContours(image, contours_blk, -1, (0,255,0), 3)

    ang_list += [[time.time() - start_time, ang]]
            
    cv2.imshow("original with line", image)
    #print(area)
    rawCapture.truncate(0)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("y"):
        if address == 0 and stop0 and not start:
            if turn0:
                turn(ccw)
                ccw = change_dir(ccw)
                turn0 = False
            stop0 = False
            start = True
        elif address == 1 and stop1:
            if turn1:
                turn(ccw)
                ccw = change_dir(ccw)
                turn1 = False
            stop1 = False
        elif address == 2 and stop2:
            if turn2:
                turn(ccw)
                ccw = change_dir(ccw)
                turn2 = False
            stop2 = False
        elif address == 3 and stop3:
            if turn3:
                turn(ccw)
                ccw = change_dir(ccw)
                turn3 = False
            stop3 = False
        elif address == 4 and stop4:
            if turn4:
                turn(ccw)
                ccw = change_dir(ccw)
                turn4 = False
            stop4 = False
        elif address == 5 and stop5:
            if turn5:
                turn(ccw)
                ccw = change_dir(ccw)
                turn5 = False
            stop5 = False
        elif address == 6 and stop6:
            if turn6:
                turn(ccw)
                ccw = change_dir(ccw)
                turn6 = False
            stop6 = False
    elif key == ord("z"):
        mmode_flag = True
        print("M-mode On")
    elif key == ord("x"):
        mmode_flag = False
        print("M-mode Off")
    elif key == ord("q"):
        kit.continuous_servo[0].throttle = 0
        kit.continuous_servo[1].throttle = 0
        break
    elif key == ord("t"):
        print('TIME: ', time.time() - start_time)

# out.release()
# cv2.destroyAllWindows()
finish_time = time.time()
fps = counter / (finish_time - start_time)
print("fps = " + str(fps))
ang_list = np.array(ang_list)
# np.save("./ang_list_ccw_p.npy", ang_list)

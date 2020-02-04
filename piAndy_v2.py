import threading as th
from queue import Queue

from socket import *
import pickle

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np


def HQ_client():
    def receive_command(sock):
        global command
        current_command = None
        while True:
            recvData = sock.recv(4096)
            command = pickle.loads(recvData)

            if current_command != command:
                print("received", command)
                current_command = command


    def send_status(sock):
        global direction, current_address, action, send_status_flag, send_status_flag_lock
        current_status = None
        while True:
            if send_status_flag:
                robot_status = makeRobotStatus(direction, current_address, action)

                sendData = pickle.dumps(robot_status, protocol=pickle.HIGHEST_PROTOCOL)
                sock.send(sendData)

                if current_status != robot_status:
                    print("send", robot_status)
                    current_status = robot_status

                send_status_flag_lock.acquire()
                send_status_flag = False
                send_status_flag_lock.release()
            else:
                time.sleep(0.2)

    def makeRobotStatus(direction, current_address, action):
        dic = {
            'direction': direction,
            'current_address': current_address,
            'action': action,
        }
        return dic

    # Connect to HQ
    HQ_ip = '128.237.218.91'
    HQ_port = 8090

    clientSock = socket(AF_INET, SOCK_STREAM)
    clientSock.connect((HQ_ip, HQ_port))

    print('connected')

    sender = th.Thread(target=send_status, args=(clientSock,))
    receiver = th.Thread(target=receive_command, args=(clientSock,))

    sender.start()
    receiver.start()


def follower():
    global send_status_flag, send_status_flag_lock
    global direction, current_address, action, send_status_flag, command
    global good_to_go_loading, good_to_go_unloading, get_drive, stop

    def change_flag(flag):
        if flag:
            flag = False
        else:
            flag = True
        return flag

    def Motor_Steer(speed, steering, stop=False):
        global ccw
        # servo[0] -> left, 1 -> forward
        # servo[1] -> right, -1 -> forward
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
            kit.continuous_servo[1].throttle = -1 * speed * param
            return
        elif steering > 0:
            steering = 100 - steering
            kit.continuous_servo[0].throttle = speed
            kit.continuous_servo[1].throttle = -1 * speed * steering / 100 * param
            return
        elif steering < 0:
            steering = steering * -1
            steering = 100 - steering
            kit.continuous_servo[0].throttle = speed * steering / 100
            kit.continuous_servo[1].throttle = -1 * speed * param
            return

    def turn(ccw):
        if ccw:
            if address == 0:
                kit.continuous_servo[0].throttle = 1
                kit.continuous_servo[1].throttle = 1
                time.sleep(1.15)
                kit.continuous_servo[0].throttle = 0
                kit.continuous_servo[1].throttle = 0
            elif address == 5 or address == 2:
                kit.continuous_servo[0].throttle = 1
                kit.continuous_servo[1].throttle = 1
                time.sleep(0.9)
            elif address == 1:
                kit.continuous_servo[0].throttle = -1
                kit.continuous_servo[1].throttle = -1
                time.sleep(0.9)
            elif address == 3:
                kit.continuous_servo[0].throttle = 1
                kit.continuous_servo[1].throttle = 1
                time.sleep(1.25)
            else:
                kit.continuous_servo[0].throttle = 1
                kit.continuous_servo[1].throttle = 1
                time.sleep(1.15)
        else:
            if address == 0:
                kit.continuous_servo[0].throttle = -1
                kit.continuous_servo[1].throttle = -1
                time.sleep(1.12)
                kit.continuous_servo[0].throttle = 0
                kit.continuous_servo[1].throttle = 0
            elif address == 4:
                kit.continuous_servo[0].throttle = -1
                kit.continuous_servo[1].throttle = -1
                time.sleep(1.2)
            elif address == 5:
                kit.continuous_servo[0].throttle = -1
                kit.continuous_servo[1].throttle = -1
                time.sleep(0.9)
            elif address == 6:
                kit.continuous_servo[0].throttle = -1
                kit.continuous_servo[1].throttle = -1
                time.sleep(1.3)
            else:
                kit.continuous_servo[0].throttle = -1
                kit.continuous_servo[1].throttle = -1
                time.sleep(1.12)


    class Address:
        def __init__(self, id, msg):
            self.id = id
            self.msg = msg

        def get_stop(self):
            global action, stop, get_drive, good_to_go_loading, good_to_go_unloading
            if self.id == operating_drive:
                if self.id == address:
                    stop = True
                    Motor_Steer(0.4, (error * kp) + (ang * ap), True)
                    if self.id == 0:
                        action = "loading"
                        if good_to_go_loading:
                            stop = False
                            get_drive = True
                            good_to_go_loading = False
                            print("Loading Confirm!!!", stop)
                    else:
                        action = "unloading"
                        if good_to_go_unloading:
                            stop = False
                            get_drive = True
                            good_to_go_unloading = False
                            print("Unloading Confirm!!!")

    address0 = Address(0, False)
    address1 = Address(1, False)
    address2 = Address(2, False)
    address3 = Address(3, False)
    address4 = Address(4, False)
    address5 = Address(5, False)
    address6 = Address(6, False)


    # Follower
    # Make flags
    operating_drive = None
    get_drive = True
    ccw = True
    mmode_flag = False
    stop = True

    current_path_id = None
    current_path = None
    # get_flag, ccw = make_TF(2, True)
    # start, good_to_go_loading, good_to_go_unloading, mmode_flag = make_TF(4, False)
    # # receive_command_flag = True
    # stop0 = True
    # stop1, stop2, stop3, stop4, stop5, stop6, msg0, msg1, msg2, msg3, msg4, msg5, msg6, turn0, turn1, turn2, turn3, turn4, turn5, turn6 = make_TF(20, False)
    stop_trigger = False
    # turn_flag = False
    # loading_block = False
    # operating_path = 0
    # loading_get_flag = False
    next_path = command['path']
    path_id = command['path_id']
    message = command['message']


    # Start PiCam
    camera = PiCamera()
    camera.resolution = (320, 240)
    rawCapture = PiRGBArray(camera, size=(320, 240))
    time.sleep(0.1)

    # Start Servo motors
    from adafruit_servokit import ServoKit
    kit = ServoKit(channels=16)

    # Initialize variables
    start_time = time.time()

    kp = 0.75  # off line
    ap = 1.0  # off angle

    x_last = 160
    y_last = 120

    # videoFile1 = './video001.avi'
    # fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    # out = cv2.VideoWriter(videoFile1, fourcc, 9.0, (320,240))
    address = 0
    road = 0
    address0_time = 0
    area_box = 10000.0
    ang_list = []
    dash_memory = np.zeros((2400, 320, 3))
    dash_block_flag = False

    start_time = time.time()
    counter = 0


    # Main Loop
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        counter += 1

        # Command handler
        path_id = command['path_id']
        if command['path'] is not None:  # and path_id != current_path_id:
            next_path = command['path']
            # current_path_id = path_id

        if command['message'] == 'loading_complete':
            good_to_go_loading = True
            print('get loading complete!!')
            command['message'] = None

        if command['message'] == 'unloading_complete':
            good_to_go_unloading = True
            print('get unloading complete!!')
            command['message'] = None


        if mmode_flag: # M-mode handler
            action = "M-mode"
            Motor_Steer(0.4, (error * kp) + (ang * ap), True)
            stop = True
        elif area_box < 5000.0:  # obstacle handler
            Motor_Steer(0.4, (error * kp) + (ang * ap), True)
            print('obstacle ahead: ', area_box)
            action = 'obstacle'
            stop = True
        else:
            stop = False


        # stop False handler
        address0.get_stop()
        address1.get_stop()
        address2.get_stop()
        address3.get_stop()
        address4.get_stop()
        address5.get_stop()
        address6.get_stop()


        # Path handler
        if get_drive:
            if path_id != current_path_id:
                current_path = list(next_path)
                current_path_id = np.copy(path_id)
            if len(current_path) > 0:
                operating_drive = current_path.pop(0)
                print("Next drive: ", operating_drive)
                print("stop: ", stop)
                get_drive = False
            else:
                operating_drive = 0
                get_drive = False

        # Image handler
        image = frame.array
        # out.write(image)
        roi = image[60:239, 0:319]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # yellow_lower = np.array([22, 60, 200], np.uint8)
        yellow_lower = np.array([22, 0, 150], np.uint8)
        yellow_upper = np.array([60, 255, 255], np.uint8)
        yellow_line = cv2.inRange(hsv, yellow_lower, yellow_upper)
        # yellow_line = cv2.inRange(image, (0, 0, 0), (75, 75, 75))

        red_lower = np.array([170, 120, 70], np.uint8)
        red_upper = np.array([180, 255, 255], np.uint8)
        stopsign = cv2.inRange(hsv, red_lower, red_upper)

        kernel = np.ones((3, 3), np.uint8)
        # yellow_line = cv2.erode(yellow_line, kernel, iterations=3)
        # yellow_line = cv2.dilate(yellow_line, kernel, iterations=3)
        # stopsign = cv2.erode(stopsign, kernel, iterations=3)
        # stopsign = cv2.dilate(stopsign, kernel, iterations=5)
        contours_blk, hierarchy_blk = cv2.findContours(yellow_line.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_red, hierarchy_red = cv2.findContours(stopsign.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Detect only main Yellow line, discard noise
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

            # Get shift and ang error to trace the line
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

        # Get Address, based on the ang
        if not mmode_flag:  # to avoid false detection
            if ccw:
                if address == 0 and ang > 40:
                    print("error: turn to ccw")
                    turn(ccw)
                elif address == 0 and ang < -60:
                    road = 101
                elif road == 101 and ang > -60:
                    print("address: 101")
                    address = 1
                    road = 0
                elif address == 1 and ang > -10:
                    print("address: 102")
                    address = 2
                elif address == 2 and ang < -60:
                    print("address: 103")
                    address = 3
                elif address == 3 and ang > -30:
                    road = 34
                elif road == 34 and ang < -65:
                    print("address: 203")
                    address = 4
                    road = 0
                elif address == 4 and ang > 20:
                    print("address: 202")
                    address = 5
                elif address == 5 and ang < -65:
                    print("address: 201")
                    address = 6
            else:
                if address == 0 and ang < -40:
                    print("error: turn to cw")
                    turn(ccw)
                elif address == 0 and ang > 60:
                    road = 201
                elif road == 201 and ang < 60:
                    print("address: 201")
                    address = 6
                    road = 0
                elif address == 6 and ang < -20:
                    print("address: 202")
                    address = 5
                elif address == 5 and ang > 65:
                    print("address: 203")
                    address = 4
                elif address == 4 and ang < 35:
                    road = 43
                elif road == 43 and ang > 65:
                    print("address: 103")
                    address = 3
                    road = 0
                elif address == 3 and ang < 10:
                    print("address: 102")
                    address = 2
                elif address == 2 and ang > 65:
                    print("address: 101")
                    address = 1

        # Stop sign handler
        if not mmode_flag:
            if len(contours_red) > 0:
                print("stopsign: ", time.time() - start_time)
                stop_trigger = True
            if not (len(contours_red) > 0) and stop_trigger:
                print("address: LZ")
                address = 0
                stop_trigger = False

        # Obstacle handler
        _, wh_box, _ = blackbox
        w_box, h_box = wh_box
        area_box = w_box * h_box

        # Stop handler
        if operating_drive == 9:
            print("turn!")
            turn(ccw)
            ccw = change_flag(ccw)
            get_drive = True
        else:
            address0.get_stop()
            address1.get_stop()
            address2.get_stop()
            address3.get_stop()
            address4.get_stop()
            address5.get_stop()
            address6.get_stop()

        if not stop:
            action = "moving"
            # print("moving!!")
            Motor_Steer(0.4, (error * kp) + (ang * ap))

        # Send robot status
        if counter % 5 == 0:
            send_status_flag_lock.acquire()
            send_status_flag = True
            send_status_flag_lock.release()
            if ccw:
                direction = 1
            else:
                direction = -1
            if current_address != address:
                current_address = address
            elif action != "loading" and action != "unloading":
                current_address = 999

        # Draw parameters on display
        box = cv2.boxPoints(blackbox)
        box = np.int0(box)
        # cv2.drawContours(image, [box], 0, (0, 0, 255), 3)
        # cv2.putText(image, str(ang), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        # cv2.putText(image, str(error), (10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        # cv2.line(image, (int(x_min), 190), (int(x_min), 230), (255, 0, 0), 3)

        # cv2.drawContours(image, contours_red, -1, (0,255,0), 3)

        cv2.imshow("original with line", image)
        rawCapture.truncate(0)

        # Ang error logger
        ang_list += [[time.time() - start_time, ang]]
        if counter % 10 == 0:
            dash_memory = dash_memory[0:2160]
            dash_memory = np.vstack((image, dash_memory))

        # Key Binding
        key = cv2.waitKey(1) & 0xFF
        # Manually Confirm
        if key == ord("y"):
            if action == "loading":
                good_to_go_loading = True
            elif action == "unloading":
                good_to_go_unloading = True
        # M-mode on/off
        elif key == ord("z"):
            mmode_flag = True
            print("M-mode On")
            if not dash_block_flag:
                np.save("./dash_cam/{}.npy".format(counter), dash_memory)
            dash_block_flag = True
        elif key == ord("x"):
            mmode_flag = False
            print("M-mode Off")
            dash_block_flag = False
        # Terminate process
        elif key == ord("q"):
            kit.continuous_servo[0].throttle = 0
            kit.continuous_servo[1].throttle = 0
            break

    # Save data
    # out.release()
    # cv2.destroyAllWindows()
    finish_time = time.time()
    fps = counter / (finish_time - start_time)
    print("fps = " + str(fps))
    ang_list = np.array(ang_list)
    # np.save("./ang_list_cw.npy", ang_list)



# Main Thread
# Dummy variables
direction = 1
current_address = 0
action = 'loading'
current_basket = {'r': 0, 'g': 0, 'b': 0}
operating_orderset = None
operating_order = {'address': 0, 'id':99999}

command = {
    'message': None,  # loading_complete / unloading_complete / None
    'path': (0,),  # path / None
    'path_id': 9999  # to ignore same path
}

ccw = True
good_to_go_loading = False
good_to_go_unloading = False
get_drive = False
stop = True

send_status_flag = False
send_status_flag_lock = th.Lock()

# Start threads
communicator = th.Thread(target=HQ_client, args=())
actor = th.Thread(target=follower, args=())

communicator.start()
actor.start()

from socket import *
import pickle
import threading as th
import time
import numpy as np
import robotstatus as rs
from datetime import datetime
import random


mid_ip = '128.237.114.101'



def send_robot_status(client):
    global direction, current_address, action, current_basket, operating_orderset, operating_order, next_orderset
    current_raw_status = None
    mmode_block = False
    mmode_start = None
    error_type = None
    dash_file_name = None
    while True:
        recvData = client.recv(16384)
        raw_status = pickle.loads(recvData)

        if raw_status['action'] == 'dash_file':
            dash_file_name = raw_status['dash_file_name']
            error_type = raw_status['error_type']
            continue
        else:
            robot_ping = time.time() - raw_status['ping']

        if raw_status != current_raw_status:
            # print("Raw status: ", raw_status, time.strftime('%c', time.localtime(time.time())))
            current_raw_status = np.copy(raw_status)

        direction = raw_status['direction']
        current_address = raw_status['current_address']
        action = raw_status['action']

        robot_status = rs.makeRobotStatus(direction, current_address, action, current_basket, operating_orderset,
                                          operating_order, next_orderset, robot_ping)

        if action == "M-mode" and not mmode_block:
            now = datetime.now()
            mmode_start = now.strftime("%Y-%m-%d %H:%M:%S")
            mmode_block = True
        if action != "M-mode" and mmode_block:
            now = datetime.now()
            m_mode = {
                'table_name': 'm_mode',
                'start_time': mmode_start,
                'end_time': now.strftime("%Y-%m-%d %H:%M:%S"),
                'error_type': error_type,
                'dash_file_name': dash_file_name
            }
            mmode_block = False
            print(m_mode)

            print("dash file send time: ", time.strftime('%c', time.localtime(time.time())))
            continue

        # print("status send: ", robot_status)


def receive_robot_command(client):
    global command, massage, operating_orderset, operating_order, operating_order_idx, operating_order_idx_lock, next_orderset
    global need_after_loading_job_flag, need_after_loading_job_flag_lock, need_after_unloading_job_flag, need_after_unloading_job_flag_lock
    global current_basket, action, next_orderset
    global direction, action
    wait_flag = False
    current_massage = None
    current_id = None
    orderset_block = False

    input_id = 1
    input_path = [0]
    input_msg = None
    i = 0
    temp = None
    time.sleep(3)
    while True:
        time.sleep(1)
        random_path = []
        if direction == 1:
            r1 = random.randint(1, 6)
            if r1 < 4:
                random_path += [r1]
                random_path += [9]
                if r1 == 3:
                    r2 = random.randint(1, 2)
                    random_path += [r2]
            else:
                random_path += [9]
                random_path += [r1]
                if r1 == 4:
                    r2 = random.randint(5, 6)
                    random_path += [9]
                    random_path += [r2]
                else:
                    random_path += [9]
            random_path += [0]
        if direction == -1:
            r1 = random.randint(1, 6)
            if r1 > 3:
                random_path += [r1]
                random_path += [9]
                if r1 == 4:
                    r2 = random.randint(5, 6)
                    random_path += [r2]
            else:
                random_path += [9]
                random_path += [r1]
                if r1 == 3:
                    r2 = random.randint(1, 2)
                    random_path += [9]
                    random_path += [r2]
                else:
                    random_path += [9]
            random_path += [0]

        input_path = random_path
        c1 = input_path.count(9)
        limit = len(input_path) - c1

        if input_id != temp:
            while i <= limit:
                temp = input_id
                print(i)
                if i == 0:
                    if action == 'loading':
                        input_msg = 'loading_complete'
                        i += 1
                        time.sleep(1)
                    else:
                        time.sleep(1)
                        continue
                elif i < limit:
                    if action == 'unloading':
                        time.sleep(0.7)
                        input_msg = 'unloading_complete'
                        i += 1
                        time.sleep(1)
                    else:
                        time.sleep(1)
                elif i == limit:
                    i += 1

                command = {
                    'message': input_msg,  # loading_complete / unloading_complete / None
                    'path': tuple(input_path),  # path / None
                    'path_id': input_id,  # to ignore same path
                    'ping': 0
                }
                time.sleep(1)
                command['ping'] = time.time()
                print("Command: ", command)
                sendData = pickle.dumps(command, protocol=3)
                client.send(sendData)
        else:
            time.sleep(0.1)
        i = 0
        print('!!!!!! input_id: ', input_id)
        input_id += 1


        # input_path = input("path: ")
        # input_id = int(input("path_id: "))
        # if input_id == 0:
        #     input_id = None
        # input_msg = input("msg: ")
        # if input_msg == 'l':
        #     input_msg = 'loading_complete'
        # elif input_msg == 'u':
        #     input_msg = 'unloading_complete'
        # else:
        #     input_msg = None
        #
        # command = {
        #     'message': input_msg,  # loading_complete / unloading_complete / None
        #     'path': tuple(map(int, input_path)),  # path / None
        #     'path_id': input_id,  # to ignore same path
        #     'ping': 0
        # }
        #
        # command['ping'] = time.time()
        # # print("Command: ", command)
        # sendData = pickle.dumps(command, protocol=3)
        #
        # client.send(sendData)


        # input_id += 1
        # # print("Message: ", massage, time.strftime('%c', time.localtime(time.time())))
        # if massage != current_massage:
        #     # print("Message: ", massage, time.strftime('%c', time.localtime(time.time())))
        #     current_massage = np.copy(massage)
        #
        # if massage['orderset'] is not None and not orderset_block:
        #     if massage['orderset']['id'] != current_id:
        #         next_orderset = massage['orderset']
        #         current_id = massage['orderset']['id']
        #     # print(1111, next_orderset)
        #
        # if action == 'loading' and (operating_order['id'] == 99999 or operating_order['id'] == 9999):
        #     if next_orderset is not None:
        #         operating_orderset = next_orderset
        #
        #         operating_order_idx_lock.acquire()
        #         operating_order_idx = 0  # reset idx
        #         operating_order_idx_lock.release()
        #
        #         operating_order = operating_orderset['dumporders'][operating_order_idx]  # get next order from orderset
        #
        #         operating_order_idx_lock.acquire()
        #         operating_order_idx += 1
        #         operating_order_idx_lock.release()
        #
        #         next_orderset = None
        #         # print(3333, next_orderset)
        #         if operating_order['address'] != 0:
        #             orderset_block = True
        #     else:
        #         operating_orderset = {'init': 'init', 'id': 99999999,
        #                               'dumporders': [{'id': 99999, 'partial': [], 'orderid': [999999], 'item': {'r': 0, 'g': 0, 'b': 0}, 'address': 0}],
        #                               'path': None, 'profit': None, 'item': {'r': 0, 'g': 0, 'b': 0}}
        #
        #         operating_order_idx_lock.acquire()
        #         operating_order_idx = 0  # reset idx
        #         operating_order_idx_lock.release()
        #
        #         current_basket = operating_orderset['item']  # update basket
        #         operating_order = operating_orderset['dumporders'][operating_order_idx]  # get next order from orderset
        # elif action != 'loading':
        #     orderset_block = False
        #     # print(4444, next_orderset)
        #
        # if action == 'unloading' and massage['massage'] == 'unloading_complete':
        #     if next_orderset is None:
        #         # update basket
        #         item = operating_order['item']
        #         current_basket['r'] -= item['r']
        #         current_basket['g'] -= item['g']
        #         current_basket['b'] -= item['b']
        #
        #         operating_order = operating_orderset['dumporders'][operating_order_idx]  # get next order from orderset
        #
        #         operating_order_idx_lock.acquire()
        #         operating_order_idx += 1
        #         operating_order_idx_lock.release()
        #     else:
        #         # print("?????????????????????????????????????????????????????")
        #         # update basket
        #         item = operating_order['item']
        #         current_basket['r'] -= item['r']
        #         current_basket['g'] -= item['g']
        #         current_basket['b'] -= item['b']
        #
        #         operating_orderset = next_orderset
        #
        #         operating_order_idx_lock.acquire()
        #         operating_order_idx = 0  # reset idx
        #         operating_order_idx_lock.release()
        #
        #         operating_order = operating_orderset['dumporders'][operating_order_idx]  # get next order from orderset
        #
        #         operating_order_idx_lock.acquire()
        #         operating_order_idx += 1
        #         operating_order_idx_lock.release()
        #
        #         next_orderset = None


        #         # print(2222, next_orderset)
        #
        # if massage['massage'] == 'loading_complete':
        #     current_basket = operating_orderset['item']  # update basket
        #
        #
        # # if massage['massage'] == 'unloading_complete':
        # #     # update basket
        # #     item = operating_order['item']
        # #     current_basket['r'] -= item['r']
        # #     current_basket['g'] -= item['g']
        # #     current_basket['b'] -= item['b']
        # #
        # #     operating_order = operating_orderset['dumporders'][operating_order_idx]  # get next order from orderset
        # #
        # #     operating_order_idx_lock.acquire()
        # #     operating_order_idx += 1
        # #     operating_order_idx_lock.release()
        #
        # if operating_orderset is not None:
        #     if operating_orderset['path'] == None:
        #         command['path'] = (0, )
        #     else:
        #         command['path'] = tuple(map(int, operating_orderset['path']))
        #     command['path_id'] = operating_orderset['id']
        #     command['message'] = massage['massage']
        #     if command['message'] == 'loading_complete' or command['message'] == 'unloading_complete':
        #         if action == "loading" or action == "unloading":
        #             wait_flag = True
        # if action != "loading" and action != "unloading":
        #     wait_flag = False
        # if wait_flag and command['message'] == None:
        #     continue



direction = 1
current_address = 0
action = 'loading'
current_basket = {'r': 0, 'g': 0, 'b': 0}
operating_orderset = {'init': 'init', 'id': 99999999,
                     'dumporders': [{'id': 99999, 'partial': [], 'orderid': [999999], 'item': {'r': 0, 'g': 0, 'b': 0}, 'address': 0}],
                     'path': None, 'profit': None, 'item': {'r': 0, 'g': 0, 'b': 0}}

operating_order = {'address': 0, 'id': 99999, 'item': {'r':0,'g':0,'b':0}, 'orderid':[999999]}
next_orderset = None

operating_order_idx = 0
operating_order_idx_lock = th.Lock()

need_after_loading_job_flag = False
need_after_loading_job_flag_lock = th.Lock()
need_after_unloading_job_flag = False
need_after_unloading_job_flag_lock = th.Lock()

massage = {
    'massage': None,  # loading_complete / unloading_complete / None
    'orderset': None  # 오더셋 넣기 / None
}

command = {
    'message': None,  # loading_complete / unloading_complete / None
    'path': (0, ),  # path / None
    'path_id': None,  # to ignore same path
    'ping': 0
}

# listen to client
middle_ip = mid_ip
middle_port = 8090

middleSock = socket(AF_INET, SOCK_STREAM)
middleSock.bind((middle_ip, middle_port))  # Middleware IP
middleSock.listen(5)

print('Waiting for client')

connectionSock, addr = middleSock.accept()

print(str(addr), 'connected as a client')

# start threads
t_send_robot_status = th.Thread(target=send_robot_status, args=(connectionSock, ))
t_receive = th.Thread(target=receive_robot_command, args=(connectionSock, ))

t_send_robot_status.start()
t_receive.start()

while True:
    # print(operating_order_idx)
    time.sleep(1)

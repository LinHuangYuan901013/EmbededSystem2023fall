

import socket

import numpy as np
import json
import time
import random
import matplotlib.pyplot as plt

HOST = '192.168.43.64' # IP address lab305 or ipconfig
PORT = 6531 # Port to listen on (use ports > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print("Starting server at: ", (HOST, PORT))
    conn, addr = s.accept()
    # print(conn, addr)
    with conn:
        print('Connected by', addr)
        while True:
            data = conn.recv(1024).decode('utf-8')
            print('Received from socket server : ', data)
            if (data.count('{') != 1):
                # Incomplete data are received.
                choose = 0
                buffer_data = data.split('}')
                while buffer_data[choose][0] != '{':
                    choose += 1
                data = buffer_data[choose] + '}'
            obj = json.loads(data)
            
            t = obj['s']
            plt.scatter(t, obj['x'], c='blue') # x, y, z, gx, gy, gz
            plt.scatter(t, obj['y'], c='red')  # x, y, z, gx, gy, gz
            plt.scatter(t, obj['z'], c='green')  # x, y, z, gx, gy, gz
            plt.xlabel("sample num")
            plt.pause(0.0001) 


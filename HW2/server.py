#!/usr/bin/env python3

import socket

import numpy as np
import json
import time
import random
import matplotlib.pyplot as plot
HOST = '192.168.0.13' # IP address lab305 or ipconfig
PORT = 6531 # Port to listen on (use ports > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print("Starting server at: ", (HOST, PORT))
    conn, addr = s.accept()
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
            plot.scatter(t, obj['x'], c='blue') # x, y, z, gx, gy, gz
            plot.scatter(t, obj['y'], c='red') # x, y, z, gx, gy, gz
            plot.scatter(t, obj['z'], c='green') # x, y, z, gx, gy, gz
            plot.xlabel("sample num")
            plot.pause(0.0001)
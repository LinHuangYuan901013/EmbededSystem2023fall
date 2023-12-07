# -*- coding: utf-8 -*-
import socket
HOST = '192.168.0.16'
PORT = 8000

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(10)
print("start server")
while True:
    conn, addr = server.accept()
    clientMessage = str(conn.recv(1024), encoding='utf-8')

    print('Client message is:', clientMessage)

    serverMessage = 'I\'m here!'
    conn.sendall(serverMessage.encode())
    conn.close()
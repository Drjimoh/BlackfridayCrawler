#!/usr/bin/python3

# Import the necessary Python modules
import socket
import csv 

client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
host_ip = '10.139.0.224' # paste your server ip address here
port = 9999
client_socket.connect((host_ip,port)) # a tuple

while (True):
	data = client_socket.recv(20)
	data = data.decode('utf-8')
	data = eval(data)
	X= float(data[0])
	Y= float(data[1])
	with open('Coordinates.csv', 'a', newline='') as csvfile:
		if (X >0) and (X<1) and (Y >0) and (Y<1):
			XY = csv.writer(csvfile)
			XY.writerow([str(X),str(Y)])
client_socket.close()

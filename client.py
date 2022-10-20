# lets make the client code
import socket,cv2, pickle,struct, csv

# create socket
client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
host_ip = '10.139.0.224' # paste your server ip address here
port = 9999
client_socket.connect((host_ip,port)) # a tuple
while (True):
	data = client_socket.recv(20)
	data = data.decode('utf-8')
	print(data)
	FPOGX=data[2:8]
	FPOGY=data[12:18]
	try:
		X = float(FPOGX)
		Y = float(FPOGY)
	except ValueError:
		X = 0.000
		Y = 0.000
	print("FPOGX IS ", FPOGX, "FPOGY IS ", FPOGY)

	print("x IS ", X, "Y IS ", Y)

	if X<0.5 and Y>0:
		print("Move Left") #instead of printing, we can send command to the robot to move left
	else:
		print("Move Right") #instead of printing, we can send command to the robot to move right

client_socket.close()

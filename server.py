# Welcome to PyShine

# This code is for the server 
# Lets import the libraries
import socket, cv2, pickle,struct,imutils, csv

# Socket Create
server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
host_name  = socket.gethostname()
host_ip = socket.gethostbyname(host_name)
print('HOST IP:',host_ip)
port = 9999
socket_address = (host_ip,port)

# Socket Bind
server_socket.bind(socket_address)

# Socket Listen
server_socket.listen(5)
print("LISTENING AT:",socket_address)

# Socket Accept
while True:
	client_socket,addr = server_socket.accept()
	print('GOT CONNECTION FROM:',addr)
	if client_socket:
		HOSTT = '127.0.0.1'
		# Gazepoint Port
		PORTT = 4242
		ADDRESS = (HOSTT, PORTT)

		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect(ADDRESS)

		s.send(str.encode('<SET ID="ENABLE_SEND_CURSOR" STATE="1" />\r\n'))
		s.send(str.encode('<SET ID="ENABLE_SEND_POG_FIX" STATE="1" />\r\n'))
		s.send(str.encode('<SET ID="ENABLE_SEND_DATA" STATE="1" />\r\n'))
		lisT=[]
		while 1:
			rxdat = s.recv(1024)    
			FPOGX=(bytes.decode(rxdat)[12:19])
			FPOGY=(bytes.decode(rxdat)[28:35])
			try:
				X = float(FPOGX)
				Y = float(FPOGY)
			except ValueError:
				X = 0.000
				Y = 0.000
			print(f"X is {X:.4f}", f"Y is {Y:.4f}")
			X = f"{X:.4f}"
			Y = f"{Y:.4f}"
			StrX=str(X)
			StrY=str(Y)
			a = [X,Y]
			a=str(a)
			a=a.encode()
			client_socket.sendall(a)

		s.close()
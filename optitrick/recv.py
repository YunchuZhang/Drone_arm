import time
import zmq
import json
import numpy as np
sub_port = 5556
context = zmq.Context()
#connect to socket we subscrib
socket_sub = context.socket(zmq.SUB)
#socket_sub.connect("tcp://localhost:5556")
socket_sub.connect("tcp://192.168.1.9:%d" %sub_port)
#socket_sub.setsockopt(zmq.SUBSCRIBE, b"")
socket_sub.setsockopt(zmq.SUBSCRIBE,b'')
def recv_array(socket, flags=0, copy=True, track=False):
    """recv a numpy array"""
    md = socket.recv_json(flags=flags)
    msg = socket.recv(flags=flags, copy=copy, track=track)
    buf = buffer(msg)
    A = np.frombuffer(buf, dtype=md['dtype'])
    return A.reshape(md['shape'])
while True:
    start = time.clock()
    print('asds')
    contents  =  recv_array(socket_sub,copy=False)
    print(contents)
    elapsed = (time.clock() - start)
    print("time",elapsed)    
    #nparr = np.asarray(bytearray(contents), dtype="uint8")




w = 0
o = 0
yy = -35
xm = 0
ym = 0
zm = 0
W = 0.4
L = 1
Rx = np.array([[1,0,0,0],[0,math.cos(w),-math.sin(w),0],[0,math.sin(w),math.cos(w),0],[0,0,0,1]])
Ry = np.array([[math.cos(o),0,math.sin(o),0],[0,1,0,0],[-math.sin(o),0,math.cos(0),0],[0,0,0,1]])
Rz = np.array([[math.cos(yy),-math.sin(yy),0,0],[math.sin(yy),math.cos(yy),0,0],[0,0,1,0],[0,0,0,1]])
Rxy = np.matmul(Rx, Ry)
Rxyz = np.matmul(Rxy, Rz)
Tm = np.matmul(Rxyz, np.array([[1,0,0,xm],[0,1,0,ym],[0,0,1,zm],[0,0,0,1]]))
print(Tm)

Trightback = np.matmul(Tm, np.array([[math.cos(math.pi/2),0,math.sin(math.pi/2),-L/2],[0,1,0,0],[-math.sin(math.pi/2),0,math.cos(math.pi/2),W/2],[0,0,0,1]]))
print(Trightback)
print(np.matmul(np.array([[0.5],[0],[-0.2]]),Trightback))


def set_pos_gain(msg_axis_id,pos_gain):
    data = db.encode_message('Set_Pos_Gain', {'Pos_Gain': pos_gain})
    msg = can.Message(arbitration_id=msg_axis_id << 5 | 0x1a, is_extended_id=False, data=data)
    try:
        bus.send(msg)
    except can.CanError:
        print("can_set_pos_gain NOT sent!")



def set_pos_gain(msg_axis_id, data=[], format='', RTR=False,pos_gain):
    data = db.encode_message('Set_Pos_Gain', {'Pos_Gain': pos_gain})
    msg = can.Message(arbitration_id=((msg_axis_id << 5) | 0x1a), data=data)
    msg.is_remote_frame = RTR
    msg.is_extended_id = False
    try:
        bus.send(msg)
    except can.CanError:
        print("set_pos_gain NOT sent!")


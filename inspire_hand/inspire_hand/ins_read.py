import serial
import time

regdict = {
    'ID' : 1000,
    'baudrate' : 1001,
    'clearErr' : 1004,
    'forceClb' : 1009,
    'angleSet' : 1486,
    'forceSet' : 1498,
    'speedSet' : 1522,
    'angleAct' : 1546,
    'forceAct' : 1582,
    'errCode' : 1606,
    'statusCode' : 1612,
    'temp' : 1618,
    'actionSeq' : 2320,
    'actionRun' : 2322
}
init_pos = [1000,1000,1000,1000,100,1000]

def openSerial(port,baudrate):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baudrate
    ser.open()
    return ser

def writeRegister(ser, id, add, num, val):
    bytes = [0xEB, 0x90]
    bytes.append(id) # id
    bytes.append(num + 3) # len
    bytes.append(0x12) # cmd
    bytes.append(add & 0xFF)
    bytes.append((add >> 8) & 0xFF) # add
    for i in range(num):
        bytes.append(val[i])
    checksum = 0x00
    for i in range(2, len(bytes)):
        checksum += bytes[i]
    checksum &= 0xFF
    bytes.append(checksum)
    ser.write(bytes)
    time.sleep(0.01)
    ser.read_all() # 把返回帧读掉，不处理

def readRegister(ser, id, add, num, mute=False):
    bytes = [0xEB, 0x90]
    bytes.append(id) # id
    bytes.append(0x04) # len
    bytes.append(0x11) # cmd
    bytes.append(add & 0xFF)
    bytes.append((add >> 8) & 0xFF) # add
    bytes.append(num)
    checksum = 0x00
    for i in range(2, len(bytes)):
        checksum += bytes[i]
    checksum &= 0xFF
    bytes.append(checksum)
    ser.write(bytes)
    time.sleep(0.01)
    recv = ser.read_all()
    if len(recv) == 0:
        return []
    num = (recv[3] & 0xFF) - 3
    val = []
    for i in range(num):
        val.append(recv[7 + i])
    if not mute:
        print('读到的寄存器值依次为：', end='')
        for i in range(num):
            print(val[i], end=' ')
        print()
    return val

def write_data_6(ser,id,_str,val):
    valid_commands = {'angleSet','forceSet','speedSet'}

    if _str in valid_commands:
        if len(val) != 6 or any(not (0 <= v <= 1000 or v == -1) for v in val):
            print('val 必须是长度为6的列表，值范围为 0~1000，允许使用 -1 作为占位符')
            return
        
        val_reg = [((v & 0xFF),(v >> 8) & 0xFF) for v in val]
        val_reg = [byte for pair in val_reg for byte in pair]
    
        writeRegister(ser, id, regdict[_str], 12, val_reg)

    else:
        print('Function call error')


def read_data_6(ser,id,_str):
    valid_data = {'angleSet', 'forceSet', 'speedSet', 'angleAct', 'forceAct'}
    valid_codes = {'errCode', 'statusCode', 'temp'}

    if _str in valid_data:
        val = readRegister(ser,id,regdict[_str],12,True)
        if len(val) < 12:
            print('no data.')
            return
        val_act = [(val[2*i] & 0xFF) + (val[1 + 2*i] << 8) for i in range(6)]
        return val_act
        # print(f"{_str} value: {' '.join(map(str, val_act))}")

    elif _str in valid_codes:
        val_act = readRegister(ser,id,regdict[_str],6,True)
        if len(val) < 6:
            print('no data.')
            return
        
        print('value: ',' '.join(map(str,val_act)))

    else:
        print('No data type.')

def forceClb(ser,id):

    writeRegister(ser,id,regdict['forceClb'],1,[1])
    time.sleep(15)

    return



if __name__ == '__main__':
    print('打开串口！')
    ser = openSerial('/dev/ttyUSB0',115200)
    # if ser:
    #     print('串口打开成功！')
    
    forceClb(ser,1)

        

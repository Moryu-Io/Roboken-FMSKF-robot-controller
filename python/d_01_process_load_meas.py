import serial
import struct
import time
import matplotlib.pyplot as plt
import numpy as np
import csv

SaveFolderPath = "./"
COMnum = "COM7"

CNT2US = 1/8
ID2PRCNAME = {
    0x10:"ADT_MAIN",
    0x11:"ADT_CAN2",
    0x12:"ADT_CAN3",
    0x20:"VDT_MAIN",
    0x21:"VDT_CAN_RX",
    0x22:"VDT_CAN_TX",
    0x30:"FDT_MAIN",
    0x40:"RMT_MAIN",
    0x50:"CGT_MAIN",
    0x60:"IMT_MAIN",
    0xE0:"LOG_MAIN",
    0xF0:"DBG_MAIN",
}

def start_test(ser:serial.Serial):
    ser.write(b't')
    time.sleep(1)
    ser.flushInput()
    ser.write(b'p')

def stop_test(ser:serial.Serial):
    ser.write(b't')
    time.sleep(1)
    ser.flushInput()
    ser.write(b'f')

def read_proc_struct(ser:serial.Serial):
    rxdata = []
    
    for i in range(6):
        rxdata.append(ser.read())
    
    PrcID    = struct.unpack('<B', rxdata[0])[0]
    PrcStart = struct.unpack('<B', rxdata[1])[0]
    PrcCnt   = struct.unpack('<L', b''.join(rxdata[2:]))[0]

    #print(rxdata)
    #print(f"{PrcID},{PrcStart},{PrcCnt}")

    return PrcID, PrcStart, PrcCnt


def sub_plot_timing_graph(pax, id, cnt_list, sts_list):
    cnt_np = np.array(cnt_list) * CNT2US    # micro us
    
    pax.step(cnt_np,sts_list,label=ID2PRCNAME[id], where='post')
    pax.fill_between(cnt_np, 0, sts_list, step="post")
    pax.set_yticks([0,1])
    pax.grid(True)
    pax.legend()

def plot_timing_graph(res_dic):
    id_num = len(res_dic.keys())
    fig, axes = plt.subplots(id_num,1, sharex="all")

    ax_idx = 0
    for id, v_list in res_dic.items():
        sub_plot_timing_graph(axes[ax_idx], id, v_list[0], v_list[1])
        ax_idx = ax_idx + 1

    axes[ax_idx-1].set_xlabel("time[us]")
    plt.show()


def main():
    ser = serial.Serial(COMnum, 460800, timeout=1)
    start_test(ser)

    start_time = time.time()    # タイムアウト処理用

    res_dic = {}
    try:
        while ((time.time() - start_time) <= 2.0):
            id, sts, cnt = read_proc_struct(ser)
            if id not in res_dic:
                res_dic[id] = [[cnt], [sts]]
            else:
                res_dic[id][0].append(cnt)
                res_dic[id][1].append(sts)

    except Exception as e:
        print(e)

    stop_test(ser)
    ser.close()

    res_dic_sorted = dict(sorted(res_dic.items()))
    plot_timing_graph(res_dic_sorted)


if __name__ == '__main__':
    main()

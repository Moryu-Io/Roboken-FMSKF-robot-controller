import matplotlib.pyplot as plt
from tkinter import filedialog
import csv

MOVE_TIME_MS = 10

class Vmotor_Log:
    def __init__(self, graphname) -> None:
        self.graphname = graphname
        self.tgt_vel = []
        self.now_vel = []
        self.now_cur = []
    
    def plot_graph(self, timelist, ax_l, ax_r):
        ax_l.plot(timelist,self.tgt_vel,color='b',label="TgtVel")
        ax_l.plot(timelist,self.now_vel,color='r',label="NowVel")
        ax_r.plot(timelist,self.now_cur,color='g',label="NowCurr")
        l_h, l_l = ax_l.get_legend_handles_labels()
        r_h, r_l = ax_r.get_legend_handles_labels()
        ax_l.legend(l_h+r_h, l_l+r_l)
        ax_l.grid(True, 'both')
        ax_l.set_title(self.graphname)
        ax_l.set_xlabel('Time[ms]')
        ax_l.set_ylabel('Vel[Rad/s]')
        ax_r.set_ylabel('Current[mA]')

class ArmJoint_Log:
    def __init__(self, graphname) -> None:
        self.graphname = graphname
        self.tgt_pos = []
        self.now_pos = []
    
    def plot_graph(self, timelist, ax_l, ax_r):
        ax_l.plot(timelist,self.tgt_pos, marker="*", color='b',label="TgtVel")
        ax_l.plot(timelist,self.now_pos, marker="*", color='r',label="NowPos")
        l_h, l_l = ax_l.get_legend_handles_labels()
        ax_l.legend(l_h, l_l)
        ax_l.grid(True, 'both')
        ax_l.set_title(self.graphname)
        ax_l.set_xlabel('Time[ms]')
        ax_l.set_ylabel('Pos[deg]')


# 結果や設定値格納用変数
TIMELIST_MS = []
M_FL = Vmotor_Log('FrontLeft')
M_BL = Vmotor_Log('BackLeft')
M_BR = Vmotor_Log('BackRight')
M_FR = Vmotor_Log('FrontRight')
J0 = ArmJoint_Log('J0_Yaw0')
J1 = ArmJoint_Log('J1_Pitch1')
J2 = ArmJoint_Log('J2_Pitch2')
J3 = ArmJoint_Log('J3_Roll0')
J4 = ArmJoint_Log('J4_Pitch3')

def csv_log_reader(filepath):
    csv_file = open(filepath, "r", encoding="utf-8")
    f = csv.reader(csv_file, delimiter=",", lineterminator="\n")

    for row in f:
        if "ADT" in row[0]:
            if ("PushCmdSeq" in row[0]) or ("Pos" in row[0]):
                pass
            else:
                TIMELIST_MS.append(int(row[0].replace("[ADT]", "")))
                J0.tgt_pos.append(float(row[1])/10)
                J0.now_pos.append(float(row[2])/10)
                J1.tgt_pos.append(float(row[3])/10)
                J1.now_pos.append(float(row[4])/10)
                J2.tgt_pos.append(float(row[5])/10)
                J2.now_pos.append(float(row[6])/10)
                J3.tgt_pos.append(float(row[7])/10)
                J3.now_pos.append(float(row[8])/10)
                J4.tgt_pos.append(float(row[9])/10)
                J4.now_pos.append(float(row[10])/10)
        elif "VDT"  in row[0]:
            M_FL.tgt_vel.append(int(row[1]))
            M_FL.now_vel.append(int(row[2]))
            M_FL.now_cur.append(int(row[3]))
            M_BL.tgt_vel.append(int(row[4]))
            M_BL.now_vel.append(int(row[5]))
            M_BL.now_cur.append(int(row[6]))
            M_BR.tgt_vel.append(int(row[7]))
            M_BR.now_vel.append(int(row[8]))
            M_BR.now_cur.append(int(row[9]))
            M_FR.tgt_vel.append(int(row[10]))
            M_FR.now_vel.append(int(row[11]))
            M_FR.now_cur.append(int(row[12]))
        else:
            pass


def main():
    typ = [('ログファイル','*.*')] 
    fle = filedialog.askopenfilename(filetypes = typ)

    csv_log_reader(fle)

    # グラフ描画
    timelist = [MOVE_TIME_MS*x for x in range(len(J0.tgt_pos))]

    # 足回り描画
    if len(TIMELIST_MS) != len(M_FL.tgt_vel):
        timelist_v = TIMELIST_MS[0:(len(M_FL.tgt_vel)-1)]
    else:
        timelist_v = TIMELIST_MS

    fig = plt.figure()
    fl_ax_l = fig.add_subplot(411)
    bl_ax_l = fig.add_subplot(412, sharex=fl_ax_l)
    br_ax_l = fig.add_subplot(413, sharex=fl_ax_l)
    fr_ax_l = fig.add_subplot(414, sharex=fl_ax_l)
    fl_ax_r = fl_ax_l.twinx()
    bl_ax_r = bl_ax_l.twinx()
    br_ax_r = br_ax_l.twinx()
    fr_ax_r = fr_ax_l.twinx()
    M_FL.plot_graph(timelist_v, fl_ax_l, fl_ax_r)
    M_BL.plot_graph(timelist_v, bl_ax_l, bl_ax_r)
    M_BR.plot_graph(timelist_v, br_ax_l, br_ax_r)
    M_FR.plot_graph(timelist_v, fr_ax_l, fr_ax_r)

    # 腕関節描画
    fig = plt.figure()
    j0_ax_l = fig.add_subplot(511)
    j1_ax_l = fig.add_subplot(512, sharex=j0_ax_l)
    j2_ax_l = fig.add_subplot(513, sharex=j0_ax_l)
    j3_ax_l = fig.add_subplot(514, sharex=j0_ax_l)
    j4_ax_l = fig.add_subplot(515, sharex=j0_ax_l)
    j0_ax_r = j0_ax_l.twinx()
    j1_ax_r = j1_ax_l.twinx()
    j2_ax_r = j2_ax_l.twinx()
    j3_ax_r = j3_ax_l.twinx()
    j4_ax_r = j4_ax_l.twinx()
    J0.plot_graph(TIMELIST_MS, j0_ax_l, j0_ax_r)
    J1.plot_graph(TIMELIST_MS, j1_ax_l, j1_ax_r)
    J2.plot_graph(TIMELIST_MS, j2_ax_l, j2_ax_r)
    J3.plot_graph(TIMELIST_MS, j3_ax_l, j3_ax_r)
    J4.plot_graph(TIMELIST_MS, j4_ax_l, j4_ax_r)

    plt.show()


if __name__ == '__main__':
    main()
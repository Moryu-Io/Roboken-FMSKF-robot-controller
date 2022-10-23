import matplotlib.pyplot as plt
from tkinter import filedialog
import csv

MOVE_TIME_MS = 10

class MT_Log:
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


# 結果や設定値格納用変数
M_FL = MT_Log('FrontLeft')
M_BL = MT_Log('BackLeft')
M_BR = MT_Log('BackRight')
M_FR = MT_Log('FrontRight')

def csv_log_reader(filepath):
    csv_file = open(filepath, "r", encoding="utf-8")
    f = csv.reader(csv_file, delimiter=",", lineterminator="\n")
    for row in f:
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


def main():
    typ = [('ログファイル','*.*')] 
    fle = filedialog.askopenfilename(filetypes = typ)

    csv_log_reader(fle)

    # グラフ描画
    timelist = [MOVE_TIME_MS*x for x in range(len(M_FL.tgt_vel))]

    fig = plt.figure()
    fl_ax_l = fig.add_subplot(411)
    bl_ax_l = fig.add_subplot(412, sharex=fl_ax_l)
    br_ax_l = fig.add_subplot(413, sharex=fl_ax_l)
    fr_ax_l = fig.add_subplot(414, sharex=fl_ax_l)
    fl_ax_r = fl_ax_l.twinx()
    bl_ax_r = bl_ax_l.twinx()
    br_ax_r = br_ax_l.twinx()
    fr_ax_r = fr_ax_l.twinx()
    M_FL.plot_graph(timelist, fl_ax_l, fl_ax_r)
    M_BL.plot_graph(timelist, bl_ax_l, bl_ax_r)
    M_BR.plot_graph(timelist, br_ax_l, br_ax_r)
    M_FR.plot_graph(timelist, fr_ax_l, fr_ax_r)

    plt.show()


if __name__ == '__main__':
    main()
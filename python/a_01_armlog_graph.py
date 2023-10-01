import matplotlib.pyplot as plt
from tkinter import filedialog
import csv

MOVE_TIME_MS = 10

class MT_Log:
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
J0 = MT_Log('J0_Yaw0')
J1 = MT_Log('J1_Pitch1')
J2 = MT_Log('J2_Pitch2')
J3 = MT_Log('J3_Roll0')
J4 = MT_Log('J4_Pitch3')

def csv_log_reader(filepath):
    csv_file = open(filepath, "r", encoding="utf-8")
    f = csv.reader(csv_file, delimiter=",", lineterminator="\n")
    for row in f:
        TIMELIST_MS.append(int(row[0]))
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


def main():
    typ = [('ログファイル','*.*')] 
    fle = filedialog.askopenfilename(filetypes = typ)

    csv_log_reader(fle)

    # グラフ描画
    timelist = [MOVE_TIME_MS*x for x in range(len(J0.tgt_pos))]

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
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
        ax_l.plot(timelist,self.tgt_pos,color='b',label="TgtVel")
        ax_l.plot(timelist,self.now_pos,color='r',label="NowPos")
        l_h, l_l = ax_l.get_legend_handles_labels()
        ax_l.legend(l_h, l_l)
        ax_l.grid(True, 'both')
        ax_l.set_title(self.graphname)
        ax_l.set_xlabel('Time[ms]')
        ax_l.set_ylabel('Pos[deg]')


# 結果や設定値格納用変数
J0 = MT_Log('J0_Yaw0')
J1 = MT_Log('J1_Pitch1')
J2 = MT_Log('J2_Pitch2')
J3 = MT_Log('J3_Roll0')
J4 = MT_Log('J4_Pitch3')

def csv_log_reader(filepath):
    csv_file = open(filepath, "r", encoding="utf-8")
    f = csv.reader(csv_file, delimiter=",", lineterminator="\n")
    for row in f:
        J0.tgt_pos.append(int(row[1]))
        J0.now_pos.append(int(row[2]))
        J1.tgt_pos.append(int(row[3]))
        J1.now_pos.append(int(row[4]))
        J2.tgt_pos.append(int(row[5]))
        J2.now_pos.append(int(row[6]))
        J3.tgt_pos.append(int(row[7]))
        J3.now_pos.append(int(row[8]))
        J4.tgt_pos.append(int(row[9]))
        J4.now_pos.append(int(row[10]))


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
    J0.plot_graph(timelist, j0_ax_l, j0_ax_r)
    J1.plot_graph(timelist, j1_ax_l, j1_ax_r)
    J2.plot_graph(timelist, j2_ax_l, j2_ax_r)
    J3.plot_graph(timelist, j3_ax_l, j3_ax_r)
    J4.plot_graph(timelist, j4_ax_l, j4_ax_r)

    plt.show()


if __name__ == '__main__':
    main()
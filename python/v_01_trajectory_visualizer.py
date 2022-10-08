import numpy as np
import matplotlib.pyplot as plt


V0 = 100    # 初期速度[mm/s]
VE = 200    # 終端速度[mm/s]
A0 = 500    # 初期加速度[mm/s2]
AM = 1000   # 最大加速度[mm/s2]
JK = 50000  # 躍度[mm/s3]
T  = 1      # 加速時間[s]
Tend = 2    # 演算終了時刻[s]
dT = 0.001  # 制御周期[s]

dt1 = (AM - A0) / JK   # 加加速時間
dt3 = AM / JK          # 減加速時間
dt2 = 1/AM * (VE - V0 - A0*dt1/2 - AM*(dt1+dt3)/2)    # 等加速時間

if dt2 < 0:
    pass
else:
    t1arr = np.arange(0, dt1+dT, dT)
    t2arr = np.arange(t1arr[-1], t1arr[-1]+dt2+dT, dT)
    t3arr = np.arange(t2arr[-1], t2arr[-1]+dt3+dT, dT)
    timearr = np.concatenate([t1arr, t2arr, t3arr])

    V1arr = V0 + (2*A0 + JK*t1arr)*t1arr / 2
    V2arr = V1arr[-1] + AM * (t2arr - t1arr[-1])
    V3arr = V2arr[-1] + (2*AM - JK*(t3arr - t2arr[-1]))*(t3arr - t2arr[-1]) / 2
    Varr = np.concatenate([V1arr, V2arr, V3arr])

    plt.plot(timearr, Varr)
    plt.show()
    plt.pause(0.1)
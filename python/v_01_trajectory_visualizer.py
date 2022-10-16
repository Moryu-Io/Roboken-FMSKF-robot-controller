from cProfile import label
import numpy as np
import matplotlib.pyplot as plt


V0 = 0    # 初期速度[mm/s]
VE = 100    # 終端速度[mm/s]
A0 = 0    # 初期加速度[mm/s2]
AM = 500   # 最大加速度[mm/s2] (絶対値)
JK = 10000  # 躍度[mm/s3] (絶対値)

dT = 0.001  # 制御周期[s]

# 方向補整
if VE - V0 < 0:
    AM = -AM
    JK = -JK

dt1 = (AM - A0) / JK   # 加加速時間
dt3 = AM / JK          # 減加速時間
dt2 = 1/AM * (VE - V0 - A0*dt1/2 - AM*(dt1+dt3)/2)    # 等加速時間

if dt2 < 0:
    # 等加速区間がない場合
    dt1 = np.sqrt(((A0/JK)**2)/2 + (VE-V0)/JK) - A0/JK
    a1 = (A0+JK*dt1)
    dt3 = a1/JK
    t1arr = np.arange(0, dt1+dT, dT)
    t3arr = np.arange(t1arr[-1], t1arr[-1]+dt3+dT, dT)

    V1arr = V0 + (2*A0 + JK*t1arr)*t1arr / 2
    V3arr = V1arr[-1] + (2*a1 - JK*(t3arr - t1arr[-1]))*(t3arr - t1arr[-1]) / 2

    plt.plot(t1arr, V1arr, color="r", label="V1")
    plt.plot(t3arr, V3arr, color="b", label="V3")
    plt.grid(True)
    plt.show()
else:
    # 等加速区間がある場合
    t1arr = np.arange(0, dt1+dT, dT)
    t2arr = np.arange(t1arr[-1], t1arr[-1]+dt2+dT, dT)
    t3arr = np.arange(t2arr[-1], t2arr[-1]+dt3+dT, dT)
    timearr = np.concatenate([t1arr, t2arr, t3arr])

    V1arr = V0 + (2*A0 + JK*t1arr)*t1arr / 2
    V2arr = V1arr[-1] + AM * (t2arr - t1arr[-1])
    V3arr = V2arr[-1] + (2*AM - JK*(t3arr - t2arr[-1]))*(t3arr - t2arr[-1]) / 2
    Varr = np.concatenate([V1arr, V2arr, V3arr])

    plt.plot(t1arr, V1arr, color="r", label="V1")
    plt.plot(t2arr, V2arr, color="g", label="V2")
    plt.plot(t3arr, V3arr, color="b", label="V3")
    plt.grid(True)
    plt.show()

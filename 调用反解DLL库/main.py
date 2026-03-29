from ctypes import *
import ctypes
import time
from ctypes import CDLL


# ----------以下四种加载DLL方式皆可-----------------
# pRSS6RBT_Inverse = WinDLL("./RSS6RBT_InverseDLL.dll")
# pRSS6RBT_Inverse = windll.LoadLibrary("./RSS6RBT_InverseDLL.dll")
# pRSS6RBT_Inverse = cdll.LoadLibrary("./RSS6RBT_InverseDLL.dll")

try:
    pRSS6RBT_Inverse = CDLL('./RSS6RBT_InverseDLL.dll')
    # 调用函数等操作
except OSError as e:
    print("Error loading DLL:", e)
    pRSS6RBT_Inverse = CDLL("./RSS6RBT_InverseDLL.dll")

# File = open('./test.txt', 'a')
# File.write("XP    YP    ZP    YAW    PITCH    ROLL    M1    M2    M3    M4    M5    M6\n")

pInversResult = (c_double*6)()
I =0
J =0

XP = 0
YP = 0
ZP =
YAW = 0
PITCH = 0
ROLL = 0

ret = pRSS6RBT_Inverse.InK6RSS(c_double(XP), c_double(YP), c_double(ZP), c_double(YAW), c_double(PITCH), c_double(ROLL), pointer(pInversResult))
print(ret)
print(pInversResult[0], pInversResult[1], pInversResult[2], pInversResult[3], pInversResult[4], pInversResult[5])

# T0 = time.time()
# for XP in range(-100, 100, 5) :
#     for YP in range(-100, 100, 5) :
#         for ZP in range(168, 228, 5) :
#             for YAW in range(-50, 50, 5) :
#                 for PITCH in range(-30, 30, 5) :
#                     for ROLL in range(-30, 30, 5) :
#                         I=I+1
#                         ret = pRSS6RBT_Inverse.InK6RSS(c_double(XP), c_double(YP), c_double(ZP), c_double(YAW), c_double(PITCH), c_double(ROLL), pointer(pInversResult))
#                         if  (I%100000)==0 :
#                             print("第", I, "次, 经过了: ", time.time()-T0, "s")
#                         if  (pInversResult[0] == -360) or \
#                             (pInversResult[1] == -360) or \
#                             (pInversResult[2] == -360) or \
#                             (pInversResult[3] == -360) or \
#                             (pInversResult[4] == -360) or \
#                             (pInversResult[5] == -360) or \
#                             (ret == 0 )                     :
#                             # print("NAN")
#                             J = J+1
#                         else:
#                             # print(pInversResult[0], pInversResult[1], pInversResult[2], pInversResult[3], pInversResult[4], pInversResult[5])
#                             File.write(str(XP))
#                             File.write("    ")
#                             File.write(str(YP))
#                             File.write("    ")
#                             File.write(str(ZP))
#                             File.write("    ")
#                             File.write(str(YAW))
#                             File.write("    ")
#                             File.write(str(PITCH))
#                             File.write("    ")
#                             File.write(str(ROLL))
#                             File.write("    ")
#                             File.write(str(pInversResult[0]))
#                             File.write("    ")
#                             File.write(str(pInversResult[1]))
#                             File.write("    ")
#                             File.write(str(pInversResult[2]))
#                             File.write("    ")
#                             File.write(str(pInversResult[3]))
#                             File.write("    ")
#                             File.write(str(pInversResult[4]))
#                             File.write("    ")
#                             File.write(str(pInversResult[5]))
#                             File.write("\n")



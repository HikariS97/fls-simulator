import numpy as np
import scipy.io as sio
from scipy.spatial.transform import Rotation as R
from einops import rearrange

def computeLUT(rmin, rmax, angle_table, res, img):
    # 得到 polar 图像的数据
    nbins = img.shape[0]
    nbeams = img.shape[1]
    origin_res = (rmax-rmin) / nbins
    R = rmin + (np.linspace(0, (rmax-rmin), nbins+1) + origin_res/2)[:-1]
    rightmost_angle = angle_table.ar[-1]
    leftmost_angle = angle_table.al[0]

    # 打印选择（默认）的分辨率
    if res is None:
        res = origin_res
        print('use default res: {} meter/pixel'.format(res))
    else: print('use specified res: {} meter/pixel'.format(res))

    # carte 图像的长
    rows = np.round(rmax / res).astype(np.int32)
    xres = rmax/rows
    print('xres: {}'.format(xres))
    rows += 1  # 包含零， 以表示坐标原点，但 xres 并未改变。

    # carte 图像的宽
    y_meter = rmax*(np.sin(rightmost_angle) - np.sin(leftmost_angle))
    cols = np.round(y_meter/res).astype(np.int32)
    yres = y_meter / cols
    print('yres: {}'.format(yres))
    cols += 1  # 包含零， 以表示坐标原点，但 yres 并未改变。

    # 创建 look-up-table
    LUT = np.zeros((rows, cols, 2))

    # 定义取索引函数
    def findR(R, R_req):
        return np.argmin(np.abs(R - R_req))
    def findA(table, angle):
        return int(np.where(table.ar[table.al < angle] > angle)[0])

    # bins 方向上的计数子
    lowest_hit = rows

    # 主循环
    for row in range(rows):
        for col in range(cols):
            y = (col - (cols-1)/2) * yres  # in meter
            x = row * xres
            R_req = np.sqrt(x**2 + y**2)
            angle = np.arctan2(y, x)  # NOTE: 注意靠左侧的角度为负值

            if R_req < rmin or R_req > rmax or angle < leftmost_angle or angle > rightmost_angle:
                continue
            else:
                R_idx = findR(R, R_req)
                A_idx = findA(angle_table, angle)
                LUT[row, col, :] = [R_idx, A_idx]

                # 记录达到的 row， row 不可能为 0
                lowest_hit = np.min([row, lowest_hit])

    return np.flip(LUT[lowest_hit:, :, :].astype(np.int32), (0, 1))

def remap(img, LUT, background=0):
    img[0, 0] = background
    return img[LUT[:, :, 0], LUT[:, :, 1]]

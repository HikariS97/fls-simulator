import sys
import time
import cv2 as cv
import OpenEXR
import os
import bpy
import numpy as np
from pathlib import Path
from einops import rearrange
import scipy.io as sio
from scipy.spatial.transform import Rotation as R

curr_py_path = Path(__file__)
sys.path.append(str(curr_py_path.parent))

from angle_table import angle_table


# Parameters
near_bound = 0.7
far_bound = 4.7
bearing_fov = np.deg2rad(angle_table.ac.max()-angle_table.ac.min())
vertical_fov = np.deg2rad(14.0)
nbeams = 96  # ARIS 1800 High Freq
distance_resolution = 0.006
nbins = int(np.round((far_bound-near_bound) / distance_resolution)) + 1
real_res = (far_bound-near_bound)/(nbins-1)
print("The real range resolution of sonar image is {}".format(real_res))
frames_num = 50

# 路径
base_path = "D:\\repos\\Sonar-simulator-blender\\Blender2.90\\"
rendred_path = base_path + "rendered\\"
sonar_path = base_path + "fls\\"


# count time start
print(">> Program Starts")
time_start = time.time()

def renderOnce():
    # 使用 Blender 的 GUI 设置作为主题，不需要额外设置节点等内容
    bpy.ops.render.render(write_still=True)


def imageGeneration(num):
    # 文件名
    rgb_filename = rendred_path + "RawImage0000.exr"
    pos_filename = rendred_path + "RawPosition0000.exr"

    # 读取 exr 函数
    def fromstr(s):
        mat = np.fromstring(s, dtype=np.float32)
        mat = mat.reshape (height,width)
        return mat

    ### RGB ###
    exrimage = OpenEXR.InputFile(rgb_filename)
    dw = exrimage.header()['dataWindow']
    (width, height) = (dw.max.x - dw.min.x + 1, dw.max.y - dw.min.y + 1)
    (r, g, b) = [fromstr(s) for s in exrimage.channels('RGB')]

    intensity_img = np.array(r)
    intensity_img = intensity_img/np.amax(intensity_img)
    intensity_img = rearrange(intensity_img, 'h w -> (h w)')

    ### POSITION ###
    exrimage = OpenEXR.InputFile(pos_filename)
    dw = exrimage.header()['dataWindow']
    (width, height) = (dw.max.x - dw.min.x + 1, dw.max.y - dw.min.y + 1)
    (r, g, b) = [fromstr(s) for s in exrimage.channels('RGB')]
    pos_img = np.stack([r,g,b], 0)

    ### Camera Pos and Pos in Camera ###
    w2c_t = np.array(bpy.data.objects['Camera'].location).reshape(-1, 1)  # (3,1)
    w2c_R = R.from_euler('xyz',np.array(bpy.data.objects['Camera'].rotation_euler)).as_matrix()  # (3, WH)
    pos_c = w2c_R.transpose() @ (rearrange(pos_img, 'pos h w -> pos (h w)') - w2c_t)

    # dist image
    dist_img = np.linalg.norm(pos_c, axis=0, ord=2)
    dist_img = rearrange(dist_img, '(h w) -> h w', h=height, w=width)
    cv.imwrite(rendred_path+"dist.png", dist_img/np.amax(dist_img)*255)

    ##################
    ### FORM POLAR ###
    ##################
    print(">> Start Sonar Image Generation")
    def getEle(p):  # p(0)->x, p(1)->y, p(2)->z
        return np.arctan2(p[1], np.sqrt(p[0]**2 + (-p[2])**2))
    def getAzi(p):  # p(0)->x, p(1)->y, p(2)->z
        return np.arctan2(p[0], -p[2])

    polar = np.zeros([nbins, nbeams])
    counter = np.zeros([nbins, nbeams])
    for i in range(pos_c.shape[1]):
        # forward -> -z, right -> x, upward -> y
        p = pos_c[:, i]
        phi = getEle(p)
        theta = getAzi(p)
        r = np.linalg.norm(p)

        # out of bound
        if phi > vertical_fov/2 or phi < -vertical_fov/2 or \
            theta > angle_table.ar.max() or theta < angle_table.al.min() or \
                r > far_bound or r < near_bound:
                    continue

        # find beam and bin number
        bin_i = int(np.round((r-near_bound)/distance_resolution))
        beam_i = int(np.where(angle_table.ar[angle_table.al < theta] > theta)[0])

        polar[bin_i, beam_i] += intensity_img[i]
        counter[bin_i, beam_i] += 1

    # 暂时先考虑平均
    counter[np.where(counter==0)] = 1
    polar /= np.amax(polar)
    polar /= counter
    polar /= np.amax(polar)

    ##################
    ### 打印图片信息 ###
    ##################
    print(">> Pics Infos")
    print('max distances:', np.amax(dist_img))
    print('min distances:', np.amin(dist_img))
    print('polar image size: {}x{}'.format(nbeams, nbins))

    # 保存
    raw_save_path = sonar_path + "polar" + str(num) + ".png"
    cv.imwrite(raw_save_path, polar*255)
    cam_xyz = np.array(bpy.data.objects['Camera'].location)
    cam_rpy = np.array(bpy.data.objects['Camera'].rotation_euler)
    pose_save_path = sonar_path + "pose" + str(num)+ ".mat"
    sio.savemat(pose_save_path, {'cam_xyz':cam_xyz, 'cam_rpy':cam_rpy})


#################
### MAIN LOOP ###
#################
for i in range(1):
    renderOnce()
    imageGeneration(i)

# calculate time cost
time_end = time.time()
print('Total cost', time_end - time_start)

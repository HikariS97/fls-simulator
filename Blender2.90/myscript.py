import sys
import time
import cv2 as cv
import OpenEXR
import os
import bpy
import numpy as np
import scipy.io as sio
from scipy.spatial.transform import Rotation as R

# Parameters
near_bound = 0.7
far_bound = 4.7
bearing_fov = 28.0
vertical_fov = 14.0
distance_resolution = 0.006
length = int(np.floor((far_bound-near_bound) / distance_resolution))
real_res = (far_bound-near_bound)/length
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

    intensity_img = r
    rgb_array = np.array(intensity_img)
    rgb_array = rgb_array/np.amax(rgb_array)

    ### POSITION ###
    exrimage = OpenEXR.InputFile(pos_filename)
    dw = exrimage.header()['dataWindow']
    (width, height) = (dw.max.x - dw.min.x + 1, dw.max.y - dw.min.y + 1)
    (r, g, b) = [fromstr(s) for s in exrimage.channels('RGB')]
    pos_img = np.stack([r,g,b], -1)

    ### CONVERT POSITION TO DISTANCE ###
    matrix_world = np.array(bpy.data.objects['Camera'].matrix_world)
    camera_xyz = matrix_world[:-1, -1]
    dist_img = np.linalg.norm(pos_img - camera_xyz, axis=2, ord=2)

    ### 中间结果测试
    cv.imwrite(rendred_path+"dist.png", dist_img/np.amax(dist_img)*255)

    # 赋值
    color_array = np.array(intensity_img)
    dist_array = np.array(dist_img)
    print("The max value of exr-rgb image is {}".format(color_array.max()))

    ##################
    ### 打印图片信息 ###
    ##################
    print(">> Pics Infos")
    print('width', width)
    print('height', height)

    max_distance = np.amax(dist_array)
    min_distance = np.amin(dist_array)
    print('max distances:', max_distance)
    print('min distances:', min_distance)

    print('the length of picture:', length)


    ##################
    ### 生成声呐图片 ###
    ##################
    print("Start Sonar Image Generation")

    raw_array = np.zeros((length, width))
    counter = np.zeros((length, width))

    # 累加声强度
    for j in range(width):
        for i in range(height):
            # 计算距离所代表图片中位置
            # if dist_array[i, j] > far_bound or dist_array[i, j] < near_bound:
            #     continue
            t = int(np.floor(((dist_array[i, j])-near_bound) / distance_resolution))
            if t >= length or t < 0:
                continue
            raw_array[t, j] = color_array[i, j] + raw_array[t, j]
            counter[t, j] = counter[t, j] + 1

    # 暂时先考虑平均
    counter[np.where(counter==0)] = 1
    raw_array /= np.amax(raw_array)
    raw_avg = raw_array / counter
    raw_avg /= np.amax(raw_avg)


    # 保存
    raw_save_path = sonar_path + "raw" + str(num) + ".png"
    cv.imwrite(raw_save_path, raw_array*255)
    raw_avg_save_path = sonar_path + "raw_avg" + str(num) + ".png"
    cv.imwrite(raw_avg_save_path, raw_avg*255)


    # 保存
    camera_xyz = np.array(bpy.data.objects['Camera'].location)
    ROT = R.from_matrix(matrix_world[:3, :3])
    camera_rpy = ROT.as_euler('xyz', degrees=True)
    original_image_size = np.array([bpy.data.scenes["Scene"].render.resolution_x,
                                    bpy.data.scenes["Scene"].render.resolution_y])

    bound = np.array([far_bound, near_bound])
    raw_save_path = sonar_path + "raw" + str(num)+ ".mat"
    sio.savemat(raw_save_path, {'camera_xyz':camera_xyz,
                                'camera_rpy':camera_rpy,
                                'original_image_size':original_image_size,
                                'bound':bound,
                                'distance_resolution':distance_resolution,
                                # 单个数视作一维数组
                                'bearing_fov':bearing_fov,
                                'vertical_fov':vertical_fov,
                                'raw':raw_array,
                                })


#################
### MAIN LOOP ###
#################
for i in range(1):
    renderOnce()
    imageGeneration(i)

# calculate time cost
time_end = time.time()
print('Total cost', time_end - time_start)

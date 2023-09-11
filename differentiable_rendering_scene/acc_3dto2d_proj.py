import numpy as np
from angle_table import angle_table

def getEle(p):  # p[0,:]->x, p[1,:]->y, p[2,:]->z
    return np.arctan2(p[1,:], np.sqrt(p[0,:]**2 + (-p[2,:])**2))

def getAzi(p):  # p[0,:]->x, p[1,:]->y, p[2,:]->z
    return np.arctan2(p[0,:], -p[2,:])

def acc_3dto2d_proj(pos_c, intensity_img, nbins, nbeams, vertical_fov, far_bound, near_bound, distance_resolution):
    polar = np.zeros([nbins, nbeams])
    counter = np.zeros([nbins, nbeams])

    # forward -> -z, right -> x, upward -> y
    phi = getEle(pos_c)
    theta = getAzi(pos_c)
    r = np.linalg.norm(pos_c, axis=0)

    # out of bound mask
    out_of_bound_mask = (phi > vertical_fov/2) | (phi < -vertical_fov/2) | (theta > angle_table.ar.max()) | (theta < angle_table.al.min()) | (r > far_bound) | (r < near_bound)

    # find beam and bin number
    bin_i = np.round((r-near_bound)/distance_resolution).astype(int)
    beam_i = np.searchsorted(angle_table.ar, theta)

    np.add.at(polar, (bin_i[~out_of_bound_mask], beam_i[~out_of_bound_mask]), intensity_img[~out_of_bound_mask])
    np.add.at(counter, (bin_i[~out_of_bound_mask], beam_i[~out_of_bound_mask]), 1)

    return polar, counter
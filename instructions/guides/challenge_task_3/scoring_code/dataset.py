import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from utils import unwrap as wrap2pi

class FlightData():
    def __init__(self, result_path, flight_id, start_idx,end_idx):

        target_path = result_path + flight_id
        file_path = target_path + "/" + flight_id + ".csv"

        print("Loading data from", target_path)

        self.raw_data = np.loadtxt(file_path,delimiter=',',skiprows=1)
        self.ts = self.raw_data[start_idx:end_idx,0]
        self.xs = self.raw_data[start_idx:end_idx,1]
        self.ys = self.raw_data[start_idx:end_idx,2]
        self.zs = self.raw_data[start_idx:end_idx,3]
        self.qxs = self.raw_data[start_idx:end_idx,4]
        self.qys = self.raw_data[start_idx:end_idx,5]
        self.qzs = self.raw_data[start_idx:end_idx,6]
        self.qws = self.raw_data[start_idx:end_idx,7]

        self.yaws = np.array([])
        self.pitches = np.array([])
        self.rolls = np.array([])

        self.traj = []

        for i in range(len(self.ts)):
            pos = np.array([self.xs[i],self.ys[i],self.zs[i]])
            quat = np.array([self.qxs[i],self.qys[i],self.qzs[i],self.qws[i]])
            rot = R.from_quat(quat).as_matrix()
            ypr = R.from_quat(quat).as_euler('zyx')
            
            T = np.eye(4)
            T[0:3,0:3] = rot
            T[0:3,3] = pos

            self.yaws = np.append(self.yaws, ypr[0])
            self.pitches = np.append(self.pitches, ypr[1])
            self.rolls = np.append(self.rolls, ypr[2])

            self.traj.append(T)

        self.yaws = wrap2pi(self.yaws)
        self.pitches = wrap2pi(self.pitches)
        self.rolls = wrap2pi(self.rolls)

        self.primary_position_tol = np.inf

        self.N = len(self.ts)
        self.elpase = self.ts[-1] - self.ts[0]

    def set_position_goal(self, primary_tol):
        self.primary_position_tol = primary_tol

    def get_dt(self, idx):
        if (idx == 0 or idx >= self.N):
            raise Exception("Invalid index for dt")
        return self.ts[idx] - self.ts[idx-1]

    def get_pose(self, idx):
        if idx >= self.N:
            raise Exception("Inquiring pose out of range!")
        return self.traj[idx]

    def get_position_error(self, i, ref_position):
        target_pose = self.get_pose(i)
        target_position = target_pose[0:3,3]
        ret = np.linalg.norm(target_position - ref_position)
        return ret
    
    def is_within_zone(self, i, ref_position):
        error = self.get_position_error(i, ref_position)
        return error <= self.primary_position_tol
 
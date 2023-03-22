import os
import bisect
import numpy as np
from scipy.spatial.transform import Rotation as R

def unwrap(x):
    return np.unwrap(2 * np.array(x)) / 2

# Find the longest increasing subsequence
def LIS(arr):
    if len(arr) == 0:
        return []
    sub = []
    subIndex = []
    path = [-1] * len(arr)
    for i, x in enumerate(arr):
        if len(sub) == 0 or sub[-1] < x:
            path[i] = -1 if len(subIndex) == 0 else subIndex[-1]
            sub.append(x)
            subIndex.append(i)
        else:
            idx = bisect.bisect_left(sub, x)
            path[i] = -1 if idx == 0 else subIndex[idx - 1]
            sub[idx] = x
            subIndex[idx] = i

    ans = []
    t = subIndex[-1]
    while t >= 0:
        ans.append(arr[t])
        t = path[t]
    if len(ans) == 1:
        return [arr[0]]
    else:
        return ans[::-1]
import os
import shutil
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from mpl_toolkits import mplot3d
from scipy.spatial.transform import Rotation as R

from dataset import FlightData
from utils import LIS
from drawing import drawSphere, putText

# Path to your result folders. Example: "/home/rob498/Vicon_Results/"
result_path = "PATH_TO_YOUR_RESULT_FOLDER"
# Path to your flight record folder. Example: "team_00_ct3_vicon_free/"
flight_id = "FLIGHT_RECORD_ID"

# Load the target waypoints here
waypoints = np.load(result_path + flight_id +"waypoints.npy")
N_wps = len(waypoints)

start_idx = 0
end_idx = -1
tolerance = 0.25

# Load data
flight_data = FlightData(result_path, flight_id, start_idx,end_idx)
flight_data.set_position_goal(tolerance)

# Find the index of the first pose that enters the volumne of each waypoint
wp_visit_pose_id = -1 * np.ones((N_wps))
for wp_id in range(N_wps):
    for pose_i in range(0, flight_data.N):
        if flight_data.is_within_zone(pose_i, waypoints[wp_id]):
            wp_visit_pose_id[wp_id] = pose_i
            break

# Find the order of waypoints that are visited
wp_visit_order = np.argsort(wp_visit_pose_id)
wp_visit_order = [wp for wp in wp_visit_order if wp_visit_pose_id[wp] != -1] # remove -1 for unvisited waypoints
# Extract the longest valid waypoint sequence
reached_wps = LIS(wp_visit_order)

# Calculate the score
N_reached_wps = len(reached_wps)
missed_wps = [i for i in range(N_wps) if i not in reached_wps]
N_missed_wps = len(missed_wps)
if N_reached_wps > 0:
    last_wp_pose_id = int(wp_visit_pose_id[reached_wps[-1]])
    last_wp_time = flight_data.ts[last_wp_pose_id] 
    score = N_reached_wps * (1.0 - 1.0 / N_wps * max(0.0, (last_wp_time - 60.0) / 60.0))
else:
    last_wp_time = 0
    score = 0

res_text_flight_time = "Total valid flight time (s): " + str(last_wp_time)
res_text_reached_wps = "Reached waypoints: " + str(reached_wps)
res_text_n_reached_wps = "Number of reached waypoints: " + str(N_reached_wps)
res_text_missed_wps = "Missed waypoints: " + str(missed_wps)
res_text_n_missed_wps = "Number of missed waypoints: " + str(N_missed_wps)
res_text_score = "Score (%): " + str(score/N_wps*100)

print(res_text_flight_time)
print(res_text_reached_wps)
print(res_text_n_reached_wps)
print(res_text_missed_wps)
print(res_text_n_missed_wps)
print(res_text_score)

# Position plots
fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
ax1.plot(flight_data.ts, flight_data.xs)
ax2.plot(flight_data.ts, flight_data.ys)
ax3.plot(flight_data.ts, flight_data.zs)

ax1.set_title('Position vs Time')
ax1.set_ylabel("x (m)")
ax2.set_ylabel("y (m)")
ax3.set_ylabel("z (m)")
ax3.set_xlabel("time (s)")

# Tajectory plot
fig2 = plt.figure()
ax4 = plt.axes(projection='3d')
ax4.plot3D(flight_data.xs, flight_data.ys, flight_data.zs,linewidth=3)

for idx in range(N_wps):
    color = 'r'
    if idx in reached_wps:
        color = 'g'
    drawSphere(ax4, waypoints[idx,:], tolerance, color)
    putText(ax4, waypoints[idx,:], str(idx))

ax4.set_title('Trajectory in 3D')
ax4.set_xlabel('x (m)')
ax4.set_ylabel('y (m)')
ax4.set_zlabel('m (m)')
ax4.view_init(elev = 50, azim = -113)

pdf = PdfPages(result_path + flight_id + "/" + flight_id + "_results.pdf")

last_page = plt.figure(figsize=(20,0.5))
last_page.clf()
last_page.text(1,7,"Flight ID: " + flight_id,transform=last_page.transFigure,size=12)
last_page.text(1,6,res_text_flight_time,transform=last_page.transFigure,size=12)
last_page.text(1,5,res_text_reached_wps,transform=last_page.transFigure,size=12)
last_page.text(1,4,res_text_n_reached_wps,transform=last_page.transFigure,size=12)
last_page.text(1,3,res_text_missed_wps,transform=last_page.transFigure,size=12)
last_page.text(1,2,res_text_n_missed_wps,transform=last_page.transFigure,size=12)
last_page.text(1,1,res_text_score,transform=last_page.transFigure,size=12)

# Save the plots as a pdf
figures = [plt.figure(i) for i in plt.get_fignums()]
for fig in figures:
    fig.savefig(pdf, format='pdf', bbox_inches='tight',pad_inches=0.5)
pdf.close()

# Draw the plots
plt.show()



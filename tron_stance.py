import mujoco
import mujoco.viewer
import time
import numpy as np
import matplotlib.pyplot as plt

model = mujoco.MjModel.from_xml_path("C:\\Users\\mosta\\OneDrive\\Desktop\\TRON1_urdf\\TRON1\\TRON1A\\PF_TRON1A\\xml\\robot.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)


model.opt.timestep = 0.001 
sim_time = 10  
sim_dt = model.opt.timestep 
ctrl_dt = 0.001
sim_steps = int(sim_time / sim_dt) 
ctrl_steps = int(ctrl_dt / sim_dt)
steps = 0
mujoco.mj_resetDataKeyframe(model, data, 0)
kp = 80
kd = 40
# for walker2d target_joint_pos = [-1.5, -0.8, 0.5, -1.5, -0.8, 0.5]
#for Tron1:
#abad  -0.38 ~ 1.39
#hip   -1.01 ~ 1.39
#knee  -0.87 ~ 1.36
target_joint_pos = np.zeros(6)
def footTrajectory(model, data, step_height, step_width, time, totalTime): 
    x = data.qpos[8][0] + setp_width * time / totalTime
    z = data.qpos[8][0] + step_height * math.sin(2 * math.pi * time)
    return [0, 0, z] 

def inverseKinematics(model, data, time): 
    #get the current foot velocity
    #    dx = J * d_theta
    #    d_theta = inv(J) * d_x
    #    incrment by d_theta until it becomes that
    pass

def getObs(model, data):
    return np.concatenate([data.qpos[3:], data.qvel[3:]])

def reset(model, data):
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        while steps < sim_steps:
            if steps % ctrl_steps == 0:
                #support leg pd 
                err = target_joint_pos - data.qpos[7: ]
                err_d = data.qvel[6:12]
                tau = np.zeros(6)
                tau = kp * err - kd * err_d + data.qfrc_bias[6:]
                #swing leg trajectory
                    # get the foot trajectory position for the crrent timestep
                    # get the inverse kinematics for the position
                    # apply that as control for the swing leg
                #applying ctrl for the stance and torso
                data.ctrl = tau
            mujoco.mj_step(model, data) 
            viewer.sync()
            time.sleep(sim_dt)
            steps += 1
        reset(model, data)
    

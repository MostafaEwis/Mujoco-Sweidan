import mujoco
import mujoco.viewer
import time
import numpy as np
import matplotlib.pyplot as plt

model = mujoco.MjModel.from_xml_path("./walker2d.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)

#creating a custom terrain
terrain_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD , "hfield")
h_nrows = model.hfield_nrow[terrain_id]
h_ncols = model.hfield_ncol[terrain_id]
model.hfield_data= np.random.uniform(size=(100,100), low=0, high =0.1).flatten()
#simulation setup
model.opt.timestep = 0.001 
sim_time = 10  
sim_dt = model.opt.timestep 
ctrl_dt = 0.001
sim_steps = int(sim_time / sim_dt) 
ctrl_steps = int(ctrl_dt / sim_dt)
steps = 0
mujoco.mj_resetDataKeyframe(model, data, 0)
#PID controls
kp = 80
kd = 40
torso_kp = 80
torso_kd = 40
# X Y Z 
# X is left and right
# Y is striking into the screen
# Z is up and down

# for walker2d target_joint_pos = [-1.5, -0.8, 0.5, -1.5, -0.8, 0.5]
#for Tron1:
#abad  -0.38 ~ 1.39
#hip   -1.01 ~ 1.39
#knee  -0.87 ~ 1.36
target_joint_pos = [-1.5, -0.8, 0.5, -1.5, -0.8, 0.5]
target_torso_tilt = 0.0
#variables for trajectory planning
#walking currently is always left and right which is the y direction
#for walking I will produce a sinsudal walk for the robot
#every dt the robot will produce a new trajectory which will change the target position for the joints through IIK
#trajectory -> kinematics -> PD -> new postion

def footTrajectory(model, data, step_height, step_width, time, totalTime): 
    x = data.qpos[8][0] + setp_width * time / totalTime
    z = data.qpos[8][0] + step_height * math.sin(2 * math.pi * time)
    return [0, 0, z] 

def inverseKinematics(model, data, time): 
    #get the current foot velocity
    #    dx = J * d_theta
    #    d_theta = dampned psuedo inverse of(J) * d_x
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
                err = target_joint_pos - data.qpos[3: ]
                err_d = data.qvel[3:]
                tau = np.zeros(6)
                tau = kp * err - kd * err_d + data.qfrc_bias[3:]
                #torso 
                torso_err = target_torso_tilt - data.qpos[2]
                torso_err_d = data.qvel[2]
                tau_for_torso = kp * torso_err - kd * torso_err_d + data.qfrc_bias[2];
                tau[0] += tau_for_torso
                tau[3] += tau_for_torso
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
    

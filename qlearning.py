from torch.distributions import Categorical
import numpy as np
import torch.nn as nn
import torch.optim as optim
import ns3env as ns3
import socket
import random
import re
import pandas as pd
import time 
import argparse
import os
from DDQN import DDQN
import torch as T
from utils import plot_learning_curve, create_directory
import sys
import tensorflow as tf
from datetime import datetime
import logging
import subprocess 
import csv
import json
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler("debug.log"),
        logging.StreamHandler()
    ]
)
t = 0
class MyEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, set):
            return list(o)
        return o


'''def storFile(data,filename): # collect information to 1 row
    with open(filename, 'w', newline='') as f:
        mywrite = csv.writer(f)
        mywrite.writerow(data)'''
'''def storFile(data, fileName):
    data = list(map(lambda x:[x], data))
    with open(fileName, 'w', newline = '') as f:
        mywrite = csv.writer(f)
        for key, i in enumerate(data):
            mywrite.writerow(i)'''

parser = argparse.ArgumentParser()
parser.add_argument('--max_episodes', type=int, default=150)
parser.add_argument('--ckpt_dir', type=str, default='/home/alex/RL-NS3-main_2/checkpoints/DDQN/')
parser.add_argument('--reward_path', type=str, default='/home/alex/RL-NS3-main_2/output_images/avg_reward/avg_reward.png')
parser.add_argument('--epsilon_path', type=str, default='/home/alex/RL-NS3-main_2/output_images/epsilon/epsilon.png')
parser.add_argument('--loss_path', type=str, default='/home/alex/RL-NS3-main_2/output_images/epsilon/loss.png')

args = parser.parse_args()
try:
    os.environ["COMBINED_DATA_PATH"]
    os.environ["DATA_PATH"]
    os.environ["NUMBER_OF_NODES"]
except:
    os.environ["COMBINED_DATA_PATH"] =  "/home/alex/RL-NS3-main_2/" # path to RL script
    os.environ["DATA_PATH"] = "/home/alex/ns-3-mmwave-oran" # path to the files with stats, by default it is placed in the root folder of ns3
    os.environ["NUMBER_OF_NODES"] = "3" 
names =["cu-up-cell-", "cu-cp-cell-", "du-cell-"]
num_of_nodes_offset = 2
#alpha = 0.1
#gamma = 0.6
#epsilon = 0.1
data_combiner_kiiler_command = ["./setup_killer.sh"]
N_steps =  20 / 2.0 # Simtime / period of RIC indication
state_space = {}
state_count = 0
class State(): # create class of state to use state as a key for python's dict
    def __init__(self,a1):
        self.a1 = a1

    def __hash__(self): 
        return hash(frozenset([int(item) for item in self.a1]))

    def __eq__(self, other):
        return (self.a1.size == other.a1.size and self.__hash__() == other.__hash__())

    def __ne__(self, other):
        # Not strictly necessary, but to avoid having both x==y and x!=y
        # True at the same time
        return not(self == other)
    def __str__(self):
        return str([str(i) + " " for i in self.a1])  
class StateTranslator(): # state translator to transform state to the index of this state in state_space
    def __init__(self):
        self.state_space = {}
        self.state_count = 0
    def get_state_index(self, state_):
        state_ = State(state_)
        if state_ in self.state_space:
            return self.state_space[state_]
        else:
            self.state_space[state_] = self.state_count
            self.state_count+=1
            return self.state_space[state_]
def get_state(df, df1, df2): # function to parsing dataframe for needed information
        if df.shape[1] < 2:
            return 0, 0, np.array([0,0])
        df = df[df["timestamp"] == df["timestamp"].max()] # choose data for the last timestamp
        df1 = df1[df1["timestamp"] == df1["timestamp"].max()]
        df2 = df2[df2["timestamp"] == df2["timestamp"].max()]
        df.columns = df.columns.map(lambda x: re.sub('\s\(.*?\)$','',x))
        df1.columns = df1.columns.map(lambda x: re.sub('\s\(.*?\)$','',x))
        df2.columns = df2.columns.map(lambda x: re.sub('\s\(.*?\)$','',x))
        #df1 = df1.drop(['timestamp', 'NodeId', 'ueImsiComplete'], axis = 1, inplace=True)
        #df2 = df2.drop(['timestamp', 'NodeId', 'ueImsiComplete'], axis = 1, inplace=True)
        df = pd.concat([df, df1, df2], axis=1)
        df["id"] = df.index
        df = pd.wide_to_long(df, ["L3 neigh Id ", "L3 neigh SINR 3gpp "  ], i ="id", j="node") # transform df to have only one column "L3 neigh Id " and "L3 neigh SINR 3gpp "
        df = df.loc[:,~df.columns.duplicated()]
        #df = df.T.groupby(level=0).first().T
        df.columns = df.columns.map(lambda x: re.sub('\s$','',x)) # droping space at the end of columns
        print(df.keys())
        print(df.index.dtype)
        #df = df[["L3 neigh Id", "L3 neigh SINR 3gpp", "L3 serving SINR 3gpp", "UE", "timestamp", "L3 serving Id(m_cellId)", "dlPrbUsage", "RRU.PrbUsedDl", "TB.ErrTotalNbrDl.1.UEID", "DRB.PdcpSduBitRateDl.UEID(pdcpThroughput)","DRB.PdcpSduDelayDl.UEID", "DRB.PdcpPduNbrDl.Qos.UEID" ]] # choose necessary columns 
        df = df[["L3 neigh Id", "L3 neigh SINR 3gpp", "L3 serving SINR 3gpp", "UE", "timestamp", "L3 serving Id(m_cellId)", "dlPrbUsage", "RRU.PrbUsedDl", "TB.ErrTotalNbrDl.1.UEID" ]] # choose necessary columns 
        #df = df.drop(df.columns[[6,7]], axis=1, inplace=True)
        
        logging.info(df)
        df = df.drop_duplicates().reset_index(drop=True)
        df["L3 neigh Id"] = pd.to_numeric(df['L3 neigh Id'], errors='coerce')
        df["L3 neigh SINR 3gpp"] = pd.to_numeric(df['L3 neigh SINR 3gpp'], errors='coerce')
        df["L3 serving SINR 3gpp"] = pd.to_numeric(df['L3 serving SINR 3gpp'], errors='coerce')
        df["UE"] = pd.to_numeric(df['UE'], errors='coerce')
        df["timestamp"] = pd.to_numeric(df['timestamp'], errors='coerce')
        df["L3 serving Id(m_cellId)"] = pd.to_numeric(df['L3 serving Id(m_cellId)'], errors='coerce')
        df["dlPrbUsage"] = pd.to_numeric(df['dlPrbUsage'], errors='coerce') # percent of used PRB 
        df["RRU.PrbUsedDl"] = pd.to_numeric(df['RRU.PrbUsedDl'], errors='coerce')  # total amount of used PRB
        df["TB.ErrTotalNbrDl.1.UEID"] = pd.to_numeric(df['TB.ErrTotalNbrDl.1.UEID'], errors='coerce') # number dublicated packets
        #df["DRB.PdcpSduBitRateDl.UEID(pdcpThroughput)"] = pd.to_numeric(df["DRB.PdcpSduBitRateDl.UEID(pdcpThroughput)"], errors='coerce') #throughput
        #df["DRB.PdcpSduDelayDl.UEID"] = pd.to_numeric(df["DRB.PdcpSduDelayDl.UEID"], errors='coerce') # delay
        #df["DRB.PdcpPduNbrDl.Qos.UEID"] = pd.to_numeric(df["DRB.PdcpPduNbrDl.Qos.UEID"], errors='coerce') # number transmitted specific user pdcp packets in dl 
        df = df.dropna() # drop nans       
        logging.info(df)
        #x = 
        #print("type of x: ", type(x))
        #num = int(x)
        
        num_of_cells = int(max(df["L3 neigh Id"].max(), df["L3 serving Id(m_cellId)"].max()))-1 # find maximum number of cell for each we have data
        state = np.ones((num_of_cells, 1)) * (-200) # generate state numpy array with default -200 value

        for i in range(0, num_of_cells):
            try:
               state[i] = int(float((df[df["L3 neigh Id"] == i+2])["L3 neigh SINR 3gpp"].iloc[0])) # add SINR of neighbour cells
               
            except:
                continue
        state[int(float(df["L3 serving Id(m_cellId)"].iloc[0]))-2] = int(float(df["L3 serving SINR 3gpp"].iloc[0])) # add SINR of serving cells
        serv_cell_id = int(float(df["L3 serving Id(m_cellId)"].iloc[0])) 
        ue_id = int(float(df["UE"].iloc[0]))
        return serv_cell_id, ue_id, state

def get_state1(df): # function to parsing dataframe for needed information
        if df.shape[1] < 2:
            return 0, 0, np.array([0,0])
        df = df[df["timestamp"] == df["timestamp"].max()] # choose data for the last timestamp
        df.columns = df.columns.map(lambda x: re.sub('\s\(.*?\)$','',x)) # drop postfixes in brackets (cellId,  convertedSinr) in column's name
        df["id"] = df.index
        df = pd.wide_to_long(df, ["L3 neigh Id ", "L3 neigh SINR 3gpp "], i ="id", j="node") # transform df to have only one column "L3 neigh Id " and "L3 neigh SINR 3gpp "
        df.columns = df.columns.map(lambda x: re.sub('\s$','',x)) # droping space at the end of columns
        #df.columns = df.columns.map(lambda x: re.sub('\s\(.*?\)$','',x)) # choose necessary columns
        df = df[["L3 neigh Id", "L3 neigh SINR 3gpp", "L3 serving SINR 3gpp", "UE", "timestamp", "L3 serving Id(m_cellId)"]]
        #df.columns = df.columns.map(lambda x: pd.to_numeric(x, errors='coerce'))
        df = df.drop_duplicates().reset_index(drop=True)
        df["L3 neigh Id"] = pd.to_numeric(df['L3 neigh Id'], errors='coerce')
        df["L3 neigh SINR 3gpp"] = pd.to_numeric(df['L3 neigh SINR 3gpp'], errors='coerce')
        df["L3 serving SINR 3gpp"] = pd.to_numeric(df['L3 serving SINR 3gpp'], errors='coerce')
        df["UE"] = pd.to_numeric(df['UE'], errors='coerce')
        df["timestamp"] = pd.to_numeric(df['timestamp'], errors='coerce')
        df["L3 serving Id(m_cellId)"] = pd.to_numeric(df['L3 serving Id(m_cellId)'], errors='coerce')
        df = df.dropna() # drop nans
        if df.empty == True:
            return True
        else:
            return False

def main():
    if T.cuda.is_available():
        logging.info("cuda is ok")
    else:
        logging.info("cuda doesnt work")
    #data = []


    #gpus = tf.config.list_physical_devices('GPU')
    #if gpus:
    # Restrict TensorFlow to only allocate 1GB of memory on the first GPU
    #    try:
    #        tf.config.set_logical_device_configuration(
    #            gpus[0],
    #            [tf.config.LogicalDeviceConfiguration(memory_limit=2048)])
    #        logical_gpus = tf.config.list_logical_devices('GPU')
    #        print(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPUs")
    #    except RuntimeError as e:
    #        # Virtual devices must be set before GPUs have been initialized
    #        print(e)

    env = ns3.ns3Environment(socket.AF_INET, socket.SOCK_STREAM) # create ns-3 playground
    agent = DDQN(alpha=0.0003, state_dim=3, action_dim=env.action_space.n,
                 fc1_dim=3, fc2_dim=256, ckpt_dir=args.ckpt_dir, gamma=0.99, tau=0.005, epsilon=1.0,
                 eps_end=0.01, eps_dec=0.003, max_size=1000000, batch_size=256)
    #eps_dec==5e-4
    create_directory(args.ckpt_dir, sub_dirs=['Q_eval', 'Q_target'])
    total_rewards, avg_rewards, eps_history, mselist = [], [], [], []
    data = {
                "episodes" : []
            }
    for episode in range(args.max_episodes):
        env.reset() # Initializing ns-3 script 
        #time.sleep(1)

        serv_cell_id, ue_id, state = get_state(pd.DataFrame([0]), pd.DataFrame([0]), pd.DataFrame([0])) # initial values are zeroes
        data, done = env.get_data() # data is ue id
        total_reward = 0
        info = ""
        action = 0
        mse = 100.0
        chance = True
        #data.append("episode")
        #data.append(episode)
        for t in range(int(N_steps)): # start
            #if t == 10:
            #    break
            if done: # if ns-3 script is not working, break this loop
               break
            if chance == get_state1(env.cu_cp.copy()):
                break
            #    data, done = env.get_data() # data is ue id
            #    serv_cell_id, ue_id, state = get_state(pd.DataFrame([0]))
            #    for k in range(t): # start
            #        serv_cell_id, ue_id, state = get_state(env.cu_cp.copy()) # parse dataframe for needed data
            #        action = agent.choose_action(np.transpose(state), isTrain=True)
            #        info = str(serv_cell_id) + " " + str(ue_id)                                
            #        done = env.step(info, action+2) # do step, apply action to ns-3
            #        if done: # if ns-3 script is not working, break this loop
            #            break   
            #        if k == 0: # reward for the first step is 0
            #            reward = 0
            #        else: # reward for others steps
            #            reward = (-(max(state) - state[action]) ) + 100 
            #        data, done = env.get_data() # data is ue id
            #        serv_cell_id, ue_id, state_ = get_state(env.cu_cp.copy())
            #        agent.remember(np.transpose(state), action, reward, np.transpose(state_), done)
            #        agent.learn()
            #        total_reward+=reward
            #        state = state_
            #        if k == t:
            #            t = k + 1
            serv_cell_id, ue_id, state = get_state(env.cu_cp.copy(), env.cu_up.copy(), env.du.copy()) # parse dataframe for needed data
            action = agent.choose_action(np.transpose(state), isTrain=True)
            info = str(serv_cell_id) + " " + str(ue_id) # additional data for ns-3 script in this case we send information about which UE should be connected to target cell 
                                                        # (action) from serving cell. Result message is "SERV_CELL_ID UE_ID TARGET_CELL_ID"
            done = env.step(info, action+2) # do step, apply action to ns-3
            #data.append("serv_cell_id")
            #data.append(serv_cell_id)
            #data.append("ue_id")
            #data.append(ue_id)
            #r.append("done")
            #r.append(done)
            if done: # if ns-3 script is not working, break this loop
                break   
            if t == 0: # reward for the first step is 0
                reward = 0
            elif state[action] == -200:
                reward = -200
                 # In example we maximize the SINR of UE, in this case the reward is just a difference beetween current cell and cell with highest SINR
            else:
                reward = (-(max(state) - state[action]) )


            data, done = env.get_data() # data is ue id
            #data.append("reward")
            
            
            #data.append(reward)
            if done: 
                break
            serv_cell_id, ue_id, state_ = get_state(env.cu_cp.copy(), env.cu_up.copy(), env.du.copy()) # parse dataframe for needed data
            
            agent.remember(np.transpose(state), action, reward, np.transpose(state_), done)
            mse = agent.learn()
            total_reward+=reward
            
            state = state_
            
            
            reward_json = str(reward)
            state_json = str(state)
            #json_arr = []
            
            #line = [{"episode_count" : {episode + 1}, "time" : {"count" : {t*2}, "items" : [{"serv_cell_id" : {serv_cell_id}, "ue_id" : {ue_id}, "reward" : {reward_json}} ]} }]
            line = [{"episode_count" : {episode + 1}, "time" : {(t*2)+2}, "serv_cell_id" : {serv_cell_id}, "ue_id" : {ue_id}, "state" : {state_json}, "reward" : {reward_json} }]
            data["episodes"].append(line)

        
            with open('/home/alex/RL-NS3-main_2/handover.json', 'r') as file:
                json.load(file)
            new_json = json.dumps(line, cls=MyEncoder)  
            with open('/home/alex/RL-NS3-main_2/handover.json', 'w') as file:
                json.dump(data, file, indent=3, cls=MyEncoder)
            
           
           
            #rewards_hist[epi] = total_reward
            #solved = total_reward > 100
            #print(f'Episode {epi}, total_reward: {total_reward}, solved: {solved}')
        
        total_rewards.append(total_reward)
        
        pd.DataFrame(total_rewards)
        avg_reward = np.mean(total_rewards[-100:])
        avg_rewards.append(avg_reward)
        eps_history.append(agent.epsilon)
        mselist.append(int(0 if mse is None else mse))
        logging.info('EP:{} reward:{} avg_reward:{} epsilon:{} mse:{}'.
            format(episode + 1, total_reward, avg_reward, agent.epsilon, int(0 if mse is None else mse)))
        env.killer()
        if (episode+ 1) % 25 == 0:
            agent.save_models(episode + 1)
    
    #np.save("rewards_hist_100i.npy", rewards_hist )
    #np.save("q_table_100i.npy", q_table )
    episodes = [i for i in range(args.max_episodes)]
    plot_learning_curve(episodes, avg_rewards, 'Reward', 'reward', args.reward_path)
    plot_learning_curve(episodes, eps_history, 'Epsilon', 'epsilon', args.epsilon_path)
    plot_learning_curve(episodes, mselist, "MSE", "mse", args.loss_path )
print(__name__)
if __name__ == '__main__':
    main()

import socket
import time
import numpy as np
import os
import subprocess 
import pandas as pd
import re
import csv
from data_combiner import DataCombiner
#ns-3env is working with combined data from multiple nodes. 
# Before starting to work with ns3 and RL script we have to launch the data combiner which collect data from files "cu-cp-cell-*.txt", "cu-up-cell-*.txt", "du-cell-*.txt" to
# "cu-cp-cell-common.txt", "cu-up-cell-common.txt", "du-cell-common.txt" respectively
ns3_command = ["./waf","--run"] # command to run NS-3 script


import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler("debug.log"),
        logging.StreamHandler()
    ]
)

class ActionSpace: # class to store possible action space
    def __init__(self, space_n):
        self.n = space_n
        self.action_space_ = np.array([i for i in range(space_n)])
    def sample(self): # random choosing of actions
        return np.random.choice(self.action_space_)
    def items(self): # return list of all actions
        return  self.action_space_

class ObservationSpace: # class to store possible action space
    def __init__(self, space_n):
        self.n = space_n
        self.observation_space_ = np.array([i for i in range(space_n)])


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

def get_state(df): # function to parsing dataframe for needed information
        if df.shape[1] < 2:
            return 0, 0, np.array([0,0])
        df = df[df["timestamp"] == df["timestamp"].max()] # choose data for the last timestamp
        df.columns = df.columns.map(lambda x: re.sub('\s\(.*?\)$','',x)) # drop postfixes in brackets (cellId,  convertedSinr) in column's name
        df["id"] = df.index
        df = pd.wide_to_long(df, ["L3 neigh Id ", "L3 neigh SINR 3gpp "], i ="id", j="node") # transform df to have only one column "L3 neigh Id " and "L3 neigh SINR 3gpp "
        df.columns = df.columns.map(lambda x: re.sub('\s$','',x)) # droping space at the end of columns
        df = df[["L3 neigh Id", "L3 neigh SINR 3gpp", "L3 serving SINR 3gpp", "UE", "timestamp", "L3 serving Id(m_cellId)"]] # choose necessary columns
        df = df.dropna() # drop nans
        num_of_cells = int(max(df["L3 neigh Id"].max(), df["L3 serving Id(m_cellId)"].max()))-1 # find maximum number of cell for each we have data
        state = np.ones((num_of_cells, 1)) * (-200) # generate state numpy array with default -200 value

        for i in range(0, num_of_cells):
            try:
               state[i] = int((df[df["L3 neigh Id"] == i+2])["L3 neigh SINR 3gpp"].iloc[0]) # add SINR of neighbour cells
            except:
                continue
        state[int(df["L3 serving Id(m_cellId)"].iloc[0])-2] = int(df["L3 serving SINR 3gpp"].iloc[0]) # add SINR of serving cells
        serv_cell_id = int(df["L3 serving Id(sm_cellId)"].iloc[0]) 
        ue_id = int(df["UE"].iloc[0])
        return serv_cell_id, ue_id, state

class ns3Environment(socket.socket ): # inherited from socket to provide connection to ns-3 script via sockets
    def __init__(self, inet, stream, ns3_dir : str, rl_dir:str, script : str):
        ns3_command.append(script)
        socket.socket.__init__(self, inet, stream)
        self.HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
        self.PORT = 1477  # Port to listen on (non-privileged ports are > 1023)
        self.action_space = ActionSpace(3) # create action space
        self.observation_space = ObservationSpace(500)
        self.observation_space_n = 500 # possible number of observation space. number is quite random
        self.bind((self.HOST, self.PORT)) # bind socket socket is closed when the object is destroyed
        self.data_combiner = DataCombiner(ns3_dir, rl_dir)
        self.ns3_dir = ns3_dir
        self.rl_dir = rl_dir

    def get_action_space(self):
        return self.action_space.items()
    
    def read_files(self):
        self.data_combiner.get_data()
        filenames = ["cu-up-cell-common.txt", "cu-cp-cell-common.txt", "du-cell-common.txt"] # list of files with data

        self.cu_up = pd.read_csv(filenames[0], sep=',', encoding='utf-8', on_bad_lines='skip', quoting=csv.QUOTE_NONE, quotechar='"') # read as pandas DataFrame
        self.cu_cp = pd.read_csv(filenames[1], sep=',', encoding='utf-8', on_bad_lines='skip', quoting=csv.QUOTE_NONE, quotechar='"')
        self.du = pd.read_csv(filenames[2], sep=',', encoding='utf-8', on_bad_lines='skip', quoting=csv.QUOTE_NONE, quotechar='"' )
        #, on_bad_lines = 'skip'

    def get_data(self):
        data = self.conn.recv(1024) # waiting for connection from ns-3
        done = self.ns3_process.poll() # check is process is still alive
        if done == 0:
            return data, True # say to RL script what ns-3 script is crushed or done
        elif done == 1:
            return data, True
        self.read_files() # read files, update stored dataframes with statistics
        return data, False
       

    
    def reset(self):
        self.data_combiner.reset()
        try:
            self.ns3_process.kill() # try to kill previous process with data combiner
        except:
            pass
        os.chdir(self.ns3_dir)
        self.ns3_process = subprocess.Popen(ns3_command) # run ns-3 script
        os.chdir(self.rl_dir)
        #time.sleep(1)
        self.listen()
        self.conn, self.addr = self.accept() # accept connection from ns-3
        #self.get_data()
        self.prev_timestep = 0


    def send_action(self, info, action):
        #print(self.action_space[str(action)].encode('utf-8'))
        try:
            self.conn.sendall((str(info) + " " + str(action)).encode('utf-8')) # send info + action
        except BrokenPipeError:
            pass
    def step(self, info, action):
        self.send_action(info, action)
        #done = self.ns3_process.poll() # check is ns-3 is still alive
        #logging.info("poll, ", done )
        #if done == None:
        #    return True

        #while self.ns3_process.poll() is None:
        #    logging.info("still working")
        #    a =  self.ns3_process.poll()
        #else:
        #    done = a

        #if done == None:

        #   return True
        #if done == 0:
        #    return True
        
        return False



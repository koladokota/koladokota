import time, os

class DataCombiner:
    def __init__(self, ns3_dir, rl_dir):
        
        self.end_of_files = {}
        try:
            os.environ["NUMBER_OF_NODES"]
        except:
            os.environ["NUMBER_OF_NODES"] = "3" # number of nodes for combining data. In my simple there is only 3 5G nodes
        self.combined_data_path = rl_dir
        self.data_path = ns3_dir
        self.names =["cu-up-cell-", "cu-cp-cell-", "du-cell-"] # which type of stats will be parsed
        self.flags_for_header = {"cu-up-cell-" : False, "cu-cp-cell-" : False, "du-cell-" : False,} 
        self.num_of_nodes_offset = 2 # number of 5g nodes beggining with 2, 0 - non exist, 1 - LTE
        for name in self.names:
            try:
                for node_id in range (self.num_of_nodes_offset, int(os.environ["NUMBER_OF_NODES"]) + self.num_of_nodes_offset):
                        try:
                            os.remove(str(os.path.join(self.data_path, name)) + str(node_id) + ".txt") # delete previous combined stats
                        except:
                            print("Unable to remove: ", str(os.path.join(self.data_path, name)) + str(node_id) + ".txt")
                os.remove(str(os.path.join(self.combined_data_path, name)) + "common.txt") # delete previous combined stats
            except:
                print("Unable to remove: ", str(os.path.join(self.combined_data_path, name)) + "common.txt")

    def get_data(self):
        for name in self.names:  
            in_common_file = open(str(os.path.join(self.combined_data_path, name)) + "common.txt",'a') # create comon file
            header_flag = False
            for node_id in range (self.num_of_nodes_offset, int(os.environ["NUMBER_OF_NODES"]) + self.num_of_nodes_offset):
                file_name = str(os.path.join(self.data_path, name)) + str(node_id) + ".txt"
                try: # try read files with stat, if there are no files script continues to run anyway
                    in_file = open(file_name,'r')
                except:
                    print("Unable to open: ", file_name)
                    continue
                if file_name not in self.end_of_files.keys(): 
                    if self.flags_for_header[name]:
                        in_file.readline()
                    self.end_of_files[file_name] = in_file.tell()
                fileBytePos = self.end_of_files[file_name]
                data = ""
                while fileBytePos != in_file.seek(0, 2): # keep going untill we reach the end of file
                    in_file.seek(fileBytePos)
                    if not self.flags_for_header[name]:
                        data = data  + "NodeId," + in_file.readline() # add columns name NodeId
                        self.flags_for_header[name] = True
                    readed_line = in_file.readline()
                    if len(readed_line) != 0:
                        data = data  + str(node_id) + "," + readed_line # add data
                    fileBytePos = in_file.tell()
                    self.end_of_files[file_name] = fileBytePos
                in_file.close()
                in_common_file.write(data)

            in_common_file.close()
    def reset(self):
        self.end_of_files = {}
        self.flags_for_header = {"cu-up-cell-" : False, "cu-cp-cell-" : False, "du-cell-" : False,}
        for name in self.names:
            try:
                for node_id in range (self.num_of_nodes_offset, int(os.environ["NUMBER_OF_NODES"]) + self.num_of_nodes_offset):
                        try:
                            os.remove(str(os.path.join(self.data_path, name)) + str(node_id) + ".txt") # delete previous combined stats
                        except:
                            print("Unable to remove: ", str(os.path.join(self.data_path, name)) + str(node_id) + ".txt")
                os.remove(str(os.path.join(self.combined_data_path, name)) + "common.txt") # delete previous combined stats
            except:
                print("Unable to remove: ", str(os.path.join(self.combined_data_path, name)) + "common.txt")

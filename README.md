# Install WSL
Use [instruction](https://learn.microsoft.com/ru-ru/windows/wsl/install)

# Install VS Code 
Use [link](https://code.visualstudio.com/)

# Install WSL Extension and connect to WSL
[Link](https://learn.microsoft.com/ru-ru/windows/wsl/tutorials/wsl-vscode)

# Install Ubuntu 22.04 on WSL
```
wsl --install Ubuntu-22.04
```
# Install Docker 

[Follow](https://docs.docker.com/engine/install/ubuntu/) section "Install using the apt repository"

# Install ORAN GYM
Installation of ORAN docker containers could be skipped

# Clone this repository and replace files of ns-3 by files in scratch

- First of all add GPG keys to settings ([instruction](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account))

- Clone repository:
```
git clone https://github.com/Seanei/RL-NS3
```

- Replace NS3  files by files from this repository

# Install pip
```
sudo apt update
sudo apt install python3-venv python3-pip
```

# Install used libraries
```
pip install torch numpy pandas buffer tensorflow typing_extensions
```

# Run with ns-3 (example paths)
Assuming:
- ns-3 is in `~/ns-3-mmwave-oran`
- this repo is in `~/RL-NS3`

1) Copy the scenario into ns-3:
```
cp ~/RL-NS3/scratch/x2-handover-5g.cc ~/ns-3-mmwave-oran/scratch/
```

2) Build ns-3:
```
cd ~/ns-3-mmwave-oran
./waf configure
./waf build
```

3) Run the RL script with the correct paths:
```
cd ~/RL-NS3
export COMBINED_DATA_PATH=~/RL-NS3
export DATA_PATH=~/ns-3-mmwave-oran
export NUMBER_OF_NODES=3
python3 qlearning.py
```

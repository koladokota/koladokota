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

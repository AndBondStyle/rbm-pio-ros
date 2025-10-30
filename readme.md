# Notes

Host setup:
```
wget -qO - https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Start & attach to docker container:
```
docker compose up -d
docker compose exec ros bash
```

Initial setup:
```
echo "cd /src" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source /src/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Extra requirements:
```
apt update
apt install -y python3-serial
apt install -y pipx
pipx ensurepath && source ~/.bashrc
pipx install platformio
```

Build & upload firmware:
```
pio run -t upload
```

Build foxglove bridge from source (version 0.8.5):
```
// TODO
```

Build custom ROS packages
```
make build-all
```

Start ROS pipeline (multiple terminals / tmux):
```
[term a] ros2 launch main bridge.yaml
[term b] python3 packages/main/hwnode.py
```

#!/bin/bash
echo "WARNING: DO NOT USE THIS !!!!!!!!!!!"
echo "!! WARNING: This script attempts a dirty downgrade from Ubuntu 25.04 to 24.04 (noble)"
echo "!! This may break your system. Make backups if needed."
read -p "Continue anyway? (y/n): " CONFIRM

if [[ "$CONFIRM" != "y" ]]; then
  echo "Aborted."
  exit 0
fi

# 1. Update hostname and codename references
echo "[*] Updating APT sources from 'plucky' to 'noble'..."
sudo sed -i 's/plucky/noble/g' /etc/apt/sources.list
sudo sed -i 's/plucky/noble/g' /etc/apt/sources.list.d/*.list 2>/dev/null

# 2. Pin Ubuntu 24.04 (noble) packages at higher priority
echo "[*] Setting APT preferences to prioritize noble packages..."
cat <<EOF | sudo tee /etc/apt/preferences.d/downgrade.pref
Package: *
Pin: release n=noble
Pin-Priority: 1001
EOF

# 3. Update package lists
echo "[*] Updating APT package lists..."
sudo apt update

# 4. Begin full upgrade (effectively a downgrade in our case)
echo "[*] Attempting full downgrade via upgrade..."
sudo apt dist-upgrade -y

# 5. Clean up unnecessary packages
sudo apt autoremove -y
sudo apt autoclean

# 6. Set locale just in case
echo "[*] Resetting locale..."
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 7. Optional: set ROS 2 Jazzy sources (Noble-only)
echo "[*] Adding ROS 2 Jazzy repo..."
sudo apt install -y curl gnupg2 lsb-release software-properties-common
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
echo "[*] Attempt complete. Rebooting is strongly recommended."

read -p "Reboot now? (y/n): " REBOOT
if [[ "$REBOOT" == "y" ]]; then
  sudo reboot
else
  echo "You should reboot manually before doing anything else."
fi

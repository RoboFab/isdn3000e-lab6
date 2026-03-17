## ISDN3000e-Lab6

This is the lab of the course **ISDN3000e: Programming for Integrative Systems** at HKUST led by Prof. Ziqi Wang. 

The lab requires **Pinocchio** for robot kinematics. Please install it following the commands below.

### Linux / WSL (Ubuntu 20.04 / 22.04 / 24.04)

Run the following commands in your Ubuntu / WSL terminal:

```bash
sudo apt install -qqy lsb-release curl

sudo mkdir -p /etc/apt/keyrings
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
| sudo tee /etc/apt/keyrings/robotpkg.asc

echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] \
http://robotpkg.openrobots.org/packages/debian/pub \
$(lsb_release -cs) robotpkg" \
| sudo tee /etc/apt/sources.list.d/robotpkg.list

sudo apt update
sudo apt install -qqy robotpkg-py3*-pinocchio

echo 'export PATH=/opt/openrobots/bin:$PATH' >> ~/.bashrc
echo 'export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> ~/.bashrc

source ~/.bashrc
```

### MacOS (Homebrew)

Run the following command in your terminal:

```bash
brew tap gepetto/homebrew-gepetto
brew install pinocchio
```

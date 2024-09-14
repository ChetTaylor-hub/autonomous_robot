#!/bin/bash

# 设置工作空间名称
WORKSPACE_NAME="autonomous_robot_ws"

# 创建工作空间和src目录
mkdir -p ~/$WORKSPACE_NAME/src
cd ~/$WORKSPACE_NAME/src

# 克隆项目仓库（假设您的项目在GitHub上）
git clone https://github.com/ChetTaylor-hub/autonomous_robot

# 安装依赖项
sudo apt-get update
sudo apt-get install -y ros-noetic-gmapping ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro ros-noetic-navigation ros-noetic-map-server ros-noetic-amcl ros-noetic-teb-local-planner

# 安装Python依赖
pip3 install torch torchvision torchaudio

# 编译工作空间
cd ~/$WORKSPACE_NAME
catkin_make

# 设置环境变量
echo "source ~/$WORKSPACE_NAME/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 创建启动脚本
echo '#!/bin/bash' > ~/$WORKSPACE_NAME/start_simulation.sh
echo "source ~/$WORKSPACE_NAME/devel/setup.bash" >> ~/$WORKSPACE_NAME/start_simulation.sh
echo "roslaunch autonomous_robot autonomous_robot.launch" >> ~/$WORKSPACE_NAME/start_simulation.sh
chmod +x ~/$WORKSPACE_NAME/start_simulation.sh

echo "工程构建完成！"
echo "要启动仿真，请运行: ~/$WORKSPACE_NAME/start_simulation.sh"
# 项目功能仓库
本仓库为私有仓库，包含当前开发的核心功能代码

## 功能说明

## 运行环境
Ubuntu 系统

## 访问令牌为：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

##/home/fzx/RoboMaster/src/IntelligentUAVChampionshipSimulator下的/simulator12.0.0.3可以去https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/releases/tag/v202302.1.0下下载，并可按照IntelligentUAVChampionshipSimulator的步骤解压放在相应位置；

##有个airsim_ros功能包，设置了软连接：
“ 创建软链接，指向basic_dev里的airsim_ros源码（
ln -s /home/fzx/IntelligentUAVChampionshipBase/basic_dev/src/airsim_ros ./airsim_ros
验证软链接：能看到airsim_ros，且箭头指向源码路径
ls -l | grep airsim_ros ”

##先roscore，再启动./run_simulator.sh 123，然后输入rosservice call /airsim_node/drone_1/takeoff "{}"即可起飞，source一下输入rqt_image_view，能看到实时图

##再打开到basic_dev,输入：
source devel/setup.bash
roslaunch basic_dev basic_dev.launch
可看到无人机起飞后，以一定速度向前飞，飞10s后悬停。

# VO1

将视觉SLAM十四讲第九讲的0.2改为双目版本，特征点深度的计算借鉴了ORB-SLAM2中Frame类中的ComputeStereoMatches()函数

每帧耗时约为60ms，绝大部分时间花在特征点提取上，约50ms

请按照高翔博士后的《视觉SLAM十四讲》安装前九讲中除g2o的所有库，g2o库请安装github上的版本

建议利用CMake进行编译：

进入VO1主目录：cd VO1

mkdir build

cd build

cmake ..

make

调用时使用控制台，本程序的demo针对kitti数据集的序列三 在控制台中使用如下方法调用：

进入VO1主目录：cd VO1

启动程序：./bin/run_vo config/default.yaml /home/wen/dataset/1/03

ps：/home/wen/dataset/1/03为作者的kitti数据集所在目录，需根据自己的情况进行更改

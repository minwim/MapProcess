# 1. 这个文件 test_map.cpp 如何使用

---0013
   |--- output.txt
   |--- input 
   |--- mask

   |--- global
   |--- local 
   |--- output
   |--- track
   |--- perception
   |--- x86 
        |--- global
        |--- local 
        |--- output
        |--- track
        |--- perception



其中 **output.txt** 里面是位姿信息 如下
58885550 240 0.2998 2.8438 1.6014
分别代表 
时间戳,帧号, x_pose,y_pose 航向角yaw


**input** 是一个文件夹
如上   input_index_240.jpg
里面存储这些 jpg文件

**mask** 是一个文件夹
如上   mask_index_240.png
里面存储对应 mask png 因为怕失真

其中 
**global** **local** **track** **perception**
这些是 arm端的真实数据

**x86** 文件夹这个使用来存储 x86 下的仿真数据。


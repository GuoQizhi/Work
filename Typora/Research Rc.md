11/21

- 接通OSDK--（pps，B0口）--stm32--（分频）--1Hz（B5口）给到lidar，通过livox hub接受pps & 10Hz（B8或B9）给到相机，相机线一根接地，一根接收pps
- 注意stm32的供电口，从输入端将电输入进来
- 下载keil5，配齐stm32 F1xx系列package
- 使用串口烧录软件:将keil文件编译--生成hex文件（功能栏中间位置的魔法棒）--将hex文件导入串口烧录软件--开始编程（下次开始编程前需清除芯片）
- 修改yuanchongjian代码，之前只能在B8、B9发出10Hz脉冲，现加入从B5针发出1Hz脉冲

11/22

- 完成CAPD大作业
- 通过画嵌套莫比乌斯环，熟系SolidWorks插入、特征等功能
- 了解SLS一些原理和特点

11/23

- livox avia雷达内有雷达开机时间和内置IMU的时间戳，这两个时间信息不同步。
- 修改完相机底座后注意不要在lidar的FOV里面
- 研究神经网络作业，学习卷积原理

11/25

- 改图，修改相机底座，使其能够放下更大的镜头
- 学习使用ultimaker-cura,3D打印的软件。

​       操作步骤：转成stl文件格式

​                          导入文件到Cura

​                          在prepare中调节零件放置方式和角度

​                          在preview中调节三个重要参数：0.2mm精度、20%打印材料输入、支撑材料的threshold（80°or85°，由经验判断，主要用来区分哪里需要                           support material，哪里不需要）

- 3D打印成功出件，配合完美
- 独立完成CPS的individual部分作业。真正自己动手实操过才会有结论，才能写出东西。
- 做神经网络的30词dataset

11/28

- 该ros文件的参数，从config和launch文件夹去找
- ros record是记录数据
- ros play 时间(通过tab)运行记录的数据
- 20ms指的是用该CPU跑fast-lio算法生成一帧点云需要的时间
- 焊接线路，布置走线。M300外接传感器全部完成，连通性测试ok。
- 拆剪杜邦线--热缩管--焊接--吹热缩管
- 时间同步成功，通过修改livox_lidar_ros_driver
- roslauch livox_lidar_ros_driver

12/23

- 点云投影到照片   和    给点云着色    是两个常见的应用来检查雷达、相机融合效果

  点云→照片                  照片的像素RGB信息→点云

12/26

- 手动标定外参，点点云和照片图像时，非常不精确，有3-10pixel的误差。点云有拉丝点。
- ![xx](J:\photo\0.bmp)

1/1

- roslaunch livox_ros_driver 
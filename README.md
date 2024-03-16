# REDP2

## Requirements
### For video processing: 
> python 3.6 
```
pip install opencv-python==4.2.0.32
```
step 2 安装ffmpeg
> https://www.gyan.dev/ffmpeg/builds/
> https://ffmpeg.org/download.html#build-windows
> https://blog.csdn.net/m0_47449768/article/details/130102406 \

1 下载ffmpeg
2 解压, 并将其中bin文件夹, e.g.,
```
C:\ffmpeg-2024-03-14\bin
```
添加进系统路径PATH


关于视频如何提高帧率
> ~~https://blog.csdn.net/Yuan_mingyu/article/details/121908410~~

还是老老实实慢放吧

 
### For ros
Windows下rosbag先用matlab读出来, 然后再用scipy读入python画图
```
pip install scipy
```

### For drones
```
pip install pymavlink
```

### For automata
```
pip install networkx==1.11 numpy==1.19.5 matplotlib==2.2.3 pyyaml==5.4.1
```
Tested
> pyyaml==5.1 (lower than 6.0) \
> matplotlib==2.2.4 \
> numpy==1.21.6 \
> networkx==1.11 \
> opencv-python==4.4.0.44 \



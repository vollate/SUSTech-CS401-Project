## 创建ros包

## 启动雷达

```
roslaunch limo_bringup limo_start.launch pub_odom_tf:=false

/scan: lidar information
```

## gmapping

```
roslaunch limo_bringup limo_gmapping.launch

sub:  /scan
    /tf
    /odom
			
pub:  /map
    /map_metadata
```

目前完成了一半rrt的运行。sentry_bringup会启动雷达与gmapping， rrt_exploration/launch中的single.launch会启动rrt（只有这个启动文件修改了订阅的节点）。
之所以说一半，是因为rrt订阅的global cost map需要move_base节点，然而这个还没跑起来。。。

rrt 订阅/发布的内容可以参考 [github](https://github.com/hasauino/rrt_exploration) 的第四章

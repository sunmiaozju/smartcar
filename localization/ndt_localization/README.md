# ndt_localization

> `desc`: 根据`三维点云地图`、`初始位置估计`以及实时的`里程计`和`激光雷达数据`，计算自身位置(tf: map->base)

parameters: 

- 车高(无激光雷达)：32cm
- 车高(有激光雷达)：48cm (激光雷达高 11cm)
- 车宽(带轮子)：43cm
- 车长(带轮子)：51cm
- 轮距：35.5cm
- 轴距：35.5cm

tf: 

- base_link -> laser : 0.17 0 0.185 0 0 0 0
- base_link -> imu : 0.105 0.16 0.17 0 0 0 0
- base_link -> rr : -0.178 -0.178 -0.165 0 0 0 0

## 测试数据记录

```shell
rosbag record -O 1008_1200_all.bag /lslidar_point_cloud /hall_sensor /raw_imu /wheel_circles /imu/data /imu/odom /tf
```
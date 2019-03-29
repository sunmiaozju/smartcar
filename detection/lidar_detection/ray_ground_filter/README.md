# Ray Ground Filter

An independent ROS_Node which play the role of filtering the ground.

---

SmartCar package for ground filter
* From a sourced terminal:
    * For RC_car:
    `roslaunch ray_ground_filter RCcar_ray_ground_filter.launch`
    * For Yunle_car:
    `roslaunch ray_ground_filter Yunlecar_ray_ground_filter.launch`

### Requirements
Sensor:
> velodyne_driver

Node:
> None

### Subscribed topics
|Topic|Type|Objective|
|---|---|---|
|/velodyne_points|sensor_msgs::PointCloud2|original point-cloud from VLP16|

### Published topics
|Topic|Type|Objective|
|---|---|---|
|/ray_filter/velodyne_points_ground|sensor_msgs::PointCloud2|just the ground-pc that filted out|
|/ray_filter/velodyne_points_costmap|sensor_msgs::PointCloud2|point-cloud with ground filtered and height cropped --this is used to generate costmap|
|/ray_filter/velodyne_points_filtered|sensor_msgs::PointCloud2|point-cloud with ground filtered out|

### Parameters
- [ ] TODO::edit it

### Process
1. `clip_above(CLIP_HEIGHT, current_pc_ptr, cliped_pc_ptr)`
> * This is used to filter out the points that is above the height of CLIP_HEIGHT(for the above points is no use for ground-filter).
> * Besides, the filtered points will be save at `cliped_pc_ptr`, to make a full point cloud(`/ray_filter/velodyne_points_filtered`) together with the no-ground PC(`/ray_filter/velodyne_points_costmap`)

2. `remove_close_pt_rectangle(remove_min_x, remove_min_y, remove_max_x, remove_max_y, cliped_pc_ptr, remove_close)`
> * Remove the close points to eliminate the effects of vehicle body.

3. `XYZI_to_RTZColor(remove_close, organized_points, radial_division_indices, radial_ordered_clouds)`   

```c++
 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
 ```  


4. `classify_pc(std::vector<PointCloudXYZIRTColor>& in_radial_ordered_clouds,pcl::PointIndices& out_ground_indices, pcl::PointIndices& out_no_ground_indices)`(**main function**)
```c++
 * Classifies Points in the PointCoud as Ground and Not Ground 
 * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
 * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
 * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
 ```  

### Reference
[CSDN:激光雷达的地面-非地面分割](https://blog.csdn.net/AdamShan/article/details/82901295)

### Video
<iframe height=450 width=800 src="https://youtu.be/FVgAsm3rm5E" frameborder=0 allowfullscreen></iframe>

If failed to load the video, clip here: [YouTube](https://youtu.be/FVgAsm3rm5E)

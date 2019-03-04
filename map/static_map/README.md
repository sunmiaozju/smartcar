### Static_Map


#### Usage:
> ```
    <node pkg="static_map" type="static_map_node" name="static_map_node">
        <param name="pcd_file" value="$(find static_map)/map_file/map_1010_cluster_filtered.pcd" />
        <param name="map_frame" value="map" />
        <param name="duration" value="1.0" />
        <remap from="/static_map" to="/map/point_cloud" />
    </node>
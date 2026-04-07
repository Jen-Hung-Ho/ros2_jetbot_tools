## Depth‑Camera Vision–Assisted Object Avoidance Self‑Driving (NEW in v2.1):

   - Code logic explanation:

     - 2D Depth‑Image Version —  [depth_vision_avoidance.py](/jetbot_tools/script/depth_vision_avoidance.py)

       - This version reads the camera’s depth image and extracts distance values from key regions in front of the robot. It uses these depth samples to estimate how close obstacles are and chooses a steering direction based on which region shows the most free space. This approach is lightweight, fast, and ideal for simple real‑time avoidance using only 2D depth data.

     - 3D PointCloud Version — [pointcloud_avoidance.py](/jetbot_tools/script/pointcloud_avoidance.py)

       - This version processes full 3D PointCloud2 data to understand the spatial layout of obstacles. It analyzes points within a forward danger zone, estimates the nearest object, and evaluates free‑space volume across left, center, and right sectors to decide the safest direction. This method provides richer spatial awareness and forms the foundation for more advanced 3D navigation in future releases.
         - **Optional: Visualizing the 3D voxel grid in RViz**
            - You can visualize the internal 3D voxel structure used by the PointCloud‑based avoidance pipeline.
            - Recommended reference materials:
               - Nav2 STVL tutorial: https://docs.nav2.org/tutorials/docs/navigation2_with_stvl.html
               - STVL repository: https://github.com/SteveMacenski/spatio_temporal_voxel_layer/
            - With RViz open and `publish_voxel_map: true`, you can inspect the voxel grid using the `{local, global}_costmap/voxel_grid` topics.
               - Set **PointCloud2 Size** to match your voxel size
               - Set **Style** to **Boxes**
               - Use a **neutral color** for clearer visualization
            - This helps verify voxelization behavior, obstacle density, and free‑space structure during 3D avoidance.

     - Use the depth camera to capture a depth image representing the distance of objects in front of the robot

     - Divide the depth image horizontally into three equal regions: Left, Center, and Right

       - Compute the representative distance (mean or median) for each region to estimate how open that direction is

       - Compare the three region distances and select the region with the farthest open area

      - (Optional) When depth‑LiDAR fusion is enabled, enhance the depth image using synchronized LiDAR data for more reliable distance estimation

   - Source code:

     - [launch file: depth_vision_avoidance.launch.py](/jetbot_tools/launch/depth_vision_avoidance.launch.py) 

     - [ROS2 node: depth_vision_avoidance.py](/jetbot_tools/script/depth_vision_avoidance.py) 

     - [launch file: pointcloud_avoidance.launch.py](/jetbot_tools/launch/pointcloud_avoidance.launch.py) 

     - [ROS2 node: pointcloud_avoidance.py](/jetbot_tools/script/pointcloud_avoidance.py) 

   - Usage:

     - 2D Depth‑Image Version

       - ros2 launch jetbot_tools depth_vision_avoidance.launch.py

       - ros2 param get /depth_vision_avoidance start

       - ros2 param set /depth_vision_avoidance start true

     - 3D PointCloud Version

       - ros2 launch jetbot_tools pointcloud_avoidance.launch.py

       - ros2 param get /pointcloud_avoidance start

       - ros2 param set /pointcloud_avoidance start true 
   - Example RViz visualization:
     <br>
    [<img src="https://img.youtube.com/vi/2WhUFiI1UmU/hqdefault.jpg" width="300" height="200"
/>](https://youtu.be/2WhUFiI1UmU) [<img src="https://img.youtube.com/vi/wy3AIB81d3M/hqdefault.jpg" width="300" height="200"
/>](https://www.youtube.com/shorts/wy3AIB81d3M)


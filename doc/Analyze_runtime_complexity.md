Analyze the runtime complexity of the algorithms.

### `tf_broadcaster.cpp`

The `tf_broadcaster.cpp` script essentially subscribes to a ROS topic, processes incoming messages, and publishes a transformation.

1. **Subscription and Message Handling (`handle_pose` function)**:
   - The node subscribes to `/a2rl/state_estimation/ego_loc_fused`.
   - Each time a message is received, it extracts the position and orientation and creates a `TransformStamped` message.
   - It then broadcasts the transformation using `tf_broadcaster_->sendTransform(transform)`.

The complexity of handling each message:
- Extracting and copying data from `msg` to `transform` is \(O(1)\) since it involves a fixed number of operations (assignments and copies).
- Broadcasting the transform is also \(O(1)\).

Thus, the complexity for handling each incoming message is \(O(1)\).

### `process_data.cpp`

The `process_data.cpp` script processes point cloud data, transforms it, and accumulates it over time.

1. **Subscription and Message Handling (`handlePointCloud` function)**:
   - Subscribes to the `/sensor/lidar_front/points` topic.
   - Transforms the point cloud data from the `lidar_front` frame to the `map` frame.
   - Accumulates the transformed point cloud.
   - Applies a voxel grid filter to downsample the accumulated point cloud.

	The complexity of handling each point cloud message:
	- **Transforming Point Cloud**: Assuming the point cloud contains \(n\) points, transforming the entire cloud involves iterating through all \(n\) points and applying a transformation matrix, which is \(O(n)\).
	- **Accumulating Point Clouds**: Adding the transformed cloud to the accumulated cloud is \(O(n)\).
	- **Voxel Grid Filtering**: This step can vary in complexity, but typically it's \(O(n \log n)\) due to sorting operations involved in creating the voxel grid.

2. **Periodic Publishing (`publishAccumulatedPointCloud` function)**:
   - Converts the accumulated point cloud (post voxel filter) to a ROS message.
   - Publishes the point cloud.

	The complexity of periodic publishing:
	- **Converting Point Cloud**: Converting a point cloud of size \(m\) to a ROS message is \(O(m)\).
	- **Publishing**: The complexity is also \(O(m)\), where \(m\) is the size of the accumulated point cloud.

### Overall Runtime Complexity

**`tf_broadcaster.cpp`**:
- Handling each message is \(O(1)\).
- If we receive \(N\) messages, the total complexity is \(O(N)\).

**`process_data.cpp`**:
- For each point cloud message, the handling complexity is \(O(n + n + n \log n) = O(n \log n)\).
- If we process \(M\) point cloud messages, the total complexity is \(O(M \cdot n \log n)\), where \(n\) is the number of points per point cloud message.

These complexities indicate that `process_data.cpp` is more computationally intensive due to point cloud transformations and filtering, while `tf_broadcaster.cpp` is relatively lightweight.
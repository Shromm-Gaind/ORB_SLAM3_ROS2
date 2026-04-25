# Stereo publishers + TF for zang09/ORB_SLAM3_ROS2

This is a drop-in modification of the **stereo** node only, in
[zang09/ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2).
It adds the publishing layer that the suchetanrs wrapper exposes
(pose, map points, keyframes, TF tree) **without pulling in any
of the CUDA / FastTrack code**, so it runs CPU-only on a Pi 5.

## What you get

| Topic                       | Type                              | Notes                                                      |
|-----------------------------|-----------------------------------|------------------------------------------------------------|
| `/orb_slam3/pose`           | `geometry_msgs/PoseStamped`       | Current camera pose in `map`, every successful frame.      |
| `/orb_slam3/tracked_points` | `sensor_msgs/PointCloud2` (XYZ)   | Map points actually tracked in the *current* frame.       |
| `/orb_slam3/map_points`     | `sensor_msgs/PointCloud2` (XYZ)   | All map points in the active map. Throttled (default 1/10 frames). |
| `/orb_slam3/keyframes`      | `geometry_msgs/PoseArray`         | All keyframes in the active map. Throttled.                |

TF (when `publish_tf:=true`):

* `map -> base_link` directly (when `odometry_mode:=false`)
* `map -> odom` correction (when `odometry_mode:=true`, requires an external `odom -> base_link` publisher)
* `base_link -> camera_link` static (identity by default — override from your URDF)

## Files

```
src/stereo/stereo-slam-node.hpp     <- replaces upstream
src/stereo/stereo-slam-node.cpp     <- replaces upstream
CMakeLists.txt                      <- replaces upstream (adds tf2/eigen/etc deps for stereo target)
package.xml                         <- replaces upstream
patches/System.h.patch.txt          <- one-line patch to ORB-SLAM3 itself
```

The `src/stereo/stereo.cpp` file from upstream is left unchanged.

## Apply

1. Patch your ORB-SLAM3 fork (zang09/ORB-SLAM3-STEREO-FIXED) per
   `patches/System.h.patch.txt` and rebuild it (`./build.sh`).
2. Rebuild the wrapper:
   ```
   cd ~/colcon_ws
   colcon build --symlink-install --packages-select orbslam3
   source install/setup.bash
   ```
3. Run as usual:
   ```
   ros2 run orbslam3 stereo \
       /path/to/ORBvoc.txt \
       /path/to/your_stereo.yaml \
       false \
       --ros-args \
         -p map_frame_id:=map \
         -p odom_frame_id:=odom \
         -p base_frame_id:=base_link \
         -p camera_frame_id:=camera_link \
         -p publish_tf:=true \
         -p odometry_mode:=false \
         -p map_publish_rate_div:=10
   ```

## Parameters

| Parameter              | Default      | Meaning                                                                      |
|------------------------|--------------|------------------------------------------------------------------------------|
| `map_frame_id`         | `map`        | Global SLAM frame.                                                           |
| `odom_frame_id`        | `odom`       | Odometry frame (only used if `odometry_mode:=true`).                         |
| `base_frame_id`        | `base_link`  | Robot body frame.                                                            |
| `camera_frame_id`      | `camera_link`| Camera body frame, child of base via static identity TF.                     |
| `publish_tf`           | `true`       | Publish the TF tree from SLAM.                                               |
| `odometry_mode`        | `false`      | If `true`, publish `map->odom` and expect external `odom->base_link`. If `false`, publish `map->base_link` directly. |
| `map_publish_rate_div` | `10`         | Publish heavy `/map_points` and `/keyframes` once every N tracked frames.    |

## Frame conventions — important

ORB-SLAM3 internally uses the camera-optical convention
(`x_right, y_down, z_forward`). ROS REP-103 uses
`x_forward, y_left, z_up`. The node applies a fixed rotation
`R_ros_orb` to convert poses and points before publishing, so
everything you see on the wire is in standard ROS axes.

The SLAM "world" frame is wherever the camera was on the first
successful initialisation, but expressed in ROS axes. If you want
this to align with a real-world frame (e.g. gravity-aligned), the
cleanest way is to mount the camera so its first frame's optical
axis matches your desired map orientation, *or* publish a static
TF from `map_world` to whatever your robot considers world.

## How the heavy publishing is kept Pi-friendly

* `tracked_points` is cheap (only the points associated with the
  current frame, typically a few hundred) — published every frame.
* `map_points` and `keyframes` are throttled by
  `map_publish_rate_div` (default: 1 in 10 frames). On a 10–20 Hz
  stereo stream that's 1–2 Hz of map publishing, which RViz handles
  fine and barely registers on a Pi 5's CPU.
* All publishers gate on `get_subscription_count() > 0` so when
  nobody is listening, the cost is a single integer compare.
* No copies of the cv::Mat images are made beyond what upstream
  already did.

## Caveats / things to know

1. **Loop closures move the whole map.** If you publish `map_points`
   at, say, 1 Hz and a loop closure fires, the whole point cloud
   will jump in RViz on the next publish. This is correct ORB-SLAM3
   behaviour, not a bug in the wrapper.

2. **`map_publish_rate_div` is a frame counter, not a wall-clock
   timer.** If tracking is lost, no map is published until tracking
   recovers. This is intentional — when tracking is lost the map
   isn't being updated anyway.

3. **`odometry_mode:=true` needs an external `odom->base_link`
   publisher.** If the lookup fails the node falls back to
   republishing the last good `map->odom` with a fresh stamp, and
   throttle-warns. If you don't have wheel/IMU odometry, just leave
   `odometry_mode:=false` and the node will publish `map->base_link`
   directly.

4. **`base_link->camera_link` is published as identity by default.**
   In a real robot you should remove this from the constructor and
   let your URDF / robot_state_publisher own that transform. The
   identity default is just so the TF tree is connected for simple
   bring-up.

5. **Tracking state check.** The node uses `GetTrackingState() == 2`
   (== `Tracking::OK`) to gate publishing. While initialising or
   after a loss, nothing is published.
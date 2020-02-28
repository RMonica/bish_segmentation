2020-02-27

BiSH Segmentation
=================

This is a ROS-compatible implementation of a categorization and semantic segmentation algorithm for point clouds, based on projective analysis.
The implementation was developed by Davide Piccinini and Riccardo Monica, at the Robotics and Intelligent Machines Laboratory (RIMLab), Department of Engineering and Architecture, University of Parma, Italy.

The segmentation method was originally described in:

- Yunhai Wang, Minglun Gong, Tianhua Wang, Daniel Cohen-Or, Hao Zhang, Baoquan Chen, "Projective Analysis for 3D Shape Segmentation", ACM Transactions on Graphics 32, 6, Article 192, November 2013. <https://doi.org/10.1145/2508363.2508393>

The segmentation algorithm is based on projective analysis, that is:

- The input (unlabeled) point cloud is projected as if observed by a set of viewpoints. For each viewpoint, a binary image is generated.
- Each binary image is segmented, and a label is assigned to each pixel.
- Labels are backprojected on the input point cloud and assigned to each point.

Binary images are segmented by comparison against a dataset of annotated images of objects in the same category. The closest image is found by using the Bi-class Symmetric Hausdorff distance (BiSH). The segmentation is semantically consistent with the annotation in the dataset. Partial code for the BiSH distance was provided by the original authors, downloadable at <http://www.yunhaiwang.net/public_html/psa.html>.

The algorithm implementation was completed with the full point cloud segmentation pipeline. Moreover, the following features were added:

- Dataset generation from a set of annotated point clouds in PCD format.
- Object point cloud categorization.
- Use of prototype images for faster categorization.
- ROS integration with action-based interface.

Associated paper:

- to appear.

### Installation

Dependencies:

- ROS melodic (Ubuntu 18.04)
- Eigen 3 math library
- PCL (Point Cloud Library)
- OpenCV 3
- OpenGL 3.3
- GLEW
- Init Fake OpenGL Context: <https://github.com/RMonica/init_fake_opengl_context>

The repository contains three ROS packages: `bish_segmentation_dataset_creation`, `bish_segmentation` and `bish_segmentation_msgs`.
Copy them in your ROS workspace and compile them using `catkin build` or `catkin_make`.

**Warning**: C++ compiler optimization should be enabled for better performance. The package should be compiled in `Release` or `RelWithDebInfo` mode.

### Dataset generation

The ROS package `bish_segmentation_dataset_creation` generates a dataset of annotated images from a dataset of annotated point clouds.
Point clouds are labeled using the RGB color property: each color corresponds to a different label.

The point clouds must be placed in a sub-folder named `labeled_pointclouds` in the `data` folder of the `bish_segmentation_dataset_creation` package. Point clouds must be split into sub-folders according to their category.

For example, given two categories `cups` and `hammers`, the directory structure may be as follows:

```
- bish_segmentation_dataset_creation
  - data
    - labeled_pointclouds
      - cups
        - cup1.pcd
        - teacup.pcd
        - ... etc.
      - hammers
        - mallet.pcd
        - maul.pcd
        - ... etc.
```

Then, launch the `bish_segmentation_dataset_creation` node using the provided launch file:

```
  roslaunch bish_segmentation_dataset_creation create.launch
```

This may take some time, depending on the size of your dataset.

After generation, two sub-folders are generated in the `data` directory: `labeled_images` and `prototypes`. `labeled_images` contains the projected images for segmentation. `prototypes` contains a subset of `labeled_images`, selected as prototypes for faster categorization.

The `labeled_images` and the `prototypes` folders should be moved to the `data` folder of the `bish_segmentation` package, to be used as dataset.

#### Parameters

- `labeled_pointclouds_path` (string): path of the `labeled_pointclouds` folder.
- `labeled_images_path` (string): path of the `labeled_images` folder.
- `prototypes_path` (string): path of the `prototypes` folder.
- `labeled_segment_number` (int): number of horizontal slabs for BiSH segmentation. Increase for more complex objects.
- `point_radius` (float): point radius in the input point cloud. Increase if the gap between surfels is visible in the resulting images.
- `normal_k_search_count` (int): k-search neighborhood count for normal estimation.
- `image_height` (int): generated image height, in pixels.
- `image_width` (int): generated image width, in pixels.
- `range_min` (float): projection near plane.
- `range_max` (float): projection far plane. It should be much larger than object size.
- `theta_x`, `theta_y`, `theta_z` (float): view sampling steps, in degrees, around the `x`, `y` and `z` axes. Set any of them to `0` to disable rotation around the corresponding axis.
- `prototypes_count` (int): number of prototypes to be generated for each class.

### Batch classification and segmentation

The ROS package `bish_segmentation` classifies and segments input unlabeled point clouds. This package requires a dataset in the `data` folder. The dataset can be generated as explained in the previous section.

In batch mode, the `bish_segmentation` node classifies and segments multiple PCD files, which are placed in the `test_pointclouds` sub-folder in the `data` folder. The directory structure should be as follows:

```
- bish_segmentation
  - data
    - labeled_images    # dataset
    - prototypes        # dataset
    - test_pointclouds
      - unlabeled_object.pcd
      - unknown_object.pcd
      - ... etc.
```

Then, launch the `bish_segmentation` node using the provided launch file:

```
  roslaunch bish_segmentation classify_and_segment.launch
```

The node creates several temporary files in `data/test_images` and `data/results`. These folders may be safely removed when the node terminates. The segmented files are generated in `results_pointclouds`.

For each input point cloud, e.g. `unknown.pcd`, two possible segmented point clouds are generated: `unknown-LABELED.pcd` and `unknown-2_LABELED.pcd`. The latter is processed with an additional label optimization step, which should reduce label noise.

#### Parameters

- `labeled_images_path` (string): path of the `labeled_images` folder.
- `prototypes_path` (string): path of the `prototypes` folder.
- `test_pointclouds_path` (string): path of the `test_pointclouds` folder.
- `test_images_path` (string): path of the temporary `test_images` folder.
- `results_path` (string): path of the temporary `result` folder.
- `results_pointclouds_path` (string): path of the `results_pointclouds` folder.
- `test_segment_number` (int): number of horizontal slabs for BiSH segmentation. It should be about twice the `labeled_segment_number` used during dataset generation.

The following parameters have the same meaning as in `bish_segmentation_dataset_creation`:
`point_radius`, `normal_k_search_count`, `image_height`, `image_width`, `range_min`, `range_max`, `theta_x`, `theta_y`, `theta_z`.

### On-demand classification and segmentation

For integration in a robot system, we developed an action-based interface for the `bish_segmentation` package. The action definition is `bish_segmentation_msgs/BishSegment.action`.

To use the action-based interface, launch the persistent `action_server` node:
```
  roslaunch bish_segmentation action_server.launch
```

Fields in the action goal:

- `cloud` (`sensor_msgs/PointCloud2`): the point cloud. Points should be of type `pcl::PointSurfel`. Radius and normal must be provided.
- `view_poses` (`geometry_msgs/Pose[]`): array of poses, a binary image is generated by projection to each pose. If empty, the poses are generated automatically using steps `theta_x`, `theta_y`, `theta_z`.
- `back_face_culling` (bool): if true, surfels observed from behind are invisible when generating projections. If the point cloud is incomplete (e.g. obtained from the observation of the object from only one side), this should be set to true.

Fields in the action result:

- `segmented_cloud` (`sensor_msgs/PointCloud2`): segmented cloud, where colors correspond to labels.
- `segmented_class` (string): category of the object, corresponding to the sub-folder name during dataset generation.

**Note:** temporary folders such as `data/test_images` and `data/results` are used by the action-based interface as well.

#### Parameters

All parameters of the `bish_segmentation` node, and:

- `action_name` (string): action name. Default: `~bish_segment`.
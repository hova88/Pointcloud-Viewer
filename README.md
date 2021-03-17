# Point Cloud Viewer
A light and easy to change point cloud / lidar visualization tool, and supports multiple formats of data types.
## Features

Features                                                                        | description
-------                                                                         | -----------
**[clouds_one_by_one](##clouds_one_by_one)**                                    | Play the point cloud in  one frame and press `q` for the next frame
**[clouds_all_in_one](##clouds_all_in_one)**                                    | **All** the point clouds in `DATA_PATH` are played in one window (Note: there is no registration for point clouds)
**[clouds_one_by_one_with_boxes](##clouds_one_by_one_with_boxes)**              | Similar to `clouds_one_by_one`,just only with bounding boxes.
**[clouds_all_in_one_with_boxes](##clouds_all_in_one_with_boxes)**              | Set the first frame in `DATA_PATH`as the base map, place all bounding boxes in the same window, different frames of boxes different colors
--------

## Description 

```bash
├── boxes            <----- Usually, `BOXES_PATH` is the storage path of boxes
│   ├── 000.npz    
│   └── 001.npz
├── clouds           <----- Usually, `DATA_PATH` is the storage path of clouds
│   ├── 000.npz
│   └── 001.npz
├── docs
├── PointCloudVis.py <----- The feature class
├── README.md
├── src              <----- You can expand the custom function here, but must use the API of "open3d"
│   ├── __init__.py
│   ├── open3d_arrow.py   
│   ├── open3d_box.py
│   └── open3d_coordinate.py
└── viewer.py        <----- The main function
```

## Install
```bash
# Install Open3D stable release with pip
$ pip install open3d

# Install Open3D stable release with Conda
$ conda install -c open3d-admin -c conda-forge open3d

# Test the installation
$ python -c "import open3d as o3d; print(o3d)"
```

## clouds_one_by_one
```bash
python viewer.py --data_path clouds
```
<p align="left">
  <img width="1200" alt="fig_method" src=docs/clouds_one_by_one.png>
</p>

## clouds_all_in_one
```bash
#The color scheme of the current version of point cloud has changed
python viewer.py --data_path clouds --all
```

<p align="left">
  <img width="1200" alt="fig_method" src=docs/clouds_all_in_one.png>
</p>


## clouds_one_by_one_with_boxes
```bash
python viewer.py --data_path clouds --boxes_path boxes --boxes  
```

<p align="left">
  <img width="1200" alt="fig_method" src=docs/with_boxes_one_by_one.png>
</p>

## clouds_all_in_one_with_boxes
```bash
python viewer.py --data_path clouds --boxes_path boxes --boxes --all
```

<p align="left">
  <img width="1200" alt="fig_method" src=docs/with_boxes_all_in_one.png>
</p>

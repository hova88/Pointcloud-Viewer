"""
author: hova88
date: 2021/03/16
"""
import numpy as np 
import numpy as np
from PointCloudVis import PointCloudVis
import open3d as o3d


def loader_datalist_to_args(args):
    ext = args.clouds_file_list[0][-4:]
    args.draw_pcd = False
    if  ext == ".npz":
        clouds_list = [np.load(cloud_file)["points"].reshape(-1, args.point_dims) for cloud_file in args.clouds_file_list]
    elif ext == ".npy" :
        clouds_list = [np.load(cloud_file).reshape(-1, args.point_dims) for cloud_file in args.clouds_file_list]
    elif ext == ".bin":
        clouds_list = [np.fromfile(cloud_file).reshape(-1, args.point_dims) for cloud_file in args.clouds_file_list]
    elif ext == ".txt":
        clouds_list = [np.loadtxt(cloud_file).reshape(-1, args.point_dims) for cloud_file in args.clouds_file_list]
    elif ext in [".pcd", ".ply", ".xyz", ".xyzrgb", ".xyzn", ".pts"]:
        clouds_list = [o3d.io.read_point_cloud(cloud_file)for cloud_file in args.clouds_file_list]
        args.draw_pcd = True
    else:
        raise NotImplemented

    ext = args.boxes_file_list[0][-4:]
    if  ext == ".npz":
        boxes_list = [np.load(box_file)["gt_boxes"].reshape(-1,7) for box_file in args.boxes_file_list]
    elif ext == ".npy" :
        boxes_list = [np.load(box_file).reshape(-1,7) for box_file in args.boxes_file_list]
    elif ext == ".bin":
        boxes_list = [np.fromfile(box_file).reshape(-1,7) for box_file in args.boxes_file_list]
    elif ext == ".txt":
        boxes_list = [np.loadtxt(box_file).reshape(-1,7)for box_file in args.boxes_file_list]
    else:
        raise NotImplemented

    args.clouds_list = clouds_list
    args.boxes_list = boxes_list
    


def draw_clouds(args):
    V = PointCloudVis(args)

    if args.all_cloud_in_one_image:
        V.draw_clouds_all_in_one_frame( args.clouds_list , args.draw_pcd) 
    else:
        V.draw_clouds_one_by_one_frame( args.clouds_list , args.draw_pcd)

def draw_clouds_with_boxes(args):
    V = PointCloudVis(args)
    if args.all_cloud_in_one_image:
        V.draw_clouds_with_boxes_all_in_one_frame(args.clouds_list , args.boxes_list ,args.draw_pcd) 
    else:
        V.draw_clouds_with_boxes_one_by_one_frame(args.clouds_list , args.boxes_list ,args.draw_pcd) 





if __name__ == "__main__":
    import argparse
    import glob
    from pathlib import Path

    parser = argparse.ArgumentParser(description="arg parser")
    parser.add_argument(
        "--data_path",
        type=str,
        default="clouds",
        help="specify the config for demo",
    )
    parser.add_argument(
        "--boxes_path",
        type=str,
        default="boxes",
        help="specify the config for demo",
    )
    parser.add_argument(
        "--point_dims",
        type=int,
        default=4,
        help="specify the number of clouds you want to draw",
    )
    parser.add_argument(
        "-pc",
        "--points_colorbar",
        type=list,
        default=None,
        help="specify the color of clouds you want to draw, such as [[1,0,0],[0,1,0],[0,0,1]]",
    )
    parser.add_argument(
        "-bc",
        "--boxes_colorbar",
        type=list,
        default=None,
        help="specify the color of boxes you want to draw , such as [[1,0,0],[0,1,0],[0,0,1]]",
    )
    parser.add_argument(
        "--boxes",action="store_true", default=False, help="Draw boxes and points"
    )

    parser.add_argument(
        "-all",
        "--all_cloud_in_one_image",
        action="store_true",
        default=False,
        help="Draw all frame in a single image",
    )

    args = parser.parse_args()
    data_path = Path(args.data_path)
    boxes_path = Path(args.boxes_path)

    clouds_file_list = (glob.glob(str(data_path /"*")) if data_path.is_dir() else [data_path])
    boxes_file_list  = (glob.glob(str(boxes_path /"*")) if boxes_path.is_dir() else [boxes_path])

    clouds_file_list.sort()
    boxes_file_list.sort()

    print("the CLOUDS file list is : \n ==> ",clouds_file_list)
    print("the BOXES file list is : \n ==> ",boxes_file_list)

    args.clouds_file_list = clouds_file_list
    args.boxes_file_list = boxes_file_list

    loader_datalist_to_args(args)
    if boxes_file_list == [] or not args.boxes:
        draw_clouds(args)
    else:
        draw_clouds_with_boxes(args)


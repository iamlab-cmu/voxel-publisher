import open3d as o3d
import rospy
import time
import argparse
import numpy as np

from sensor_msgs.msg import PointCloud2
from utils import convertCloudFromRosToOpen3d
from autolab_core import RigidTransform
from voxel_msgs.msg import VoxelList,Voxel

def visualize(pcd_list):
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0,0,0])

    o3d.visualization.draw_geometries(pcd_list + [mesh_frame])

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--extrinsics_file_path', type=str, default='/home/kevin/Documents/camera-calibration/calib/azure_kinect_overhead/azure_kinect_overhead_to_world.tf') 
    args = parser.parse_args()

    azure_kinect_to_world_transform = RigidTransform.load(args.extrinsics_file_path)

    rospy.init_node('voxel_publisher')

    voxel_pub = rospy.Publisher('/voxels', VoxelList, queue_size=10)

    def make_voxel(data):
        v = Voxel()
        v.x = data[0]
        v.y = data[1]
        v.z = data[2]
        v.r = data[3]
        v.g = data[4]
        v.b = data[5]
        return(v)

    def publishVoxelList(pc2_msg):
        open3d_pc = convertCloudFromRosToOpen3d(pc2_msg)
        open3d_pc.transform(azure_kinect_to_world_transform.matrix)
        cropped_pc = open3d_pc.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=np.array([-0.1,-0.6,-0.1]), max_bound=np.array([0.9,0.6,0.8])))
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(cropped_pc,voxel_size=0.01)

        voxel_list_msg = VoxelList()

        voxel_list_msg.voxel_list = [make_voxel(np.concatenate([voxel_grid.origin + pt.grid_index*voxel_grid.voxel_size, pt.color])) for pt in voxel_grid.get_voxels()]
        voxel_pub.publish(voxel_list_msg)

    rospy.Subscriber("/points2", PointCloud2, publishVoxelList)

    rospy.spin()  
import hydra
# import open3d as o3d
from pathlib import Path
from digit_depth.third_party import geom_utils
from digit_depth.digit import DigitSensor
from digit_depth.train import MLP
from digit_depth.train.prepost_mlp import *
from attrdict import AttrDict
from digit_depth.third_party import vis_utils
from digit_depth.handlers import find_recent_model, find_background_img

# import ROS stuff
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
# import numpy as np


seed = 42
torch.seed = seed
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
base_path = Path(__file__).parent.parent.parent.resolve()

print('ciao0')


@hydra.main(config_path=f"{base_path}/config", config_name="digit.yaml", version_base=None)
def show_point_cloud(cfg):
    # view_params = AttrDict({'fov': 60, 'front': [-0.1, 0.1, 0.1], 'lookat': [
    #     -0.001, -0.01, 0.01], 'up': [0.04, -0.05, 0.190], 'zoom': 2.5})
    view_params = AttrDict({
                "fov": 60,
                "front": [-0.3, 0.0, 0.5],
                "lookat": [-0.001, 0.001,-0.001],
                "up": [0.0, 0.0, 0.50],
                "zoom": 0.5,
            })
    vis3d = vis_utils.Visualizer3d(base_path=base_path, view_params=view_params)

    print('ciao2')
    # projection params
    proj_mat = torch.tensor(cfg.sensor.P)
    model_path = find_recent_model(f"{base_path}/models")
    model = torch.load(model_path).to(device)
    model.eval()
    # base image depth map
    background_img_path = find_background_img(base_path)
    background_img = cv2.imread(background_img_path)
    background_img = preproc_mlp(background_img)
    background_img_proc = model(background_img).cpu().detach().numpy()
    background_img_proc, _ = post_proc_mlp(background_img_proc)
    # get gradx and grady
    gradx_base, grady_base = geom_utils._normal_to_grad_depth(img_normal=background_img_proc, gel_width=cfg.sensor.gel_width,
                                                              gel_height=cfg.sensor.gel_height, bg_mask=None)

    # reconstruct depth
    img_depth_base = geom_utils._integrate_grad_depth(gradx_base, grady_base, boundary=None, bg_mask=None,
                                                      max_depth=0.0237)
    img_depth_base = img_depth_base.detach().cpu().numpy() # final depth image for base image
    # setup digit sensor
    digit = DigitSensor(cfg.sensor.fps, cfg.sensor.resolution, cfg.sensor.serial_num)
    digit_call = digit()

    #setup publisher
    pub = rospy.Publisher('/digit/pointcloud', PointCloud2, queue_size=10)

    print('ciao3')
    while True:
        print('ciao4')
        frame = digit_call.get_frame()
        img_np = preproc_mlp(frame)
        img_np = model(img_np).detach().cpu().numpy()
        img_np, _ = post_proc_mlp(img_np)
        # get gradx and grady
        gradx_img, grady_img = geom_utils._normal_to_grad_depth(img_normal=img_np, gel_width=cfg.sensor.gel_width,
                                                                gel_height=cfg.sensor.gel_height,bg_mask=None)
        # reconstruct depth
        img_depth = geom_utils._integrate_grad_depth(gradx_img, grady_img, boundary=None, bg_mask=None, max_depth=cfg.max_depth)
        view_mat = torch.eye(4)  # torch.inverse(T_cam_offset)
        # Project depth to 3D
        points3d = geom_utils.depth_to_pts3d(depth=img_depth, P=proj_mat, V=view_mat, params=cfg.sensor)
        points3d = geom_utils.remove_background_pts(points3d, bg_mask=None)
        # cloud = o3d.geometry.PointCloud()
        # clouds = geom_utils.init_points_to_clouds(clouds=[copy.deepcopy(cloud)], points3d=[points3d])
        # vis_utils.visualize_geometries_o3d(vis3d=vis3d, clouds=clouds)

        # Create ROS message for PointCloud2
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "yumi_base_link"  # Set appropriate TF frame. MODIFY THIS AS NEEDED

        # Converts from Point3d to PointCloud2 format. 
        points = [(point[0], point[1], point[2]) for point in points3d]

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            #PointField('rgba', 12, PointField.UINT32, 1), Potentially need to add one more field for RGB or intensity
        ] 

        point_cloud = pc2.create_cloud(header, fields, points)

        # Publish the point cloud to the ROS topic
        pub.publish(point_cloud)

        return point_cloud

if __name__ == "__main__":
    rospy.init_node('digit_pointcloud', anonymous=True)
    print('ciao1')
    show_point_cloud()
import os
from hydra import initialize, compose
from pathlib import Path
from digit_depth.third_party import geom_utils
from digit_depth.digit import DigitSensor
from digit_depth.train import MLP
from digit_depth.train.prepost_mlp import *
from attrdict import AttrDict
from digit_depth.handlers import find_recent_model, find_background_img

# import ROS stuff
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

seed = 42
torch.seed = seed
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
relative_config_path = os.path.join('..','config')

# Build config file path to retrieve .yaml file
current_dir = os.path.dirname(os.path.abspath(__file__))
base_path = os.path.abspath(os.path.join(current_dir, 'digit-depth'))
model_path_weights = os.path.abspath(os.path.join(current_dir, 'digit-depth','models'))

# print('base_path:', base_path)
# print('model_path_weights:', model_path_weights)

def show_point_cloud(cfg):
    
    # projection params
    proj_mat = torch.tensor(cfg.sensor.P)
    model_path = find_recent_model(f"{model_path_weights}")
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

    # setup publisher
    pub = rospy.Publisher('pointcloud', PointCloud2, queue_size=2)

    while not rospy.is_shutdown():
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
        header.frame_id = "map"  # Set appropriate TF frame. MODIFY THIS AS NEEDED

        # Converts from Point3d to PointCloud2 format. 
        points = list(zip(*points3d.tolist()))
        # print(points)
        # print(points3d.shape)

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            # PointField('rgba', 12, PointField.UINT32, 1), # Potentially need to add one more field for RGB or intensity
        ] 

        point_cloud = pc2.create_cloud(header, fields, points)

        # Publish the point cloud to the ROS topic
        pub.publish(point_cloud)

if __name__ == "__main__":
    # Retrieve config file name from ROS parameter server. Provides default value if not set
    config_file_name = rospy.get_param("~config_file_name", "digit_D20951.yaml")
    
    with initialize(config_path=relative_config_path):
        cfg = compose(config_name=config_file_name)

    rospy.init_node('digit_pointcloud', anonymous=True)

    show_point_cloud(cfg)
import os
import sys
from omegaconf import OmegaConf
current_dir = os.path.dirname(os.path.abspath(__file__))

other_dir = os.path.abspath(os.path.join(current_dir, 'digit-depth','scripts'))
other_dir_2 = os.path.abspath(os.path.join(current_dir, 'digit-depth','scripts','ros'))
sys.path.append(current_dir)
sys.path.append(other_dir)
sys.path.append(other_dir_2)

# print all the directories in sys.path
print(sys.path)

# Creazione della configurazione hardcoded
cfg = OmegaConf.create({
    "sensor": {
        "serial_num": "D20928",
        "resolution": "QVGA",  # Opzioni: QVGA, VGA
        "fps": 30,  # Max è 60
        "T_cam_offset": [
            [2.22e-16, 2.22e-16, -1.00e+00, 0.00e+00],
            [-1.00e+00, 0.00e+00, -2.22e-16, 0.00e+00],
            [0.00e+00, 1.00e+00, 2.22e-16, 1.50e-02],
            [0.00e+00, 0.00e+00, 0.00e+00, 1.00e+00]
        ],
        "P": [
            [2.30940108, 0.0, 0.0, 0.0],
            [0.0, 1.73205081, 0.0, 0.0],
            [0.0, 0.0, -1.04081633, -0.00204081633],
            [0.0, 0.0, -1.0, 0.0]
        ],
        "z_near": 0.001,
        "z_far": 0.05,
        "gel_width": 0.01835,
        "gel_height": 0.02460,
        "gel_thickness": 0.0046,
        "gel_min_depth": 0.01954
    },
    "mm_to_pixel": 17.15,
    "ball_diameter": 6.0,
    "max_depth": 0.02076,
    "visualize": {
        "normals": True,
        "points3d": True,
        "ellipse": True
    },
    "dataloader": {
        "batch_size": 1,
        "shuffle": False,
        "num_workers": 8,
        "annot_flag": True,
        "annot_file": "annotate.csv"
    },
    "dataset": {
        "dataset_type": "imgs",
        "save_dataset": True,
        "save_depth": True
    }
})

from depth import show_depth
from record import record_frame
from digit_image_pub import rgb_pub

if __name__ == "__main__":
    print("starting...")
    rgb_pub(cfg)
    # record_frame(cfg)
    # show_depth(cfg)
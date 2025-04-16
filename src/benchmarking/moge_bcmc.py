import rclpy
from pathlib import Path
import numpy as np
import os, sys
import time
from tqdm import tqdm

ws_dir = os.getenv("ROS_WS_DIR", "/external/smores_drone_software")
# sys.path.append("/opt/conda/lib/python3.10/site-packages/")
sys.path.append(f"{ws_dir}/include")
sys.path.append(f"{ws_dir}/include/MoGe")
from moge.model.v1 import MoGeModel
from moge.utils.vis import colorize_depth

import cv2
import torch
from cv_bridge import CvBridge

if torch.cuda.is_available():
    device = torch.device("cuda")
    print(f"MogeInferenceMono.py: Loaded GPU successfully")

else:
    device = torch.device("cpu")
    print(f"MogeInferenceMono.py: Loaded from the CPU. GPU unavailable")
            
model = MoGeModel.from_pretrained(f"{ws_dir}/include/MoGe/moge/model/weights/model.pt").to(device)
        
print(f"MogeInferenceMono.py: Initialized MogeInference node successfully")

bridge = CvBridge()
img_path = Path("/workspace/smores_drone_software/data/airlab1_images")

total_time = 0
inf_time = 0
colorspace_time = 0
imgmsg_time = 0

for idx in tqdm(range(100)):
    
    start = time.time_ns()
    cv2_left_rgb = cv2.cvtColor(cv2.imread(str(img_path / f"left_{idx}.png")), cv2.COLOR_BGR2RGB)
    cv2_right_rgb = cv2.cvtColor(cv2.imread(str(img_path / f"right_{idx}.png")), cv2.COLOR_BGR2RGB)
    
    
    cv2_left_tensor = torch.tensor(cv2_left_rgb / 255, dtype=torch.float16, device=device).permute(2, 0, 1)   #(channels, length, width)
    cv2_right_tensor = torch.tensor(cv2_right_rgb / 255, dtype=torch.float16, device=device).permute(2, 0, 1) #(channels, length, width)
    
    input_tensor = torch.stack([cv2_left_tensor, cv2_right_tensor])
    
    # print(f"MogeInferenceMono.py: About to run model")
    
    output = model.infer(input_tensor, fov_x=95, resolution_level=1)
    depth = output["depth"].cpu().numpy()
    
    # print(f"MogeInferenceMono.py: Model inference complete")
    # height, width = cv2_left_rgb.shape[:2]
    # new_width = int(width/2)
    # height, width = min(new_width, int(new_width * height / width)), min(new_width, int(new_width * width / height))
    # cv2_left_rgb = cv2.resize(cv2_left_rgb, (width, height), cv2.INTER_AREA)
    # cv2_right_rgb = cv2.resize(cv2_right_rgb, (width, height), cv2.INTER_AREA)

    model_inf_time = time.time_ns()
    
    processed_depthmaps = np.zeros([2, depth.shape[1], depth.shape[2], 3])

    processed_depthmaps[0, :, :, :] = cv2.cvtColor(colorize_depth(depth[0]), cv2.COLOR_RGB2BGR)
    processed_depthmaps[1, :, :, :] = cv2.cvtColor(colorize_depth(depth[1]), cv2.COLOR_RGB2BGR)
    
    colorspace_conversion_time = time.time_ns()
    
    left_img = bridge.cv2_to_imgmsg(processed_depthmaps[0])
    right_img = bridge.cv2_to_imgmsg(processed_depthmaps[1])
    
    imgmsg_conversion_time = time.time_ns()
    
    # cv2.imwrite(f"left_1res{idx}.png", processed_depthmaps[0])
    # cv2.imwrite(f"Right_1res{idx}.png", processed_depthmaps[1])
    # cv2.waitKey(0)
    
    imgmsg_time += (imgmsg_conversion_time - colorspace_conversion_time) / 1e6
    total_time += (imgmsg_conversion_time - start) / 1e6
    inf_time += (model_inf_time - start) / 1e6
    colorspace_time += (colorspace_conversion_time - model_inf_time) / 1e6
    time.sleep(1/15) # 15 FPS

print(f"Total time: {total_time/1000.0} s")
print(f"Model inference time: {inf_time/1000.0} s")
print(f"Colorspace conversion time: {colorspace_time/1000.0} s")
print(f"Image message conversion time: {imgmsg_time/1000.0} s")
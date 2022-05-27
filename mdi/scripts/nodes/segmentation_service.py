import sys
from urllib import response
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision
import torchvision.transforms as transforms
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from torchvision.models.segmentation import fcn
import glob
from PIL import Image
import time
from torch.utils.data import DataLoader
import rospy
from mdi_msgs.srv import Model, ModelResponse, ModelRequest
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# from rospy.numpy_msg import numpy_msg

from torch.utils.tensorboard import SummaryWriter

device = torch.device("cuda:0")
# device = "cpu"
print("Device chosen:", torch.cuda.get_device_name(device))

# Parameters
num_classes = 7  # Automatically calculated in the next cell
# Optimiser parameters
learning_rate = 0.0001
beta1 = 0.9
beta2 = 0.999

torch.cuda.empty_cache()


# Loading saved model
model = fcn.fcn_resnet50(pretrained=False, progress=True, num_classes=num_classes,
                         aux_loss=False, pretrained_backbone=True).to(device)
optimiser = torch.optim.Adam(model.parameters(), lr=learning_rate, betas=[
                             beta1, beta2], eps=1e-08)
model_path = sys.argv[1]
print(model_path)
loaded_checkpoint = torch.load(model_path)


model.load_state_dict(loaded_checkpoint["model_state"])
model.eval()
model.to(device=device)
optimiser.load_state_dict(loaded_checkpoint["optimiser_state"])
epoch = loaded_checkpoint["epoch"]
criterion = nn.CrossEntropyLoss()


# Service Inference Callback
# sensor_msgs/Image Message
bridge = CvBridge()


@torch.inference_mode()
def semantic_segmentation_inference_CB(image_msg):  # Consider nograd
    resp = ModelResponse()
    resp.header.seq = image_msg.image.header.seq
    resp.header.frame_id = "camera"
    resp.header.stamp = rospy.Time.now()
    resp.successful = False

    if len(image_msg.image.data) > 0:
        print("SERVICE CALLBACK INITIATED")
        # Image digestion and resizing
        # print(image_msg.image.data)
        cv_image = bridge.imgmsg_to_cv2(
            image_msg.image, desired_encoding='passthrough')
        np.asarray(cv_image)
        print(cv_image.shape)

        image = torch.tensor(cv_image, dtype=torch.float32)
        image = image.to(device=device)
        image.resize_(1, 3, image_msg.image.height,
                      image_msg.image.width)  # [3, H, W]
        print(image.shape)
        print(image.device)
        # print(model.device)

        # Inference
        output = model(image)['out']
        # output.to("cpu")
        print('Image inference completed')

        pred = torch.argmax(output, dim=1)
        fig, ax = plt.subplots(ncols=2, figsize=(24, 16))
        ax[0].imshow(pred.to("cpu").detach().squeeze())
        ax[1].imshow(cv_image)
        plt.savefig("/home/jens/Desktop/test.png")
        # Flatten into message and return

        resp.height = image_msg.image.height
        resp.width = image_msg.image.width
        print("before flattenning")
        resp.data = list(torch.flatten(output))
        print("after flattenning")
        resp.successful = True

        del image
        print('Returning the classification mask')

    return resp


# ROS service
rospy.init_node('computer_vision_node')
inference_service = rospy.Service('/mdi/semantic_segmentation', Model,
                                  semantic_segmentation_inference_CB)

print('Ready to recieve requests')
rospy.spin()

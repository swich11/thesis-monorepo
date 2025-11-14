# Runs a quick sanity test for each flow net


import torch
import torch.nn as nn


from utils import calculate_depth_loss, calculate_flow_loss, calculate_velocity_loss


from OpticalFlowNet import OpticalFlowNet
from OpticalFlowVelocityNet import OpticalFlowVelocityNet
from DepthMapNet import DepthMapNet
from DepthMapVelocityNet import DepthMapVelocityNet
from VelocityNet import VelocityNet


net1 = DepthMapNet()
net2 = DepthMapVelocityNet()
net3 = OpticalFlowNet()
net4 = OpticalFlowVelocityNet()
net5 = VelocityNet()


x = torch.randn(1, 2, 346, 260)
v = torch.randn(1, 6)

d_real = torch.randn(1, 1, 346, 260)
o_real = torch.randn(1, 2, 346, 260)
v_real = torch.randn(1, 6)


print("Depth Net")
calculate_flow_loss(net1(x), d_real).backward()
print("Depth Vel Net")
calculate_flow_loss(net2(x, v), d_real).backward()
print("Flow Net")
calculate_depth_loss(net3(x), o_real).backward()
print("Flow Velocity Net")
calculate_depth_loss(net4(x, v), o_real).backward()
print("Velocity Net")
v_out: torch.Tensor
v_out, _, _ = net5(x, v)
calculate_velocity_loss(v_out, v_real).backward()



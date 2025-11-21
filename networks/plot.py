import numpy as np
import matplotlib.pyplot as plt
import h5py
from VelocityNet import VelocityNet
from OpticalFlowNet import OpticalFlowNet
from DepthMapVelocityNet import DepthMapVelocityNet
from utils import calculate_AAE_simple
import torch

device = torch.device("cuda")

# f = h5py.File('../datasets/first.hdf5', 'r')
# # # # accelerations = f["IMULinearAcceleration"]
# physics_time = f["IMUTime"][1:501]
# sim_velocities = torch.Tensor(f["SimVelocities"][:501])
# # flows = torch.Tensor(f["MotionFlow"][:501, :, :, :2])
# depths = torch.Tensor(f["Depths"][:501])
# events_ON = torch.Tensor(f["OnEvents"][:501])
# events_OFF = torch.Tensor(f["OffEvents"][:501])
# events = torch.stack([torch.Tensor(events_ON), torch.tensor(events_OFF)], dim=1)

# depths[torch.isinf(depths)] = 0.0
# flows[flows == -1.0] = 0.0


# losses = np.load("depth-5-epoch-losses.npy")



net = DepthMapVelocityNet().to(device)
net.load_state_dict(torch.load("models/depth_net.pth")["model"])
for name, p in net.named_parameters():
    print(name, p)


# mems = net.init_mems(events[0].unsqueeze(0), device)[0]
# net.eval()
# torch.no_grad()
# net_AAE = []
# net_AAF = []
# for i in range(1, 501):
#     event = events[i].unsqueeze(0).to(device)
#     depth = depths[i].unsqueeze(0).unsqueeze(0).to(device)
#     vel = sim_velocities[i-1].unsqueeze(0).to(device)
#     res, mems = net(event, vel, mems)
#     mems = [m.detach() for m in mems]
#     net_AAF.append(float(torch.mean(torch.abs(res)).detach()))
#     net_AAE.append(float(calculate_AAE_simple(res, depth).cpu().detach()))

# net_AAE = np.vstack(net_AAE)
# net_AAF = np.vstack(net_AAF)


# fig, axes = plt.subplots(3, 2)
# fig.suptitle("Velocity Net Predictions vs Ground Truth on Dataset A")

# l1 = axes[0, 0].plot(physics_time[:500, 0], net_velocities[:500, 0], color='blue', label="Predicted")
# l2 = axes[0, 0].plot(physics_time[:500, 0], sim_velocities[:500, 0], color='green', label="Ground Truth")
# axes[0, 0].set_title("X Linear")
# axes[0, 0].set_ylabel("Velocity (ms\u207b\u00b9)")
# axes[0, 0].set_xticks([])
# axes[0, 1].plot(physics_time[:500, 0], net_velocities[:500, 1], color='blue', label="Predicted")
# axes[0, 1].plot(physics_time[:500, 0], sim_velocities[:500, 1], color='green', label="Ground Truth")
# axes[0, 1].set_title("Y Linear")
# axes[0, 1].set_xticks([])

# axes[1, 0].plot(physics_time[:500, 0], net_velocities[:500, 2], color='blue', label="Predicted")
# axes[1, 0].plot(physics_time[:500, 0], sim_velocities[:500, 2], color='green', label="Ground Truth")
# axes[1, 0].set_title("Z Linear")
# axes[1, 0].set_xticks([])
# axes[1, 0].set_ylabel("Velocity (ms\u207b\u00b9)")
# axes[1, 1].plot(physics_time[:500, 0], net_velocities[:500, 3], color='blue', label="Predicted")
# axes[1, 1].plot(physics_time[:500, 0], sim_velocities[:500, 3], color='green', label="Ground Truth")
# axes[1, 1].set_title("X Angular")
# axes[1, 1].set_xticks([])

# axes[2, 0].plot(physics_time[:500, 0], net_velocities[:500, 4], color='blue', label="Predicted")
# axes[2, 0].plot(physics_time[:500, 0], sim_velocities[:500, 4], color='green', label="Ground Truth")
# axes[2, 0].set_title("Y Angular")
# axes[2, 0].set_ylabel("Velocity (ms\u207b\u00b9)")
# axes[2, 0].set_xlabel("Time (s)")
# axes[2, 1].plot(physics_time[:500, 0], net_velocities[:500, 5], color='blue', label="Predicted")
# axes[2, 1].plot(physics_time[:500, 0], sim_velocities[:500, 5], color='green', label="Ground Truth")
# axes[2, 1].set_title("Z Angular")
# axes[2, 1].set_xlabel("Time (s)")
# fig.legend(axes[0, 0].get_legend_handles_labels()[1])
# right side y label
# ax.text(acceleration (ms\u207b\u00b2)
#     1.02, 0.5, "velocity (ms\u207b\u00b9)",
#     transform=ax.transAxes,
#     rotation=90,
#     va='center'
# )
# axes.set_title("Velocity Net Predictions vs Ground Truth on Dataset A")
# plt.show()



# fig, ax = plt.subplots()
# ax.plot(physics_time[:, 0], net_AAE, color="blue", label="AADE")
# ax.plot(physics_time[:, 0], net_AAF, color="green", label="AAD")
# ax.set_xlabel("Time (s)")
# ax.set_ylabel("Error")
# ax.legend()
# ax.set_title("AADE and AAD for Depth Map Net on Dataset A")
# plt.show()


# f.close()


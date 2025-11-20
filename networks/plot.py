import numpy as np
import matplotlib.pyplot as plt
import h5py
from VelocityNet import VelocityNet
import torch

device = torch.device("cuda")

f = h5py.File('../datasets/first.hdf5', 'r')
# accelerations = f["IMULinearAcceleration"]
physics_time = f["IMUTime"][:500]
sim_velocities = f["SimVelocities"][:500]
flows = torch.Tensor(f["MotionFlow"][:500])
depths = torch.Tensor(f["Depths"][:500])

depths[torch.isinf(depths)] = 0.0
flows[flows == -1.0] = 0.0


losses = np.load("velocity-5-epoch-losses.npy")
net = VelocityNet().to(device)
net.load_state_dict(torch.load("models/velocity_net.pth")["model"])


net_velocities = []
for i in range(500):
    net_velocities.append(net(depths[i].unsqueeze(dim=0).unsqueeze(dim=0).to(device), 
                               flows[i].permute(2, 0, 1)[:2].unsqueeze(dim=0).to(device))
                               .detach().cpu().numpy())

net_velocities = np.vstack(net_velocities)


fig, axes = plt.subplots(3, 2)
fig.suptitle("Velocity Net Predictions vs Ground Truth on Dataset A")

l1 = axes[0, 0].plot(physics_time[:500, 0], net_velocities[:500, 0], color='blue', label="Predicted")
l2 = axes[0, 0].plot(physics_time[:500, 0], sim_velocities[:500, 0], color='green', label="Ground Truth")
axes[0, 0].set_title("X Linear")
axes[0, 0].set_ylabel("Velocity (ms\u207b\u00b9)")
axes[0, 0].set_xticks([])
axes[0, 1].plot(physics_time[:500, 0], net_velocities[:500, 1], color='blue', label="Predicted")
axes[0, 1].plot(physics_time[:500, 0], sim_velocities[:500, 1], color='green', label="Ground Truth")
axes[0, 1].set_title("Y Linear")
axes[0, 1].set_xticks([])

axes[1, 0].plot(physics_time[:500, 0], net_velocities[:500, 2], color='blue', label="Predicted")
axes[1, 0].plot(physics_time[:500, 0], sim_velocities[:500, 2], color='green', label="Ground Truth")
axes[1, 0].set_title("Z Linear")
axes[1, 0].set_xticks([])
axes[1, 0].set_ylabel("Velocity (ms\u207b\u00b9)")
axes[1, 1].plot(physics_time[:500, 0], net_velocities[:500, 3], color='blue', label="Predicted")
axes[1, 1].plot(physics_time[:500, 0], sim_velocities[:500, 3], color='green', label="Ground Truth")
axes[1, 1].set_title("X Angular")
axes[1, 1].set_xticks([])

axes[2, 0].plot(physics_time[:500, 0], net_velocities[:500, 4], color='blue', label="Predicted")
axes[2, 0].plot(physics_time[:500, 0], sim_velocities[:500, 4], color='green', label="Ground Truth")
axes[2, 0].set_title("Y Angular")
axes[2, 0].set_ylabel("Velocity (ms\u207b\u00b9)")
axes[2, 0].set_xlabel("Time (s)")
axes[2, 1].plot(physics_time[:500, 0], net_velocities[:500, 5], color='blue', label="Predicted")
axes[2, 1].plot(physics_time[:500, 0], sim_velocities[:500, 5], color='green', label="Ground Truth")
axes[2, 1].set_title("Z Angular")
axes[2, 1].set_xlabel("Time (s)")
fig.legend(axes[0, 0].get_legend_handles_labels()[1])
# right side y label
# ax.text(acceleration (ms\u207b\u00b2)
#     1.02, 0.5, "velocity (ms\u207b\u00b9)",
#     transform=ax.transAxes,
#     rotation=90,
#     va='center'
# )
# axes.set_title("Velocity Net Predictions vs Ground Truth on Dataset A")
plt.show()



# fig, ax = plt.subplots()
# ax.plot(losses, color='blue')
# ax.set_xlabel("Batch Number")
# ax.set_ylabel("MSE Loss")
# ax.set_title("Training Losses for Velocity Net")
# plt.plot(losses)
# plt.show()


f.close()


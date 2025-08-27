import snntorch as snn


from snntorch import spikeplot as splt
from snntorch import spikegen


import torch
import torch.nn as nn


import numpy as np
import matplotlib.pyplot as plt



num_steps = 100

# lapicque simple neuron
time_step = 1e-3
R = 3
C = 1e-3

lif1 = snn.Lapicque(R=R, C=C, time_step=time_step)
lif2 = snn.Leaky(0.8, reset_mechanism="subtract")

spk_in = spikegen.rate_conv(torch.ones((num_steps, 1)) * 0.4)*0.5
spk_out = torch.zeros(1)
mem = torch.zeros(1)

mem_rec = [mem]
spk_rec = [spk_out]
for step in range(num_steps):
    spk_out, mem = lif1(spk_in[step], mem)
    mem_rec.append(mem)
    spk_rec.append(spk_out)

mem_rec = torch.stack(mem_rec)
spk_rec = torch.stack(spk_rec)


fig = plt.figure(facecolor="w", figsize=(8, 9))
ax = fig.add_subplot(3, 1, 1)
splt.raster(spk_in.reshape(num_steps, -1), ax, s=100, c="black", marker="|")
plt.title("rawr title")
plt.yticks([])
plt.ylabel("Input Spikes")
ax = fig.add_subplot(3, 1, 2)
ax.plot(mem_rec)
plt.ylabel("Membrane Potential (U)")
ax = fig.add_subplot(3, 1, 3)
splt.raster(spk_rec, ax, s=100, c="black", marker="|")
plt.ylabel("Output Spikes")
plt.xlabel("Time Step")


plt.show()



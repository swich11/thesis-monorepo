import snntorch as snn
import torch


from torchvision import datasets, transforms
from snntorch import utils, spikegen
from torch.utils.data import DataLoader


import matplotlib.pyplot as plt
import snntorch.spikeplot as splt


# training parameters
batch_size = 128
data_path='/tmp/data/mnist'
num_classes = 10 # mnist 0-9

dtype = torch.float


# grab the MNIST dataset
transform = transforms.Compose([
                transforms.Resize((28, 28)),
                transforms.Grayscale(),
                transforms.ToTensor(),
                transforms.Normalize((0,), (1,))
            ])

mnist_train = datasets.MNIST(data_path, train=True, download=True, transform=transform)


subset = 10
mnist_train = utils.data_subset(mnist_train, subset)  # subsample the dataset


# datasets are loaded into memory, dataloaders can pull them from memory in batches
train_loader = DataLoader(mnist_train, batch_size=batch_size, shuffle=True)

# sequence the data sample into time varying spikes of random probability (this mimics a time-varied video)


# rate coding:      input -> spike frequency
# latency coding:   input -> spike timing
# delta modulation: input -> temporal change in input generates spikes


# rate coding:
num_steps = 100
raw_vector = torch.ones(num_steps)*0.5
rate_coded_vector = torch.bernoulli(raw_vector) # perform a bernoulli trial on the input


data = iter(train_loader) # grab the iterable over the data loader
data_it, targets_it = next(data) # grab the next batch in the iterator from the dataloader

spike_data = spikegen.rate(data_it, num_steps=num_steps) # do the exact bernoulli trial above

spike_data_sample = spike_data[:, 0, 0]
# fig, ax = plt.subplots()
# anim1 = splt.animator(spike_data_sample, fig, ax)


spike_data = spikegen.rate(data_it, num_steps=num_steps, gain=0.25)
spike_data_sample2 = spike_data[:, 0, 0]
# fig, ax = plt.subplots()
# anim2 = splt.animator(spike_data_sample2, fig, ax)



# fig = plt.figure(facecolor="w")
# ax = plt.subplot(2, 2, 1)
# anim1 = splt.animator(spike_data_sample.mean(axis=0).repeat(num_steps, 1, 1), fig, ax, cmap='binary')
# plt.axis('off')
# plt.title('Gain = 1')

# ax = plt.subplot(2, 2, 2)
# anim2 = splt.animator(spike_data_sample2.mean(axis=0).repeat(num_steps, 1, 1), fig, ax, cmap='binary')
# plt.axis('off')
# plt.title('Gain = 0.25')

# ax = plt.subplot(2, 2, 3)
# anim3 = splt.animator(spike_data_sample, fig, ax, cmap='binary')

# ax = plt.subplot(2, 2, 4)
# anim4 = splt.animator(spike_data_sample2, fig, ax, cmap='binary')



# raster plots
spike_data_sample2 = spike_data_sample2.reshape((num_steps, -1)) # condense array output to a single input layer


# fig = plt.figure(facecolor="w", figsize=(10, 5))
# ax = fig.add_subplot(111)
# splt.raster(spike_data_sample2, ax, s=1.5, c="black")

# plt.title("Input Layer")
# plt.xlabel("Time step")
# plt.ylabel("Neuron Number")



# single neuron spiking
# idx=470
# fig = plt.figure(facecolor="w", figsize=(8, 1))
# ax = fig.add_subplot(111)
# splt.raster(spike_data_sample2[:, idx].unsqueeze(1), ax, s=100, c="black", marker="|")

# plt.title("Input Neuron")
# plt.xlabel("Time step")
# plt.yticks([])
# plt.show()


# latency coding -> RC circuit time delay
def convert_to_time(data, tau=5, threshold=0.01):
    return tau*torch.log(data/(data-threshold))    

# or just do spikegen.latency
spike_data = spikegen.latency(data_it, num_steps=num_steps, tau=5, threshold=0.01, normalize=True, clip=True)

# create raster plot to visualise the latency encoding
fig = plt.figure(figsize=(10,5))
ax = fig.add_subplot(111)
splt.raster(spike_data[:, 0].view(num_steps, -1), ax, s=10, c="black")
plt.title("Input Layer")
plt.xlabel("Time step")
plt.ylabel("Neuron Number")


# animate
spike_data_sample = spike_data[:, 0, 0]
fig, ax = plt.subplots()
anim = splt.animator(spike_data_sample, fig, ax, cmap="binary")



# plt.show() -> latency encoding seems kind of pointless



# delta modulation is the main way to go here
















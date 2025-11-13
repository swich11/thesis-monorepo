import h5py
import numpy as np


# Takes a hdf5 file, doesn't really matter what the heck the end tag is

f = h5py.File('sim_dataset.hdf5', 'r')
dset = f






print(dset)
print(dset.size)


f.close()
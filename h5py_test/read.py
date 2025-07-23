import h5py
import numpy as np

f = h5py.File('test.hdf5', 'r')
dset = f['random_array'][:]


print(dset)
print(dset.size)
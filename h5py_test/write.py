import h5py
import numpy as np

np_array = np.random.rand(20, 10)

f = h5py.File('test.hdf5', 'w')

dset = f.create_dataset('random_array',
                        shape=(0, 20, 10),
                        dtype=np.float64,
                        maxshape=(None, 20, 10),
                        chunks=(1, 20, 10),
)


for i in range(10):
    data = np.random.rand(20, 10)
    dset.resize(tuple([dset.shape[0] + 1]) + dset.shape[1:len(dset.shape)])
    dset[-1] = data

f.close()
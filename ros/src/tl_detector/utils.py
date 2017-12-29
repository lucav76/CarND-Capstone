import os
import random
import numpy as np
from os.path import isfile, join
import tensorflow as tf

def train_test_split(samples, test_size, shuffle_samples=True):
    if shuffle_samples:
        samples = shuffle(samples)

    assert(test_size>0 and test_size<1)

    num_test = int(len(samples) * test_size)
    num_train = int(len(samples) - num_test)

    train = samples[0:num_train]
    test = samples[num_train:]

    assert (len(train) == num_train)
    assert (len(test) == num_test)
    assert (len(train) + len(test) == len(samples))

    return (train, test)

def shuffle(X):
    use_np = hasattr(X, 'shape')

    if use_np:
        indices = np.arange(X.shape[0])
        np.random.shuffle(indices)
        return X[indices]

    indices = [x for x in range(len(X))]
    random.shuffle(indices)
    return [X[idx] for idx in indices]

def shuffle2(X, Y):
    use_np = hasattr(X, 'shape')

    if use_np:
        indices = np.arange(X.shape[0])
        np.random.shuffle(indices)
        return (X[indices], Y[indices])

    indices = [x for x in range(len(X))]
    random.shuffle(indices)
    return ([X[idx] for idx in indices], [Y[idx] for idx in indices])

def shuffle3(X, Y, Z):
    use_np = hasattr(X, 'shape')

    if use_np:
        indices = np.arange(X.shape[0])
        np.random.shuffle(indices)
        return (X[indices], Y[indices], Z[indices])

    indices = [x for x in range(len(X))]
    random.shuffle(indices)
    return ([X[idx] for idx in indices], [Y[idx] for idx in indices], [Z[idx] for idx in indices])

def resize_to_1_if_required(img):
    shape = np.shape(img)

    if (len(shape)==3):
        return img

    return np.reshape(img, (shape[0], shape[1], 1))

def files_only(dir):
    #list = [join(dir, f) for f in os.listdir(dir) if isfile(join(dir, f))]

    if not os.path.isdir(dir):
        return []

    list = []

    for f in os.listdir(dir):
        path = join(dir, f)

        if (isfile(path)):
            list.append(path)

    return list

def files_from(init_dir, include_sub_directories = False):
    if not init_dir:
        return []

    if include_sub_directories:
        dirs = [x[0] for x in os.walk(init_dir)]
    else:
        dirs = [init_dir]

    files = []

    for dir in dirs:
        files.extend(files_only(dir))

    return files

def get_gpu_name():
    return tf.test.gpu_device_name()
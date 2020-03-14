import struct
import numpy
import sys
import matplotlib.pyplot as plt

filename = sys.argv[1]
bindata = b''
data = numpy.empty([1,8])

print(filename)

with open(filename,'rb') as dat:
    bindata = dat.read()

for dat in struct.iter_unpack("IffffffI",bindata):
    data = numpy.vstack((data,dat))


def plotaccxyz():
    plt.plot(data[1:,0],data[1:,1])
    plt.plot(data[1:,0],data[1:,2])
    plt.plot(data[1:,0],data[1:,3])
    plt.legend(("Acceleration X axis","Acceleration Y axis","Acceleration Z axis"))

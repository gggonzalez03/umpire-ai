import os
import sys
import struct
import matplotlib.pyplot as plt
import numpy as np

file = open(sys.argv[1], 'rb')
file_size = os.path.getsize(sys.argv[1])
data_points_total_count = file_size / 4
data_label_points = 1
data_points_per_vibration = 100 + data_label_points

numbers = list(struct.unpack('f' * data_points_total_count, file.read(4 * data_points_total_count)))
file.close()

with open(os.path.splitext(sys.argv[1])[0] + '.txt', 'w') as f:
	for index, item in enumerate(numbers):
		f.write("%s: %s, " % (index + 1, item))
		if (index != 0 and (index + 1) % data_points_per_vibration == 0):
			f.write("\n")

plt.plot(numbers)
plt.show()
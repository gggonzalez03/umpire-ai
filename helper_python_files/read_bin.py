import os
import struct
import matplotlib.pyplot as plt
import numpy as np

file = open('data_dump.bin', 'rb')
file_size = os.path.getsize('data_dump.bin')
data_points_total_count = file_size / 4
data_points_per_vibration = 100

numbers = list(struct.unpack('f' * data_points_total_count, file.read(4 * data_points_total_count)))
file.close()

with open('readable.txt', 'w') as f:
	for index, item in enumerate(numbers):
		f.write("%s, " % item)
		if (index != 0 and index % data_points_per_vibration == 0):
			f.write("\n")


plt.plot(numbers)
plt.show()
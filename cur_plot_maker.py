from csv import reader

voltages_requested = []
currents_requested = []
currents_error = []

for amperage in range(0, 82, 4):
	file_name = "Inc_Power_ch2/%1.2fA Voltage.csv" % (amperage / 100.0)
	with open(file_name, 'r') as file:
		csv_reader = reader(file)
		for data_values in csv_reader:
			voltages_requested.append(float(data_values[0]))
			currents_requested.append(float(data_values[3]))
			current_error = (float(data_values[4]) - float(data_values[5])) * 1000.0
			currents_error.append(current_error)

import matplotlib.pyplot as plt

fig = plt.figure()
cur_plot = fig.add_subplot(projection='3d')

cur_plot.scatter(voltages_requested, currents_requested, current_error, color="Red")
cur_plot.set_title("Current Error")
cur_plot.set_xlabel('Voltages Requested (V)')
cur_plot.set_ylabel('Current Requested (V)')
cur_plot.set_zlabel('Current Error (mA)')

plt.show()
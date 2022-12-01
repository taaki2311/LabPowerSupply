from csv import reader

voltages_requested = []
voltages_error = []
currents_requested = []

for amperage in range(0, 82, 4):
	file_name = "Inc_Power_ch2/%1.2fA Voltage.csv" % (amperage / 100.0)
	with open(file_name, 'r') as file:
		csv_reader = reader(file)
		for data_values in csv_reader:
			voltages_requested.append(float(data_values[0]))
			voltage_error = float(data_values[1]) - float(data_values[2])
			voltages_error.append(voltage_error)
			currents_requested.append(float(data_values[3]))

import matplotlib.pyplot as plt

fig = plt.figure()
volt_plot = fig.add_subplot(projection='3d')
volt_plot.scatter(voltages_requested, currents_requested, voltages_error, color="Blue")
volt_plot.set_title("Voltage Error")
volt_plot.set_xlabel('Voltages Requested (V)')
volt_plot.set_ylabel('Current Requested (V)')
volt_plot.set_zlabel('Voltage Error(V)')

plt.show()
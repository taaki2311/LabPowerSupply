from csv import reader
from statistics import mean, median, stdev
from matplotlib.pyplot import figure, show

creamentation = ("In", "De")
channels = ('1', '2')

voltages_requested = []
currents_requested = []
currents_error = []

for creament in creamentation:
	for channel in channels:
		for amperage in range(0, 81, 8):
			file_name = "%sc Power ch%c/%1.2fA Voltage.csv" % (creament, channel, (amperage / 100.0))
			with open(file_name, 'r') as file:
				csv_reader = reader(file)
				for data_values in csv_reader:
					voltages_requested.append(float(data_values[0]))
					currents_requested.append(float(data_values[3]) * 1000.0)
					current_error = (float(data_values[4]) - float(data_values[5])) * 1000.0
					currents_error.append(current_error)

print("Mean: %0.2f mA" % mean(currents_error))
print("Median: %.2f mA" % median(currents_error))
print("Standard Deviation: %.2f mA" % stdev(currents_error))

fig = figure()
cur_plot = fig.add_subplot(projection='3d')
cur_plot.scatter(voltages_requested, currents_requested, currents_error, color="Red")
cur_plot.set_title("Current Error")
cur_plot.set_xlabel('Voltages Requested (V)')
cur_plot.set_ylabel('Current Requested (mA)')
cur_plot.set_zlabel('Current Error (mA)')
show()

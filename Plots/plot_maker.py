from csv import reader
from statistics import mean, median, stdev
from matplotlib.pyplot import figure, show
from matplotlib import cm

creamentation = ("In", "De")
channels = ('1', '2')

voltages_requested = []
currents_requested = []
voltages_error = []
currents_error = []

for creament in creamentation:
	for channel in channels:
		for amperage in range(0, 81, 8):
			file_name = "%sc Power ch%c/%1.2fA Voltage.csv" % (creament, channel, (amperage / 100.0))
			with open(file_name, 'r') as file:
				csv_reader = reader(file)
				for data_values in csv_reader:
					voltages_requested.append(float(data_values[0]))
					voltage_error = (float(data_values[1]) - float(data_values[2]))
					voltages_error.append(voltage_error)
					currents_requested.append(float(data_values[3]) * 1000.0)
					current_error = (float(data_values[4]) - float(data_values[5])) * 1000.0
					currents_error.append(current_error)

print("Voltage Error Mean: %.2f mV" % mean(voltages_error))
print("Voltage Error Median: %.2f mV" % median(voltages_error))
print("Voltage Error Standard Deviation: %.2f mV" % stdev(voltages_error))
print("Current Error Mean: %0.2f mA" % mean(currents_error))
print("Current Error Median: %.2f mA" % median(currents_error))
print("Current Error Standard Deviation: %.2f mA" % stdev(currents_error))

fig = figure()
volt_plot = fig.add_subplot(projection='3d')
plot = volt_plot.scatter(voltages_requested, currents_requested, voltages_error, c=currents_error, cmap=cm.coolwarm)
fig.colorbar(plot, location="left", shrink=0.5, label="Current Error (mA)")
volt_plot.set_title("Nominal Error")
volt_plot.set_xlabel('Voltages Requested (V)')
volt_plot.set_ylabel('Current Requested (mA)')
volt_plot.set_zlabel('Voltage Error (V)')
show()
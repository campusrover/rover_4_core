from pickle import load
from os.path import dirname, realpath
from os import sep
import sys
import matplotlib.pyplot as plt 
from statistics import mean

def get_nearby_file(filename):
    return dirname(realpath(sys.argv[0])) + sep + filename

data = []
with open(get_nearby_file('move_data.pickle'), 'rb') as infile:
    data = load(infile)

"""
'data' is a series of nested lists
data[0] is linear lists
data[1] is angular

linear[0] is velocity reported by odom
linear[1] is velocity insturcted by twist (cmd_vel)

angular follows same pattern

"""


print(data[0][0])
print(mean(data[0][0]))
print(mean(data[0][0][20:]))
print(mean(data[0][0][40:]))
plt.plot(data[0][0], 'b')
plt.plot(data[1][0], 'r')
plt.ylabel('red=rotation, blue=linear')
plt.show()

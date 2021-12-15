import matplotlib.pyplot as plt

order = open("speed_r.txt").readlines()
print(type(order))
speed = []
for line in order:
    # print(line)
    speed.append(float(line))
frame = range(0,len(speed))
plt.plot(frame,speed)
plt.show()

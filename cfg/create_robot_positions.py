# note : this does not create the link between the map and the world. It only spawns the robots.
# Please make sure to go back and manually add the path to the bitmap file
file_name='my_empty_10x10_30.world'
f = open("../resources/sim/"+file_name,"w+")
x = 0.0
y = 0.0
for i in range(2):
	for j in range(10):
		f.write('sphero( pose [ %f %f 0.000 0.000 ] name "sphero_%d" color "blue")\n'%(x,y,i*10+j))
		x -= 0.2
	y += 0.2
	x = 0.0
f.close()

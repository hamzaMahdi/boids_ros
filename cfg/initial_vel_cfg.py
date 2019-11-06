f = open("sphero_init_vel.cfg","w+")
for i in range(50):
	f.write('Sphero_%d	0	0\n'%(i))
f.close()
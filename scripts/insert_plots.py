# note : this does not create the link between the map and the world. It only spawns the robots.
# Please make sure to go back and manually add the path to the bitmap file
file_name = 'plots.txt'
f = open("../new_results/" + file_name, "w+")
counter = 1
for i in range(1, 10):
    for j in range(1, 6):
        f.write('\subfloat{\includegraphics[width=0.5\linewidth]{figures/test_%d_%d.png}}\n' % (i, j))
        if counter % 2 == 0:
            f.write(r'\\ ')

        counter+=1
f.close()

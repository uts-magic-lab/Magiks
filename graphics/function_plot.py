import matplotlib.pyplot as plt
import numpy as np
 

'''
Specify x axis data:
'''
t = np.arange(-1,2, .01)
'''
Specify y axis data:
'''
y = np.sin(2*np.pi*t)

plt.plot(t,y)
'''
you can insert a normal text or a latex code in the x and y axis label string: 
'''
plt.ylabel('Sine wave:  $y = \sin(t)$')
plt.xlabel('$t \in [-1,2]$')

width = 0.0       
#ind = np.arange(5)
ind = np.array([-2, -1, 0, 1, 3])

print ind

plt.xticks(ind+width, ('-2', '$t = -1$', 'Zero', '1.0', '$\pi$') )

plt.show()



"""

'''
The axis() command in the example above takes a list of [xmin, xmax, ymin, ymax] and specifies the viewport of the axes
'''
plt.axis([0, 6, 0, 20])



"""



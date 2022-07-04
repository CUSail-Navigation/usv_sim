#!/usr/bin/env python
import numpy as np

# mass in kg
orig_mass = 0.000071
new_mass = 1

inertia = np.array([[0.00000644178, -0.000000, -0.00000000001], [-0.000000, 0.00000644173, -0.000000000011], [-0.00000000001, -0.000000000011, 0.00000000355]])

inertia = (inertia / orig_mass) * new_mass

print("<ixx>{}</ixx>".format(inertia[0,0]))
print("<ixy>{}</ixy>".format(inertia[0,1]))
print("<ixz>{}</ixz>".format(inertia[0,2]))
print("<iyy>{}</iyy>".format(inertia[1,1]))
print("<iyz>{}</iyz>".format(inertia[1,2]))
print("<izz>{}</izz>".format(inertia[2,2]))
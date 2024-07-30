import numpy
from stl import mesh
import os

my_mesh = mesh.Mesh.from_file('wheel.stl')
my_mesh.vectors /= 1000
my_mesh.save('scaled_wheel.stl')
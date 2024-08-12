import os

from stl import mesh

mesh_path = "klp_urdf/meshes/collision/"
meshes = os.listdir(mesh_path)
print(meshes)
for stl_mesh in meshes:
    if os.path.isdir(mesh_path + stl_mesh):
        continue
    print(mesh_path + stl_mesh)
    my_mesh = mesh.Mesh.from_file(mesh_path + stl_mesh)
    my_mesh.vectors /= 1000
    my_mesh.save(mesh_path + stl_mesh)

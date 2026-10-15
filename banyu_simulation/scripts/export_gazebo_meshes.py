#!/usr/bin/env python3
"""
Export FreeCAD competition node geometry to Gazebo STL meshes.
"""
import xmlrpc.client
import os

MESH_DIR = "/home/erlangga/Desktop/underwater-magnetic-activated-light/models/robotx_light_node/meshes"
os.makedirs(MESH_DIR, exist_ok=True)

proxy = xmlrpc.client.ServerProxy('http://localhost:9875')

code = f'''
import FreeCAD as App
import Mesh

doc = App.ActiveDocument
if not doc or doc.Name != "RobotX_Light_Node":
    doc = App.openDocument("/home/erlangga/Desktop/underwater-magnetic-activated-light/RobotX_Light_Node.FCStd")

print("Exporting meshes from doc:", doc.Name)

base_objs = [doc.Enclosure_Base, doc.Cover_Screws]
Mesh.export(base_objs, "{MESH_DIR}/base.stl")

lid_objs = [doc.Enclosure_Lid]
Mesh.export(lid_objs, "{MESH_DIR}/lid.stl")

penetrator_objs = [doc.WetLink_Penetrator_Left, doc.WetLink_Penetrator_Right]
Mesh.export(penetrator_objs, "{MESH_DIR}/penetrators.stl")

internal_objs = [
    doc.Internal_Carrier, doc.LED_Ring_PCB, doc.WS2812_LED_Packages, 
    doc.Magnetometer_PCB, doc.Magnetometer_Gold_Pads, doc.TLV493D_Sensor_IC, 
    doc.Magnetometer_Silkscreen, doc.STEMMA_QT_Connectors
]
Mesh.export(internal_objs, "{MESH_DIR}/internals.stl")

led_objs = [doc.WS2812_LED_Lenses]
Mesh.export(led_objs, "{MESH_DIR}/led_lenses.stl")

print("Mesh export complete!")
'''

res = proxy.execute_code(code)
print(res['message'])

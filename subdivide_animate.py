import numpy as np
import os
import random
import time
import vtk
import sys
import argparse
import cv2 as cv
from vtk.util.numpy_support import vtk_to_numpy

import mesh

def generate_polydata(surface_mesh):

    vertices = surface_mesh.vertices
    polygons = surface_mesh.faces

    points = vtk.vtkPoints()
    
    for v in vertices:
        p = v.get_val()
        points.InsertNextPoint(p[0], p[1], p[2])

    vtk_polygons = vtk.vtkCellArray()

    for p in polygons:
        
        vtk_polygon = vtk.vtkPolygon()
        vtk_polygon.GetPointIds().SetNumberOfIds(len(p.adj_vertices))
        
        for i in range(len(p.adj_vertices)):
            vtk_polygon.GetPointIds().SetId(i, p.adj_vertices[i].id)

        vtk_polygons.InsertNextCell(vtk_polygon)

    polygonPolyData = vtk.vtkPolyData()
    polygonPolyData.SetPoints(points)
    polygonPolyData.SetPolys(vtk_polygons)

    return polygonPolyData

def update_polydata_from_animate_mesh(polydata, mesh_animate, t):
    
    for i in range(len(mesh_animate.vertices)):
        p = mesh_animate.vertices[i].get_val(t)
        polydata.GetPoints().SetPoint(i, (p[0], p[1], p[2]))


class animate_callback():

    FRAMES_PER_SUBDIVISION = 90
    CATMULL_CLARK = 1
    DOO_SABIN = 2

    def __init__(self, mesh, subdivision_method, subdivisions, renderer_window=None, out=None):
        self.ticks = 0
        self.subdivide_mesh = mesh

        self.animate_frames = [0]
        
        for i in range(subdivisions):
            frame_count = max(int(90 / ((1.25)**i)), 10)
            self.animate_frames.append(frame_count)
            
        self.animate_frames.append(60)
        
        self.animate_intervals = [sum(self.animate_frames[0:i + 1]) for i in range(len(self.animate_frames))]

        self.max_frames = subdivisions * self.FRAMES_PER_SUBDIVISION
        self.subdivision_method = subdivision_method
        self.subdivision_count = 0
        
        self.renderer_window = renderer_window
        self.out = out
        
        self.finished_animation = False
    
    def write_to_video_stream(self):
        
        window_out = vtk.vtkWindowToImageFilter()
        window_out.SetInput(self.renderer_window)
        window_out.Update()
        
        window_image = window_out.GetOutput()
        
        width = window_image.GetDimensions()[0]
        height = window_image.GetDimensions()[1]
        
        np_image = vtk_to_numpy(window_image.GetPointData().GetScalars()).reshape(height, width, 3)
        np_image = cv.cvtColor(np_image, cv.COLOR_BGR2RGB)
        np_image = np.flip(np_image, 0)
        
        self.out.write(np_image)
    
    def update(self, obj, event):

        if self.ticks < self.animate_intervals[-1]:
            if self.out != None:
                self.write_to_video_stream()

            if self.ticks < self.animate_intervals[-2]:
                if self.ticks == self.animate_intervals[self.subdivision_count]:
                    
                    self.subdivision_count += 1
                    print("Computing subdivision #" + str(self.subdivision_count))
                
                    if self.subdivision_method == self.CATMULL_CLARK:
                        self.subdivide_mesh, self.subdivide_animate_mesh = self.subdivide_mesh.compute_catmull_clark_surface()
                    elif self.subdivision_method == self.DOO_SABIN:
                        self.subdivide_mesh, self.subdivide_animate_mesh = self.subdivide_mesh.compute_doo_sabin_surface()

                    self.polydata = generate_polydata(self.subdivide_animate_mesh)
                    self.mapper = vtk.vtkPolyDataMapper()
                    self.mapper.SetInputData(self.polydata)
                    self.actor.SetMapper(self.mapper)
                    
                interval_frame = self.ticks - self.animate_intervals[self.subdivision_count - 1]
                num_frames = self.animate_frames[self.subdivision_count]
                    
                update_polydata_from_animate_mesh(self.polydata, self.subdivide_animate_mesh, (interval_frame + 1) / num_frames)
                self.polydata.GetPoints().Modified()
            
            iren = obj
            iren.GetRenderWindow().Render()
            
            self.ticks += 1

def read_off_file(filename):

    reader = open(filename, 'r')
    
    lines = reader.readlines()
    reader.close()
    
    points = []
    polys = []
    
    info_split = lines[1].split()
    
    num_vertices = int(info_split[0])
    num_faces = int(info_split[1])
    
    for line in lines[2:2 + num_vertices]:
        line_split = line.split()
        point = [float(x) for x in line_split]
        points.append(np.array(point))
    
    for line in lines[2 + num_vertices:2 + num_vertices + num_faces]:
        line_split = line.split()
        num_face_vertices = int(line_split[0])
        face = [int(x) for x in line_split[1:num_face_vertices + 1]]
        polys.append(face)
        
    return points, polys

def read_obj_file(filename):

    reader = open(filename, 'r')
    
    lines = reader.readlines()
    
    reader.close()
    
    points = []
    polys = []
    
    for line in lines:
            
        line_split = line.split()
    
        if len(line_split) == 0:
            continue
    
        if line_split[0] == "v":
            #vertex
            point = [float(x) for x in line_split[1:]]
            
            points.append(np.array(point))
            
        elif line_split[0] == "f":
            #face
            f = [int(x.split("/")[0]) - 1 for x in line_split[1:]]
            
            polys.append(f)
            
    return points, polys
                
#parse arguments

parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
parser.add_argument("mesh_file", help="an .off or .obj mesh file to subdivide")
parser.add_argument("-a", type=int, help="""subdivision algorithm to visualize
\tSubdivision algorithms:
\t\t1: Catmull-Clark
\t\t2: Doo-Sabin""")
parser.add_argument("-d", type=int, help="number of subdivisions to animate")
parser.add_argument("-v", help="video file to output animation to")
args = parser.parse_args()

filename = args.mesh_file
subdivision_method = 1
subdivisions = 3
video_filename = ""

if args.a:
    subdivision_method = args.a
    
if args.d:
    subdivisions = args.d

if args.v:
    video_filename = args.v

#output argument information

print("Mesh file: " + filename)

if subdivision_method == 1:
    print("Subdivision algorithm: Catmull-Clark")
elif subdivision_method == 2:
    print("Subdivision algorithm: Doo-Sabin")

print("Number of subdivisions to be animated: " + str(subdivisions))

if video_filename == "":
    print("Not outputting to video file")
else:
    print("Outputting to video to file " + video_filename)

#create initial mesh

filename_suffix = filename.split(".")[-1]

points = []
polys = []

if filename_suffix == "off":
    points, polys = read_off_file(filename)
elif filename_suffix == "obj":
    points, polys = read_obj_file(filename)

subdivide_mesh = mesh.Mesh()
subdivide_animate_mesh = mesh.Mesh()

for poly in polys:

    poly_points = []
    
    for i in poly:
        poly_points.append(points[i])

    subdivide_mesh.add_face_by_points(poly_points)

#configure vtk window

width = 1080
height = 1080

renderer = vtk.vtkRenderer()
render_window = vtk.vtkRenderWindow()
render_window.SetSize(width, height)
render_window.SetWindowName("Surface Subdivision")
render_window.AddRenderer(renderer)
render_window_interactor = vtk.vtkRenderWindowInteractor()
render_window_interactor.SetRenderWindow(render_window)

actor = vtk.vtkActor()

polydata = generate_polydata(subdivide_mesh)
mapper = vtk.vtkPolyDataMapper()
mapper.SetInputData(polydata)
actor.SetMapper(mapper)

polydata = generate_polydata(subdivide_mesh)
mapper = vtk.vtkPolyDataMapper()
mapper.SetInputData(polydata)
actor.SetMapper(mapper)

actor.SetMapper(mapper)
actor.GetProperty().SetColor(.75, .75, 1)
actor.GetProperty().SetEdgeColor(0, 0, 0)
actor.GetProperty().SetLineWidth(3)
actor.GetProperty().SetEdgeVisibility(True)

renderer.AddActor(actor)
render_window.Render()

render_window_interactor.Initialize()

out = None

if video_filename != "":
    out = cv.VideoWriter(video_filename, cv.VideoWriter_fourcc('m','p','4','v'), 30.0, (width, height), 1)

cb = animate_callback(subdivide_mesh, subdivision_method, subdivisions, render_window, out)
cb.actor = actor
render_window_interactor.AddObserver('TimerEvent', cb.update)
timerId = render_window_interactor.CreateRepeatingTimer(33);

render_window_interactor.GetInteractorStyle().SetCurrentStyleToTrackballCamera()
render_window_interactor.Start()



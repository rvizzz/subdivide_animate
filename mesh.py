import numpy as np

class Mesh:

    def __init__(self):
        self.vertices = []
        self.edges = []
        self.faces = []
        
        self.vertex_dict = {}
        self.edge_dict = {}
    
    def num_vertices(self):
        return len(self.vertices)
        
    def num_edges(self):
        return len(self.edges)
        
    def num_faces(self):
        return len(self.faces)
    
    def print_edges(self):
        
        for e in self.edges:
            print(str(e.adj_vertices[0].coor) + " " + str(e.adj_vertices[1].coor))
    
    def add_face_by_points(self, points):
        
        face_vertices = []
        
        for p in points:
            face_vertices.append(self.add_vertex(p))
            
        self.add_face(face_vertices)
    
    def add_face_by_indicies(self, indices):
        
        face_vertices = []
        
        for i in indices:
            face_vertices.append(self.vertices[i])
            
        self.add_face(face_vertices)
    
    def add_face(self, face_vertices):
    
        #make face edges
        face_edges = []
    
        for i in range(len(face_vertices)):
            
            v1 = face_vertices[i]
            v2 = face_vertices[(i + 1) % len(face_vertices)]
            
            face_edges.append(self.add_edge(v1, v2))
    
        face = Face(face_vertices, face_edges)
        self.faces.append(face)
        
        #add face to vertices
        for v in face_vertices:
            v.add_face(face)
            
        #add face to edges
        for e in face_edges:
            e.add_face(face)
            
        #ignore adjacent faces
        
            
    def add_edge(self, v1, v2):
                
        v1t = tuple(v1.coor.tolist())
        v2t = tuple(v2.coor.tolist())
        
        if (v1t, v2t) in self.edge_dict:
            return self.edges[self.edge_dict[(v1t, v2t)]]
        
        new_edge = Edge(v1, v2)
        self.edges.append(new_edge)
        
        id = len(self.edges) - 1
        
        self.edge_dict[(v1t, v2t)] = id
        self.edge_dict[((v2t, v1t))] = id
        
        v1.add_edge(new_edge)
        v2.add_edge(new_edge)
        
        v1.add_vertex(v2)
        v2.add_vertex(v1)
        
        return new_edge
            
    def add_vertex(self, p):
    
        pt = tuple(p.tolist())
        
        if pt in self.vertex_dict:
            return self.vertices[self.vertex_dict[pt]]
        
        new_vertex = Vertex(p)
        
        self.vertices.append(new_vertex)
        id = len(self.vertices) - 1
        
        self.vertex_dict[pt] = id
        
        new_vertex.set_id(id)
        
        return new_vertex
    
    def compute_mid_edge_surface(self):
    
        me_mesh = Mesh()
        
        me_animate_mesh = MeshAnimate()
        
        for e in self.edges:
            e.compute_mid_edge_point()
            
        for f in self.faces:
        
            poly = []
            
            for e in f.adj_edges:
                poly.append(e.me_point)
                
            me_mesh.add_face_by_points(poly)
             
            point_pairs = []
            
            for i in range(len(f.adj_vertices)):
            
                e1_p = f.adj_edges[(i - 1) % len(f.adj_edges)].me_point
                e2_p = f.adj_edges[i].me_point
            
                pp1 = [f.adj_vertices[i].coor, e1_p]
                pp2 = [f.adj_vertices[i].coor, e2_p]
                
                point_pairs.append(pp1)
                point_pairs.append(pp2)
            
            me_animate_mesh.add_face_by_points(point_pairs)
            
        for v in self.vertices:
        
            if len(v.adj_faces) < 1:
                continue
                
            start_f = v.adj_faces[0]
            f = start_f
            
            poly = []
            point_pairs = []
            
            broke_on_boundary = False
            
            while True:
        
                index = f.adj_vertices.index(v)
                index = (index - 1) % len(f.adj_vertices)
                
                e = f.adj_edges[index]
                
                poly.append(e.me_point)
                point_pairs.append([v.coor, e.me_point])
                
                if len(e.adj_faces) < 2:
                    broke_on_boundary = True
                    break
                
                if e.adj_faces[0] == f:
                    f = e.adj_faces[1]
                else:
                    f = e.adj_faces[0]
            
                if f == start_f:
                    break
            
            if broke_on_boundary:
                f = start_f
                
                while True:
                
                    index = f.adj_vertices.index(v)
                    
                    e = f.adj_edges[index]
                    
                    poly.insert(0, e.me_point)
                    point_pairs.insert(0, [v.coor, e.me_point])
                    
                    if len(e.adj_faces) < 2:
                        break
                        
                    if e.adj_faces[0] == f:
                        f = e.adj_faces[1]
                    else:
                        f = e.adj_faces[0]
                
                    if f == start_f:
                        break
                        
                poly.insert(0, v.coor)
                point_pairs.insert(0, [v.coor, v.coor])
            
            me_mesh.add_face_by_points(poly)
            me_animate_mesh.add_face_by_points(point_pairs)
            
        return (me_mesh, me_animate_mesh)
        
    
    def compute_catmull_clark_surface(self):
        
        cc_mesh = Mesh()
        
        cc_animate_mesh = MeshAnimate()
        
        for f in self.faces:
            f.compute_catmull_clark_point()
            
        for e in self.edges:
            e.compute_catmull_clark_point()
            
        for v in self.vertices:
            v.compute_catmull_clark_point()
            
        for f in self.faces:
        
            polys = []
            
            for i in range(len(f.adj_edges)):
                e1_point = f.adj_edges[(i - 1) % len(f.adj_edges)].cc_point
                e2_point = f.adj_edges[i].cc_point
                v_point = f.adj_vertices[i].cc_point
                f_point = f.cc_point
                
                poly = [e1_point, v_point, e2_point, f_point]
                polys.append(poly)
                
                cc_mesh.add_face_by_points(poly)
                
            face_point_pairs = []
            
            for i in range(len(f.adj_edges)):
                pp = [f.adj_vertices[i].coor, f.cc_point]
                face_point_pairs.append(pp)
            
            cc_animate_mesh.add_face_by_points(face_point_pairs)
            
            for i in range(len(f.adj_edges)):
                
                v1_point = f.adj_vertices[i].coor
                v2_point = f.adj_vertices[(i + 1) % len(f.adj_vertices)].coor
                
                e_point = f.adj_edges[i].cc_point
                
                f_point = f.cc_point
                
                edge_point_pairs = [[v1_point, e_point], [v2_point, e_point], [v2_point, f_point], [v1_point, f_point]]
                
                cc_animate_mesh.add_face_by_points(edge_point_pairs)
                
            for i in range(len(f.adj_edges)):
            
                vertex_point_pairs = []
                
                for p in polys[i]:
                    vertex_point_pairs.append([f.adj_vertices[i].coor, p])
                    
                cc_animate_mesh.add_face_by_points(vertex_point_pairs)
                
        
        return (cc_mesh, cc_animate_mesh)
        
    def compute_doo_sabin_surface(self, clamp = False):
        
        ds_mesh = Mesh()
        ds_animate_mesh = MeshAnimate()
        
        for f in self.faces:
            f.compute_doo_sabin_face_point()
            
        for e in self.edges:
            e.compute_doo_sabin_point()
            
        for f in self.faces:
            f.compute_doo_sabin_points()
            
        #generate face-faces
        for f in self.faces:
            ds_mesh.add_face_by_points(f.ds_points)
            
        #generate edge-faces
        for e in self.edges:
            
            if e.on_boundary():
                continue
            
            f1 = e.adj_faces[0]
            f2 = e.adj_faces[1]
            
            f1_index = f1.adj_edges.index(e)
            f2_index = f2.adj_edges.index(e)
            
            poly = []
            point_pairs = []
            
            poly.append(f1.ds_points[(f1_index + 1) % len(f1.ds_points)])
            poly.append(f1.ds_points[f1_index])
            poly.append(f2.ds_points[(f2_index + 1) % len(f2.ds_points)])
            poly.append(f2.ds_points[f2_index])
            
            point_pairs.append([f1.adj_vertices[(f1_index + 1) % len(f1.adj_vertices)].coor, poly[0]])
            point_pairs.append([f1.adj_vertices[f1_index].coor, poly[1]])
            point_pairs.append([f2.adj_vertices[(f2_index + 1) % len(f2.adj_vertices)].coor, poly[2]])
            point_pairs.append([f2.adj_vertices[f2_index].coor, poly[3]])
            
            ds_mesh.add_face_by_points(poly)
            ds_animate_mesh.add_face_by_points(point_pairs)
            
        #generate vertex-faces
        for v in self.vertices:
            
            if v.on_boundary():
                continue
                
            start_f = v.adj_faces[0]
            f = start_f
            
            poly = []
            point_pairs = []
            
            while True:
        
                index = f.adj_vertices.index(v)
                
                poly.append(f.ds_points[index])
                point_pairs.append([v.coor, f.ds_points[index]])
                
                e = f.adj_edges[(index - 1) % len(f.ds_points)]
                
                if e.adj_faces[0] == f:
                    f = e.adj_faces[1]
                else:
                    f = e.adj_faces[0]
            
                if f == start_f:
                    break
            
            ds_mesh.add_face_by_points(poly)
            ds_animate_mesh.add_face_by_points(point_pairs)
            
        for f in self.faces:
            
            point_pairs = []
            
            for i in range(len(f.adj_vertices)):
                pp = [f.adj_vertices[i].coor, f.ds_points[i]]
                point_pairs.append(pp)
                
            ds_animate_mesh.add_face_by_points(point_pairs)
        
        
        return (ds_mesh, ds_animate_mesh)

class MeshAnimate(Mesh):
    
    def __init__(self):
        super().__init__()
        
    def add_vertex(self, p, q):
        
        t = (tuple(p.tolist()), tuple(q.tolist()))
        if t in self.vertex_dict:
            return self.vertices[self.vertex_dict[t]]
        
        new_vertex = VertexAnimate(p, q)
        self.vertices.append(new_vertex)
        
        id = len(self.vertices) - 1
        
        self.vertex_dict[t] = id
        
        new_vertex.set_id(id)
        
        return new_vertex
        
    def add_edge(self, v1, v2):
                
        v1t = (tuple(v1.coor_1.tolist()), tuple(v1.coor_2.tolist()))
        v2t = (tuple(v2.coor_1.tolist()), tuple(v2.coor_2.tolist()))
        
        if (v1t, v2t) in self.edge_dict:
            return self.edges[self.edge_dict[(v1t, v2t)]]
        
        new_edge = Edge(v1, v2)
        self.edges.append(new_edge)
        
        id = len(self.edges) - 1
        
        self.edge_dict[(v1t, v2t)] = id
        self.edge_dict[((v2t, v1t))] = id
        
        v1.add_edge(new_edge)
        v2.add_edge(new_edge)
        
        v1.add_vertex(v2)
        v2.add_vertex(v1)
        
        return new_edge
        
    def add_face_by_points(self, point_pairs):
        
        face_vertices = []
        
        for pp in point_pairs:
            face_vertices.append(self.add_vertex(pp[0], pp[1]))
            
        self.add_face(face_vertices)

class TopoElement:
    
    def __init__(self):
        self.adj_vertices = []
        self.adj_edges = []
        self.adj_faces = []
        self.clamp_to_boundary = False
    
    def add_vertex(self, v):
        self.adj_vertices.append(v)
    
    def add_edge(self, e):
        self.adj_edges.append(e)
    
    def add_face(self, f):
        self.adj_faces.append(f)

    def get_adj_vertices():
        return self.adj_vertices
    
    def get_adj_edges():
        return self.adj_edges
    
    def get_adj_faces():
        return self.adj_faces


class VertexAnimate(TopoElement):

    def __init__(self, start, end):
        super().__init__()
        
        self.coor_1 = start
        self.coor_2 = end
        
    def set_id(self, i):
        self.id = i
        
    def get_val(self, t = 0):
        return (1 - t) * self.coor_1 + t * self.coor_2

class Vertex(TopoElement):
    
    def __init__(self, p):
        super().__init__()
        
        self.coor = p[:]
        
    def set_id(self, i):
        self.id = i
    
    def compute_catmull_clark_point(self):
        
        if self.on_boundary():
            self.cc_point = self.coor[:]
        else:
        
            num_points = len(self.adj_vertices)
        
            f_point = np.sum([f.cc_point for f in self.adj_faces], axis = 0) / len(self.adj_faces)
            e_point = (np.sum([v.coor for v in self.adj_vertices], axis = 0) + (self.coor * num_points))
            e_point = e_point / (2 * num_points)
            
            self.cc_point = (f_point + 2 * e_point + (num_points - 3) * self.coor) / num_points
            
    def on_boundary(self):
        
        for e in self.adj_edges:
            if e.on_boundary():
                return True
        
        return len(self.adj_faces) == 0
        
    def get_val(self):
        return self.coor

class Edge(TopoElement):

    def __init__(self, p, q):
        super().__init__()
        
        self.adj_vertices.append(p)
        self.adj_vertices.append(q)

    def on_boundary(self):
        return len(self.adj_faces) < 2

    def compute_catmull_clark_point(self):
        
        if self.on_boundary():
            self.cc_point = np.sum([v.coor for v in self.adj_vertices], axis = 0)
            self.cc_point = self.cc_point / len(self.adj_vertices)
        else:
            self.cc_point = np.sum([v.coor for v in self.adj_vertices], axis = 0)
            self.cc_point += np.sum([f.cc_point for f in self.adj_faces], axis = 0)
            self.cc_point = self.cc_point / (len(self.adj_faces) + 2)
            
    def compute_doo_sabin_point(self):
        
        self.ds_point = np.sum([v.coor for v in self.adj_vertices], axis = 0)
        self.ds_point = self.ds_point / len(self.adj_vertices)
            
    def compute_mid_edge_point(self):
    
        self.me_point = np.sum([v.coor for v in self.adj_vertices], axis = 0)
        self.me_point = self.me_point / len(self.adj_vertices)
        
    

class Face(TopoElement):

    def __init__(self, vertex_list, edge_list):
        super().__init__()
        
        self.adj_vertices = vertex_list[:]
        self.adj_edges = edge_list[:]
        
    def compute_catmull_clark_point(self):
    
        self.cc_point = self.get_average_point()
      
    def compute_doo_sabin_face_point(self):
    
        self.ds_point = self.get_average_point()
        
    def get_average_point(self):
    
        avg_point = np.sum([v.coor for v in self.adj_vertices], axis = 0)
        avg_point = avg_point / len(self.adj_vertices)
        
        return avg_point
        
    def compute_doo_sabin_points(self):
    
        self.ds_points = []
        
        for i in range(len(self.adj_vertices)):
            
            if self.adj_vertices[i].on_boundary():
                point = self.adj_vertices[i].coor[:]
            
                self.ds_points.append(point)
            else:
                e1_p = self.adj_edges[(i - 1) % len(self.adj_vertices)].ds_point
                e2_p = self.adj_edges[i].ds_point
                v_p = self.adj_vertices[i].coor
                f_p = self.ds_point
                
                point = (e1_p + e2_p + v_p + f_p) / 4
            
                self.ds_points.append(point)
        
        
        

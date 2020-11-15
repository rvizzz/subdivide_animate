# subdivide_animate
Create 2D or 3D animations of subdivision surface algorithms

# 3D examples:

Doo-Sabin example: https://twitter.com/r_vizzz/status/1326339990624858113

Catmull-Clark example: https://twitter.com/r_vizzz/status/1325839951431405568

# 2D example:

Midedge example: https://twitter.com/r_vizzz/status/1328117443668176897

Doo-Sabin example: https://twitter.com/r_vizzz/status/1326983763000254470

# How to use:

Currently supports Catmull-Clark and Doo-Sabin subdivisions

usage: python3 subdivide_animate.py [-h] [-a A] [-d D] [-v V] mesh_file

example: python3 subdivide_animate.py -a 1 -d 2 -v cube_animation.mp4 data/cube.off

positional arguments:

    mesh_file   an .off or .obj mesh file to subdivide

optional arguments:

    -h, --help  show this help message and exit
  
    -a [1, 2, 3]     subdivision algorithm to visualize
  
              	Subdivision algorithms:
                
              		1: Catmull-Clark
                  
              		2: Doo-Sabin
                    
                    3: Midedge
                  
    -d D        number of subdivisions to animate
  
    -v V        video file to output animation to
  

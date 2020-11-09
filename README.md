# subdivide_animate
Create animations of subdivision surface algorithms

Currently supports Catmull-Clark and Doo-Sabin subdivisions

usage: python3 subdivide_animate.py [-h] [-a A] [-d D] [-v V] mesh_file

positional arguments:

    mesh_file   an .off or .obj mesh file to subdivide

optional arguments:

    -h, --help  show this help message and exit
  
    -a A        subdivision algorithm to visualize
  
              	Subdivision algorithms:
                
              		1: Catmull-Clark
                  
              		2: Doo-Sabin
                  
    -d D        number of subdivisions to animate
  
    -v V        video file to output animation to
  

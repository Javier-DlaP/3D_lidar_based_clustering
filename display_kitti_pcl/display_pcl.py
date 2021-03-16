import mayavi.mlab as mlab
import numpy as np
from tracklet import Object3D, get_objects
import argparse
import os
import math

def main():
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--frame", help="number of the frame to show", type=int)
    args = parser.parse_args()

    path = "data/"+str(sorted(os.listdir("data/"))[args.frame])
    print(path)
    
    with open(path, 'rb') as fid:
        data_array = np.fromfile(fid, np.float32)

    #print(data_array)

    points = np.zeros((len(data_array)//4, 4), dtype=np.float32)
    for i in range(len(data_array)//4):
        points[i][0] = data_array[i*4+0]
        points[i][1] = data_array[i*4+1]
        points[i][2] = data_array[i*4+2]
        points[i][3] = data_array[i*4+3]

    #print(points)

    fig = mlab.figure(figure=None, bgcolor=(0, 0, 0), fgcolor=None, engine=None, size=(1000, 800))
    mlab.points3d(points[:, 0], points[:, 1], points[:, 2], mode='point', color=(1,1,1),
                  colormap="copper", scale_factor=10, figure=fig)

    
    mlab.plot3d([0, 15], [0, 0], [0, 0], color=(1, 0, 0), tube_radius=None)
    mlab.plot3d([0, 0], [0, 15], [0, 0], color=(1, 0, 0), tube_radius=None)
    mlab.plot3d([0, 0], [0, 0], [0, 15], color=(1, 0, 0), tube_radius=None)
    mlab.text3d(15, -1, +1, 'X', color=(1, 0, 0), scale=1.)
    mlab.text3d(0, 15, +1, 'Y', color=(1, 0, 0), scale=1.)
    mlab.text3d(0, -1, 15, 'Z', color=(1, 0, 0), scale=1.)
    
    

    objects3D = get_objects(args.frame)
    for object3D in objects3D:
        print(str(object3D)+"\n")
        x, y, z = addBoundingBox(object3D)
        mlab.plot3d(x, y, z, color=(0,1,1), tube_radius=None)
    #mlab.plot3d([0, 10], [0, 0], [0, 0], color=(0,1,1), tube_radius=None)

    mlab.show()

def addBoundingBox(object3d):
    """
        5 -------- 7
       /|         /|
      1 -------- 3 .
      | |        | |
      . 4 ----8--- 6
      |/     9   |/
      0 -------- 2
    """
    tx = object3d.tx+object3d.w/2*math.sin(object3d.rz)-object3d.l/2*math.cos(object3d.rz)
    ty = object3d.ty-object3d.w/2*math.cos(object3d.rz)-object3d.l/2*math.sin(object3d.rz)

    p0 = [tx, ty, object3d.tz]
    p1 = [tx, ty, object3d.tz+object3d.h]
    p2 = [tx-object3d.w*math.sin(object3d.rz), ty+object3d.w*math.cos(object3d.rz), object3d.tz]
    p3 = [tx-object3d.w*math.sin(object3d.rz), ty+object3d.w*math.cos(object3d.rz), object3d.tz+object3d.h]
    p4 = [tx+object3d.l*math.cos(object3d.rz), ty+object3d.l*math.sin(object3d.rz), object3d.tz]
    p5 = [tx+object3d.l*math.cos(object3d.rz), ty+object3d.l*math.sin(object3d.rz), object3d.tz+object3d.h]
    p6 = [tx+object3d.l*math.cos(object3d.rz)-object3d.w*math.sin(object3d.rz), ty+object3d.w*math.cos(object3d.rz)+object3d.l*math.sin(object3d.rz), object3d.tz]
    p7 = [tx+object3d.l*math.cos(object3d.rz)-object3d.w*math.sin(object3d.rz), ty+object3d.w*math.cos(object3d.rz)+object3d.l*math.sin(object3d.rz), object3d.tz+object3d.h]
    p8 = [(p4[0]+p6[0])/2,(p4[1]+p6[1])/2,object3d.tz]
    p9 = [(p0[0]+p2[0]+p4[0]+p6[0])/4,(p0[1]+p2[1]+p4[1]+p6[1])/4,object3d.tz]


    x = []
    y = []
    z = []

    for elem in [p0, p2, p3, p1, p0, p4, p5, p1, p3, p7, p6, p2, p6, p8, p9, p8, p4, p5, p7]:
        x.append(elem[0])
        y.append(elem[1])
        z.append(elem[2])

    return x, y, z


if __name__ == "__main__":
    main()
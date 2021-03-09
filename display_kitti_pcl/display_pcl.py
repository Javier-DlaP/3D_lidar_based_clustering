import mayavi.mlab as mlab
import numpy as np

def main():
  
    with open("data/0000000010.bin", 'rb') as fid:
        data_array = np.fromfile(fid, np.float32)

    print(data_array)

    points = np.zeros((len(data_array)//4, 4), dtype=np.float32)
    for i in range(len(data_array)//4):
        points[i][0] = data_array[i*4+0]
        points[i][1] = data_array[i*4+1]
        points[i][2] = data_array[i*4+2]
        points[i][3] = data_array[i*4+3]

    print(points)

    fig = mlab.figure(figure=None, bgcolor=(0, 0, 0), fgcolor=None, engine=None, size=(1000, 500))
    mlab.points3d(points[:, 0], points[:, 1], points[:, 2], mode='point', color=(1,1,1),
                  colormap="copper", scale_factor=10, figure=fig)
    mlab.show()

if __name__ == "__main__":
    main()
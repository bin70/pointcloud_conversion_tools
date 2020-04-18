#!/usr/bin/python3

import os
import numpy as np
import argparse
import plyfile
from datetime import datetime
from matplotlib import cm

def save_ply(points, filename, colors=None, intensity=None, normals=None):
    vertex = np.core.records.fromarrays(points.transpose(), names='x, y, z', formats='f4, f4, f4')
    n = len(vertex)
    desc = vertex.dtype.descr

    if normals is not None:
        vertex_normal = np.core.records.fromarrays(normals.transpose(), names='nx, ny, nz', formats='f4, f4, f4')
        assert len(vertex_normal) == n
        desc = desc + vertex_normal.dtype.descr

    if intensity is not None:
        vertex_intensity = np.core.records.fromarrays(intensity.transpose(), names='intensity', formats='f4')
        assert len(vertex_intensity) == n
        desc = desc + vertex_intensity.dtype.descr

    if colors is not None:
        vertex_color = np.core.records.fromarrays(colors.transpose() * 255, names='red, green, blue',
                                                  formats='u1, u1, u1')
        assert len(vertex_color) == n
        desc = desc + vertex_color.dtype.descr

    vertex_all = np.empty(n, dtype=desc)

    for prop in vertex.dtype.names:
        vertex_all[prop] = vertex[prop]

    if normals is not None:
        for prop in vertex_normal.dtype.names:
            vertex_all[prop] = vertex_normal[prop]

    if intensity is not None:
        for prop in vertex_intensity.dtype.names:
            vertex_all[prop] = vertex_intensity[prop]

    if colors is not None:
        for prop in vertex_color.dtype.names:
            vertex_all[prop] = vertex_color[prop]

    ply = plyfile.PlyData([plyfile.PlyElement.describe(vertex_all, 'vertex')], text=False)
    if not os.path.exists(os.path.dirname(filename)):
        os.makedirs(os.path.dirname(filename))
    ply.write(filename)

def save_ply_property(points, property, property_max, filename, cmap_name='tab20'):
    point_num = points.shape[0]
    colors = np.full(points.shape, 0.5)
    cmap = cm.get_cmap(cmap_name)
    for point_idx in range(point_num):
        if property[point_idx] == 0: # 标签为0，表示未分类，是黑色的点
            colors[point_idx] = np.array([0, 0, 0])
        else:
            # 上色是按占最大类的比例，映射到tab20上进行上色，而不是模操作
            colors[point_idx] = cmap(property[point_idx] / property_max)[0, :3]
    save_ply(points, filename, colors, property)

def main():
	parser = argparse.ArgumentParser()
	parser.add_argument('--root', '-f', help='Path to data folder')
	args = parser.parse_args()

	root = args.root if args.root else '.'

	xyz_path = os.path.join(root, 'xyz.npy')
	label_path = os.path.join(root, 'pred.npy')
	xyz_label_path = os.path.join(root, 'xyz_pred.ply')

	# Load .npy file
	xyz = np.load(xyz_path)	
	label = np.loadtxt(label_path)
	print('{}-Loading {}...'.format(datetime.now(), xyz_path))
	
	label = np.reshape(label, (label.shape[0], 1))
	label_max = np.max(label)
	print('Max label = {}'.format(label_max))

	# Save as .ply file
	save_ply_property(xyz, label,  label_max, xyz_label_path)
	print('{}-Saving {}...'.format(datetime.now(), xyz_label_path))

if __name__ == '__main__':
    main()

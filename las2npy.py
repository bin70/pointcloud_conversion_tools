#!/usr/bin/python3

import os
import numpy as np
from datetime import datetime
from liblas import file
import argparse

def main():
	parser = argparse.ArgumentParser()
	parser.add_argument('--root', '-f', help='Path to data folder')
	args = parser.parse_args()

	root = args.root if args.root else '.'
	datasets = os.listdir(root) 
				
	for dataset_idx, dataset in enumerate(datasets):
                
		if(dataset.split('.')[0] != 'las'):
			continue

		filename_data = os.path.join(root, dataset) #, 'xyzrgb.npy')
		print('{}-Loading {}...'.format(datetime.now(), filename_data))
		lasfile = file.File(filename_data, mode='r')
		ii = 0
		xyz = np.zeros((np.size(lasfile), 3))
		for p in lasfile:
			xyz[ii, 0] = p.x
			xyz[ii, 1] = p.y
			xyz[ii, 2] = p.z
			ii = ii+1

		npy_folder = os.path.join(root, 'npy')
		if not os.path.exists(npy_folder):
			os.mkdir(npy_folder)
		npy_path = os.path.join(npy_folder, dataset.split('.')[0]+'.npy')
		np.save(npy_path, xyz)
		print('{}-Saving {}...'.format(datetime.now(), npy_path))

if __name__ == '__main__':
    main()

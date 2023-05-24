import pcl
import numpy as np
import os
from collections import defaultdict

class KittiLabel():
    def __init__(self, tracking_id, type, truncated, occluded, alpha, bbox, dimensions, location, rotation_y, score=0.0):
        self.tracking_id = tracking_id
        self.type = type
        self.truncated = truncated
        self.occluded = occluded
        self.alpha = alpha
        self.bbox = bbox
        self.dimensions = dimensions
        self.location = location
        self.rotation_y = rotation_y
        self.score = score
        
    def __str__(self):
        return 'tracking_id: {}, type: {}, truncated: {}, occluded: {}, alpha: {}, bbox: {}, dimensions: {}, location: {}, rotation_y: {}, score: {}'.format(
            self.tracking_id, self.type, self.truncated, self.occluded, self.alpha, self.bbox, self.dimensions, self.location, self.rotation_y, self.score)

class KittiLoader():
    def __init__(self, velodyne_path, label_file, calib_file):
        self.velodyne_path = velodyne_path
        self.label_file = label_file
        self.calib_file = calib_file
        
        self.velodyne_files = sorted(os.listdir(self.velodyne_path))
        self.calib = self.load_calib()
        self.labels = self.load_label()
        
        self.step = 0
        
    def load_velodyne_points(self, velodyne_file):
        points = np.fromfile(os.path.join(self.velodyne_path, velodyne_file), dtype=np.float32).reshape(-1, 4)
        return points

    def load_calib(self):
        calib_file = open(self.calib_file + '.txt', 'r')
        calib_lines = calib_file.readlines()
        
        return {
            'p0': np.array(calib_lines[0].strip().split(' ')[1:], dtype=np.float32).reshape(3, 4),
            'p1': np.array(calib_lines[1].strip().split(' ')[1:], dtype=np.float32).reshape(3, 4), 
            'p2': np.array(calib_lines[2].strip().split(' ')[1:], dtype=np.float32).reshape(3, 4), 
            'p3': np.array(calib_lines[3].strip().split(' ')[1:], dtype=np.float32).reshape(3, 4), 
            'r0_rect': np.array(calib_lines[4].strip().split(' ')[1:], dtype=np.float32).reshape(3, 3), 
            'tr_velo_to_cam': np.array(calib_lines[5].strip().split(' ')[1:], dtype=np.float32).reshape(3, 4), 
            'tr_imu_to_velo': np.array(calib_lines[6].strip().split(' ')[1:], dtype=np.float32).reshape(3, 4)
            }
        
        
    
    def load_label(self):
        label_file = open(self.label_file + '.txt', 'r')
        label_lines = label_file.readlines()
        
        label_dict = defaultdict(list)
        for line in label_lines:
            data = line.strip().split(' ')
            if(data[2] != 'DontCare'):
                label_dict[int(data[0])].append(KittiLabel(
                    tracking_id=int(data[1]),
                    type=data[2],
                    truncated=float(data[3]),
                    occluded=int(data[4]),
                    alpha=float(data[5]),
                    bbox=np.array(data[6:10], dtype=np.float32),
                    dimensions=np.array(data[10:13], dtype=np.float32),
                    location=np.array(data[13:16], dtype=np.float32),
                    rotation_y=float(data[16]),
                    score=float(data[17]) if len(data) == 18 else 0.0
                ))
        return label_dict
    
    def next(self):
        velodyne_file = self.velodyne_files[self.step]
        points = self.load_velodyne_points(velodyne_file)

        data = {
            'velodyne': points,
            'calib': self.calib,
            'labels': self.labels[self.step]
        }

        self.step += 1

        return data
    
    
def main():
    load_folder = '0000'
    root_path = '/home/kang/dev/data/kitti/tracking'
    velodyne_path = os.path.join(root_path, 'data_tracking_velodyne', 'training', 'velodyne', load_folder)
    label_path = os.path.join(root_path, 'data_tracking_label_2', 'training', 'label_02', load_folder)
    calib_path = os.path.join(root_path, 'data_tracking_calib', 'training', 'calib', load_folder)
    
    print('load data:')
    print('- velodyne_path: {}'.format(velodyne_path))
    print('- label_path: {}'.format(label_path))
    print('- calib_path: {}'.format(calib_path))
    
    loader = KittiLoader(velodyne_path, label_path, calib_path)
    data = loader.next()
    
if __name__ == '__main__':
    main()
import pcl
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker


class PointCloudPublisher(Node):
    def __init__(self, lidar_file, label_file, calib_file):
        super().__init__('point_cloud_publisher')
        self.point_publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)
        self.point_timer = self.create_timer(0.1, self.point_callback)

        self.bbox_publisher = self.create_publisher(MarkerArray, 'bbox', 10)
        self.point_timer = self.create_timer(0.1, self.bbox_callback)
        
        self.points = pcl.load(lidar_file)
        self.points = np.array(self.points)
        
        self.labels = open(label_file).readlines()
        self.labels = [label.strip().split(' ') for label in self.labels]
        
        self.locate = np.array([label[11:14] for label in self.labels])
        self.locate = self.locate.astype(np.float32)
        
        self.dimensions = np.array([label[8:11] for label in self.labels])
        self.dimensions = self.dimensions.astype(np.float32)
        
        self.rotations_y = np.array([label[14] for label in self.labels])
        self.rotations_y = self.rotations_y.astype(np.float32)
        
        self.calibs = open(calib_file).readlines()
        self.calibs = [calib.strip().split(' ') for calib in self.calibs]
        self.p2 = np.array(self.calibs[2][1:]).astype(np.float32)
        self.p2 = self.p2.reshape(3, 4)
        self.r0_rect = np.array(self.calibs[4][1:]).astype(np.float32)
        self.r0_rect = self.r0_rect.reshape(3, 3)
        self.tr_velo_to_cam = np.array(self.calibs[5][1:]).astype(np.float32)
        self.tr_velo_to_cam = self.tr_velo_to_cam.reshape(3, 4)
    
    def point_callback(self):
        msg = PointCloud2()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = self.points.shape[0]
        msg.fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32, count=1),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32, count=1),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32, count=1)]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = False
        
        msg.data = self.points.astype(np.float32).tobytes()
        self.point_publisher.publish(msg)
        
    def bbox_callback(self):
        markers = MarkerArray()
        for i in range(self.locate.shape[0]):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            locate_lidar = np.linalg.inv(self.tr_velo_to_cam) @ np.linalg.inv(self.r0_rect) @ np.append(self.locate[i], 1)
            
            marker.pose.position.x = float(locate_lidar[0])
            marker.pose.position.y = float(locate_lidar[1])
            marker.pose.position.z = 0.0
            
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = float(self.rotations_y[i])
            
            marker.scale.x = float(self.dimensions[i][0])
            marker.scale.y = float(self.dimensions[i][1])
            marker.scale.z = float(self.dimensions[i][2])
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            
            markers.markers.append(marker)
        
        self.bbox_publisher.publish(markers)

def main():
    rclpy.init()
    
    lidar_file = '../sample_data/points/000031.pcd'
    label_file = '../sample_data/labels/000031.txt'
    calib_file = '../sample_data/calibs/000031.txt'
    
    node = PointCloudPublisher(lidar_file, label_file, calib_file)
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
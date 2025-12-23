#!/usr/bin/env python3

# ros
import rclpy
from rclpy.node import Node

# lib
import math
import uuid
import carla
import numpy as np
from tf_transformations import euler_matrix, quaternion_from_matrix, quaternion_matrix, euler_from_quaternion

# autoware
from autoware_perception_msgs.msg import PredictedObjects

class CarlaEgoFollower(Node):
    def __init__(self, host="127.0.0.1", port=2000, hz=20.0):
        super().__init__("carla_agent_controller")
        self.get_logger().info("launch carla_agent_controller")

        # parameter initialization
        self.declare_parameter('host', "127.0.0.1")
        self.declare_parameter('port', 2000)
        self.declare_parameter('time_out', 5.0)
        self.declare_parameter('euler.roll', 3.141592653589793)
        self.declare_parameter('euler.pitch', 0.0)
        self.declare_parameter('euler.yaw', 0.0)
        self.declare_parameter('translation.x', -81655.015625)
        self.declare_parameter('translation.y', 50135.9421875)
        self.declare_parameter('translation.z', 43.09799999999389)

        # connect carla
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        time_out = self.get_parameter('time_out').get_parameter_value().double_value
        self.client = carla.Client(host, 2000)
        self.client.set_timeout(time_out)
        self.world = self.client.get_world()
        bp_lib = self.world.get_blueprint_library()
        self.veh_bp = bp_lib.find("vehicle.tesla.model3")

        # set subscriber
        self.subscription = self.create_subscription(
            PredictedObjects,
            '/perception/object_recognition/objects',
            self.callback,
            10
        )
        
        # Transformation Matrix
        self.T_mgrs_carla = np.eye(4, dtype=np.float64)
        # rotation
        roll = self.get_parameter('euler.roll').get_parameter_value().double_value
        pitch = self.get_parameter('euler.pitch').get_parameter_value().double_value
        yaw = self.get_parameter('euler.yaw').get_parameter_value().double_value
        matrix = euler_matrix(roll, pitch, yaw, axes='sxyz').astype(np.float64)
        self.T_mgrs_carla = matrix
        # translation
        self.T_mgrs_carla[0][3] = self.get_parameter('translation.x').get_parameter_value().double_value
        self.T_mgrs_carla[1][3] = self.get_parameter('translation.y').get_parameter_value().double_value
        self.T_mgrs_carla[2][3] = self.get_parameter('translation.z').get_parameter_value().double_value

        # for managing the state of NPC
        self.npc_map = {}



    def callback(self, msg):
        predictedObjects = msg.objects
        msg_uuid_set = set()
        if(msg.objects is None):
            self.npc_map = []
        for object in predictedObjects:
            spawn_pose = self.get_carla_pose(object.kinematics.initial_pose_with_covariance.pose)
            object_uuid = uuid.UUID(bytes=bytes(object.object_id.uuid)) 
            msg_uuid_set.add(object_uuid)
            self.update_npc(
                object_uuid, 
                spawn_pose
            )
        
        for object_uuid in self.npc_map.keys():
            if(object_uuid not in msg_uuid_set):
                try:
                    self.npc_map[object_uuid].destroy()
                except Exception as e:
                    self.get_logger().warning(f"{e}")
        
        return

    def get_carla_pose(self, ros_pose):
        T_mgrs = np.eye(4, dtype=np.float64)
        q = np.array([
            ros_pose.orientation.x,
            ros_pose.orientation.y,
            ros_pose.orientation.z,
            ros_pose.orientation.w
        ], dtype=np.float64)
        T_mgrs = quaternion_matrix(q).astype(np.float64)
        
        p = np.array([
            ros_pose.position.x,
            ros_pose.position.y,
            ros_pose.position.z
        ], dtype=np.float64)
        T_mgrs[0:3, 3] = p

        T_carla = np.eye(4, dtype=np.float64)
        T_carla = self.T_mgrs_carla @ T_mgrs


        position = T_carla[0:3, 3]
        quaternion = quaternion_from_matrix(T_carla)
        ros_roll, ros_pitch, ros_yaw = euler_from_quaternion(quaternion)
        spawn_pose = carla.Transform(
            carla.Location(
                x= np.float64(position[0]), 
                y= np.float64(position[1]), 
                z= np.float64(position[2])
            ), 
            carla.Rotation(
                pitch= np.float64(ros_pitch*(180.0/math.pi)), 
                yaw=   np.float64(ros_yaw*(180.0/math.pi)), 
                roll=  0.0 # np.float64(ros_roll*(180.0/math.pi))
            )
        )

        return spawn_pose

    def update_npc(self, uuid, spawn_pose):
        if(uuid in self.npc_map):
            self.get_logger().info("object is already spawn")
            spawn_pose.location.z = self.npc_map[uuid].get_transform().location.z
            try:
                self.npc_map[uuid].set_transform(spawn_pose)
            except Exception as e:
                self.get_logger().warning(f"{e}")
        else:
            self.get_logger().info("object spawn")
            try:
                self.npc_map[uuid] = self.world.spawn_actor(self.veh_bp, spawn_pose)
            except Exception as e:
                self.get_logger().warning(f"{e}")
        return

    def destroy_node(self):
        self.get_logger().info("Node is destroy")
        for actor in self.npc_map.values():
            if(actor is not None):
                actor.destroy()

def main():
    rclpy.init()
    node = None
    try:
        node = CarlaEgoFollower()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

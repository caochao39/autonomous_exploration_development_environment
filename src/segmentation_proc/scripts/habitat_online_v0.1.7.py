#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import sys
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image as ROS_Image
import transforms3d

import argparse

import math
import multiprocessing
import os
import random
import time
from enum import Enum

import numpy as np
from PIL import Image

import habitat_sim
import habitat_sim.agent
from habitat_sim.utils.common import (
    d3_40_colors_rgb,
    quat_from_coeffs,
)

from scipy.spatial.transform import Rotation as R

default_sim_settings = {
    "frame_rate": 30, # image frame rate
    "width": 640, # horizontal resolution
    "height": 360, # vertical resolution
    "hfov": "114.591560981", # horizontal FOV
    "camera_offset_z": 0, # camera z-offset
    "color_sensor": True,  # RGB sensor
    "depth_sensor": True,  # depth sensor
    "semantic_sensor": True,  # semantic sensor
    "scene": "../../vehicle_simulator/mesh/matterport/segmentations/matterport.glb",
}

parser = argparse.ArgumentParser()
parser.add_argument("--scene", type=str, default=default_sim_settings["scene"])
args = parser.parse_args()

def make_settings():
    settings = default_sim_settings.copy()
    settings["scene"] = args.scene

    return settings

settings = make_settings()

def make_cfg(settings):
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.frustum_culling = True
    sim_cfg.gpu_device_id = 0
    if not hasattr(sim_cfg, "scene_id"):
        raise RuntimeError(
            "Error: Please upgrade habitat-sim. SimulatorConfig API version mismatch"
        )
    sim_cfg.scene_id = settings["scene"]

    sensors = {
        "color_sensor": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["camera_offset_z"], 0.0],
            "sensor_subtype": habitat_sim.SensorSubType.PINHOLE,
            "hfov": settings["hfov"],
        },
        "depth_sensor": {
            "sensor_type": habitat_sim.SensorType.DEPTH,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["camera_offset_z"], 0.0],
            "sensor_subtype": habitat_sim.SensorSubType.PINHOLE,
            "hfov": settings["hfov"],
        },
        "semantic_sensor": {
            "sensor_type": habitat_sim.SensorType.SEMANTIC,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["camera_offset_z"], 0.0],
            "sensor_subtype": habitat_sim.SensorSubType.PINHOLE,
            "hfov": settings["hfov"],
        },
    }

    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        if settings[sensor_uuid]:
            sensor_spec = habitat_sim.SensorSpec()
            sensor_spec.uuid = sensor_uuid
            sensor_spec.sensor_type = sensor_params["sensor_type"]
            sensor_spec.sensor_subtype = sensor_params["sensor_subtype"]
            sensor_spec.resolution = sensor_params["resolution"]
            sensor_spec.position = sensor_params["position"]
            sensor_spec.gpu2gpu_transfer = False
            sensor_spec.parameters["hfov"] = sensor_params["hfov"]

            sensor_specs.append(sensor_spec)

    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])

class DemoRunnerType(Enum):
    BENCHMARK = 1
    EXAMPLE = 2
    AB_TEST = 3

class ABTestGroup(Enum):
    CONTROL = 1
    TEST = 2

class DemoRunner(Node):
    def __init__(self, sim_settings, simulator_demo_type):
        super().__init__('habitat_online')

        if simulator_demo_type == DemoRunnerType.EXAMPLE:
            self.set_sim_settings(sim_settings)

        start_state = self.init_common()

        self._demo_type = simulator_demo_type
        self.time = 0;
        self.time_stamp = self.get_clock().now().to_msg()   
        self.camera_roll = 0
        self.camera_pitch = 0
        self.camera_yaw = 0
        self.camera_x = 0
        self.camera_y = 0
        self.camera_z = 0.5

        self.create_subscription(Odometry, '/state_estimation', self.state_estimation_callback,2)

        if self._sim_settings["color_sensor"]:
            self.color_image_pub = self.create_publisher(ROS_Image,"/habitat_camera/color/image", 2)
            self.color_image = ROS_Image()
            self.color_image.header.frame_id = "habitat_camera"
            self.color_image.height = settings["height"]
            self.color_image.width  = settings["width"]
            self.color_image.encoding = "rgb8"
            self.color_image.step = 3 * self.color_image.width
            self.color_image.is_bigendian = False

        if self._sim_settings["depth_sensor"]:
            self.depth_image_pub = self.create_publisher(ROS_Image,"/habitat_camera/depth/image", 2)
            self.depth_image = ROS_Image()
            self.depth_image.header.frame_id = "habitat_camera"
            self.depth_image.height = settings["height"]
            self.depth_image.width  = settings["width"]
            self.depth_image.encoding = "mono8"
            self.depth_image.step = self.color_image.width
            self.depth_image.is_bigendian = False

        if self._sim_settings["semantic_sensor"]:
            self.semantic_image_pub = self.create_publisher(ROS_Image,"/habitat_camera/semantic/image", 2)
            self.semantic_image = ROS_Image()
            self.semantic_image.header.frame_id = "habitat_camera"
            self.semantic_image.height = settings["height"]
            self.semantic_image.width  = settings["width"]
            self.semantic_image.encoding = "rgb8"
            self.semantic_image.step = 3 * self.color_image.width
            self.semantic_image.is_bigendian = False
        
        rate = default_sim_settings["frame_rate"]
        timer_period = 1 / rate 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def set_sim_settings(self, sim_settings):
        self._sim_settings = sim_settings.copy()

    def publish_color_observation(self, obs):
        color_obs = obs["color_sensor"]
        color_img = Image.fromarray(color_obs, mode="RGBA")
        self.color_image.data = np.array(color_img.convert("RGB")).tobytes()
        self.color_image.header.stamp = self.time_stamp
        self.color_image_pub.publish(self.color_image)

    def publish_semantic_observation(self, obs):
        semantic_obs = obs["semantic_sensor"]
        semantic_img = Image.new("P", (semantic_obs.shape[1], semantic_obs.shape[0]))
        semantic_img.putpalette(d3_40_colors_rgb.flatten())
        semantic_img.putdata((semantic_obs.flatten() % 40).astype(np.uint8))
        self.semantic_image.data = np.array(semantic_img.convert("RGB")).tobytes()
        self.semantic_image.header.stamp = self.time_stamp
        self.semantic_image_pub.publish(self.semantic_image)

    def publish_depth_observation(self, obs):
        depth_obs = obs["depth_sensor"]
        depth_img = Image.fromarray((depth_obs / 10 * 255).astype(np.uint8), mode="L")
        self.depth_image.data = np.array(depth_img.convert("L")).tobytes()
        self.depth_image.header.stamp = self.time_stamp
        self.depth_image_pub.publish(self.depth_image)

    def init_common(self):
        self._cfg = make_cfg(self._sim_settings)
        scene_file = self._sim_settings["scene"]

        self._sim = habitat_sim.Simulator(self._cfg)

        if not self._sim.pathfinder.is_loaded:
            navmesh_settings = habitat_sim.NavMeshSettings()
            navmesh_settings.set_defaults()
            self._sim.recompute_navmesh(self._sim.pathfinder, navmesh_settings)

    def state_estimation_callback(self, msg):
        self.time_stamp = msg.header.stamp
        orientation = msg.pose.pose.orientation
        (self.camera_roll, self.camera_pitch, self.camera_yaw) = transforms3d.euler.quat2euler([orientation.w,orientation.x, orientation.y, orientation.z])
        self.camera_x = msg.pose.pose.position.x
        self.camera_y = msg.pose.pose.position.y
        self.camera_z = msg.pose.pose.position.z

    def timer_callback(self):
        roll = -self.camera_roll
        pitch = self.camera_pitch
        yaw = 1.5708 - self.camera_yaw

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        position = np.array([self.camera_x, self.camera_y, self.camera_z])
        position[1], position[2] = position[2], -position[1]
        
        agent_state = self._sim.get_agent(0).get_state()
        for sensor in agent_state.sensor_states:
            agent_state.sensor_states[sensor].position = position + np.array([0, default_sim_settings["camera_offset_z"], 0])
            agent_state.sensor_states[sensor].rotation = quat_from_coeffs(np.array([-qy, -qz, qx, qw]))

        self._sim.get_agent(0).set_state(agent_state, infer_sensor_states = False)
        observations = self._sim.step("move_forward")

        if self._sim_settings["color_sensor"]:
            self.publish_color_observation(observations)
        if self._sim_settings["depth_sensor"]:
            self.publish_depth_observation(observations)
        if self._sim_settings["semantic_sensor"]:
            self.publish_semantic_observation(observations)

        state = self._sim.last_state()

def main(args=None):
    rclpy.init(args=args)

    demo_runner = DemoRunner(settings, DemoRunnerType.EXAMPLE)

    rclpy.spin(demo_runner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    demo_runner.destroy_node()
    rclpy.shutdown()
    demo_runner._sim.close()
    del demo_runner._sim

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import argparse
import tf.transformations as tft
import yaml
import numpy as np

TRANSFORM_NODE_NAME = "T_cn_cnm1"


def parse_yaml(file_path):
    with open(file_path, 'r') as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return

        for cam_name, cam_node in data.items():
            if TRANSFORM_NODE_NAME in cam_node:
                transform_node = cam_node[TRANSFORM_NODE_NAME]
                matrix = np.array(transform_node)
                tag_str = create_origin_tag(matrix)
                print_result(cam_name, tag_str)


def create_origin_tag(transform_matrix):
    roll, pitch, yaw = tft.euler_from_matrix(transform_matrix, axes='sxyz')
    x, y, z = transform_matrix[0:3, 3]
    tag_str = f'<origin xyz="{x} {y} {z}" rpy="{roll} {pitch} {yaw}" />'
    return tag_str


def print_result(cam_name, tag_str):
    cam_number = int(cam_name[-1])
    cam_m1 = cam_number -1
    print(f"Transform from cam{cam_number} to cam{cam_m1}:")
    print(f"{tag_str}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert kalibr calibrations for usage in ROS')
    parser.add_argument('yaml', metavar='YAML-PATH', help='Path to the calibration result yaml file')

    args = parser.parse_args()
    parse_yaml(args.yaml)


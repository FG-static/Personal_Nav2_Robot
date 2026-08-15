#!/usr/bin/env python3

"""Rasterize simple SDF collision geometry into a Nav2 PGM/YAML map."""

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


FREE_VALUE = 254
OCCUPIED_VALUE = 0


@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class CollisionShape:
    name: str
    kind: str
    pose: Pose
    size_x: float
    size_y: float
    size_z: float

    def vertical_bounds(self):
        return (
            self.pose.z - 0.5 * self.size_z,
            self.pose.z + 0.5 * self.size_z,
        )

    def horizontal_bounds(self, padding=0.0):
        if self.kind == 'box':
            cosine = abs(math.cos(self.pose.yaw))
            sine = abs(math.sin(self.pose.yaw))
            extent_x = 0.5 * (cosine * self.size_x + sine * self.size_y)
            extent_y = 0.5 * (sine * self.size_x + cosine * self.size_y)
        else:
            extent_x = 0.5 * self.size_x
            extent_y = 0.5 * self.size_y
        return (
            self.pose.x - extent_x - padding,
            self.pose.y - extent_y - padding,
            self.pose.x + extent_x + padding,
            self.pose.y + extent_y + padding,
        )

    def contains(self, x, y, padding):
        dx = x - self.pose.x
        dy = y - self.pose.y
        if self.kind == 'box':
            cosine = math.cos(self.pose.yaw)
            sine = math.sin(self.pose.yaw)
            local_x = cosine * dx + sine * dy
            local_y = -sine * dx + cosine * dy
            return (
                abs(local_x) <= 0.5 * self.size_x + padding
                and abs(local_y) <= 0.5 * self.size_y + padding
            )

        radius = 0.5 * self.size_x + padding
        return dx * dx + dy * dy <= radius * radius


def parse_numbers(text, count, description):
    if text is None:
        raise ValueError(f'Missing {description}')
    values = [float(value) for value in text.split()]
    if len(values) != count:
        raise ValueError(
            f'{description} must contain {count} numbers, got {len(values)}')
    return values


def parse_pose(element):
    if element is None:
        return Pose()
    if element.get('relative_to'):
        raise ValueError(
            'SDF poses using relative_to are not supported; resolve the SDF '
            'frames or use explicit model/link/collision poses')
    values = parse_numbers(element.text, 6, 'pose')
    if element.get('degrees', 'false').lower() == 'true':
        values[3:] = [math.radians(value) for value in values[3:]]
    return Pose(*values)


def compose_pose(parent, child):
    if any(abs(value) > 1.0e-8 for value in (
            parent.roll, parent.pitch, child.roll, child.pitch)):
        raise ValueError(
            'Only yaw-rotated collision geometry is supported for 2D maps')
    cosine = math.cos(parent.yaw)
    sine = math.sin(parent.yaw)
    return Pose(
        x=parent.x + cosine * child.x - sine * child.y,
        y=parent.y + sine * child.x + cosine * child.y,
        z=parent.z + child.z,
        yaw=parent.yaw + child.yaw,
    )


def parse_collision(collision, parent_pose, model_name, link_name):
    collision_name = collision.get('name', 'unnamed_collision')
    full_name = f'{model_name}/{link_name}/{collision_name}'
    pose = compose_pose(parent_pose, parse_pose(collision.find('pose')))
    geometry = collision.find('geometry')
    if geometry is None:
        raise ValueError(f'{full_name} has no geometry element')

    box = geometry.find('box')
    if box is not None:
        size = parse_numbers(box.findtext('size'), 3, f'{full_name} box size')
        if any(value <= 0.0 for value in size):
            raise ValueError(f'{full_name} box size must be positive')
        return CollisionShape(full_name, 'box', pose, *size)

    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        radius = float(cylinder.findtext('radius', 'nan'))
        length = float(cylinder.findtext('length', 'nan'))
        if not math.isfinite(radius) or not math.isfinite(length):
            raise ValueError(f'{full_name} cylinder radius/length is missing')
        if radius <= 0.0 or length <= 0.0:
            raise ValueError(f'{full_name} cylinder radius/length must be positive')
        diameter = 2.0 * radius
        return CollisionShape(
            full_name, 'cylinder', pose, diameter, diameter, length)

    sphere = geometry.find('sphere')
    if sphere is not None:
        radius = float(sphere.findtext('radius', 'nan'))
        if not math.isfinite(radius) or radius <= 0.0:
            raise ValueError(f'{full_name} sphere radius must be positive')
        diameter = 2.0 * radius
        return CollisionShape(full_name, 'sphere', pose, diameter, diameter, diameter)

    print(f'Warning: skipping unsupported geometry {full_name}', file=sys.stderr)
    return None


def parse_model(model, parent_pose=Pose(), parent_name=''):
    model_name = model.get('name', 'unnamed_model')
    if parent_name:
        model_name = f'{parent_name}/{model_name}'
    model_pose = compose_pose(parent_pose, parse_pose(model.find('pose')))
    shapes = []

    for link in model.findall('link'):
        link_name = link.get('name', 'unnamed_link')
        link_pose = compose_pose(model_pose, parse_pose(link.find('pose')))
        for collision in link.findall('collision'):
            shape = parse_collision(collision, link_pose, model_name, link_name)
            if shape is not None:
                shapes.append(shape)

    for nested_model in model.findall('model'):
        shapes.extend(parse_model(nested_model, model_pose, model_name))
    return shapes


def load_collision_shapes(sdf_path):
    root = ET.parse(sdf_path).getroot()
    if root.tag == 'model':
        models = [root]
    elif root.tag == 'sdf':
        models = list(root.findall('model'))
        for world in root.findall('world'):
            models.extend(world.findall('model'))
            if world.findall('include'):
                print(
                    'Warning: world <include> elements are not expanded and were skipped',
                    file=sys.stderr)
    else:
        raise ValueError(f'Expected <sdf> or <model> root, got <{root.tag}>')

    if not models:
        raise ValueError('No inline <model> elements found in the SDF file')

    shapes = []
    for model in models:
        shapes.extend(parse_model(model))
    if not shapes:
        raise ValueError('No supported collision geometry found in the SDF file')
    return shapes


def select_height_intersections(shapes, z_min, z_max):
    selected = []
    for shape in shapes:
        shape_min, shape_max = shape.vertical_bounds()
        if shape_max <= z_min or shape_min >= z_max:
            continue
        selected.append(shape)
    return selected


def compute_bounds(shapes, resolution, margin):
    bounds = [shape.horizontal_bounds() for shape in shapes]
    x_min = math.floor((min(value[0] for value in bounds) - margin) / resolution) * resolution
    y_min = math.floor((min(value[1] for value in bounds) - margin) / resolution) * resolution
    x_max = math.ceil((max(value[2] for value in bounds) + margin) / resolution) * resolution
    y_max = math.ceil((max(value[3] for value in bounds) + margin) / resolution) * resolution
    return x_min, y_min, x_max, y_max


def rasterize(shapes, bounds, resolution, conservative):
    x_min, y_min, x_max, y_max = bounds
    width = int(math.ceil((x_max - x_min) / resolution))
    height = int(math.ceil((y_max - y_min) / resolution))
    if width <= 0 or height <= 0:
        raise ValueError('Map bounds produce an empty grid')

    grid = bytearray([FREE_VALUE]) * (width * height)
    padding = resolution / math.sqrt(2.0) if conservative else 0.0

    for shape in shapes:
        shape_bounds = shape.horizontal_bounds(padding)
        first_x = max(0, int(math.floor((shape_bounds[0] - x_min) / resolution)))
        first_y = max(0, int(math.floor((shape_bounds[1] - y_min) / resolution)))
        last_x = min(width - 1, int(math.floor((shape_bounds[2] - x_min) / resolution)))
        last_y = min(height - 1, int(math.floor((shape_bounds[3] - y_min) / resolution)))

        for map_y in range(first_y, last_y + 1):
            world_y = y_min + (map_y + 0.5) * resolution
            image_row = height - 1 - map_y
            row_offset = image_row * width
            for map_x in range(first_x, last_x + 1):
                world_x = x_min + (map_x + 0.5) * resolution
                if shape.contains(world_x, world_y, padding):
                    grid[row_offset + map_x] = OCCUPIED_VALUE

    occupied_cells = sum(value == OCCUPIED_VALUE for value in grid)
    return grid, width, height, occupied_cells


def write_map(output_prefix, grid, width, height, resolution, origin, force):
    pgm_path = output_prefix.with_suffix('.pgm')
    yaml_path = output_prefix.with_suffix('.yaml')
    if not force:
        existing = [path for path in (pgm_path, yaml_path) if path.exists()]
        if existing:
            paths = ', '.join(str(path) for path in existing)
            raise ValueError(f'Output already exists: {paths}; pass --force to replace it')

    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    header = f'P5\n{width} {height}\n255\n'.encode('ascii')
    pgm_path.write_bytes(header + grid)
    yaml_path.write_text(
        f'image: {pgm_path.name}\n'
        'mode: trinary\n'
        f'resolution: {resolution:.9g}\n'
        f'origin: [{origin[0]:.9g}, {origin[1]:.9g}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n',
        encoding='utf-8')
    return pgm_path, yaml_path


def create_argument_parser():
    parser = argparse.ArgumentParser(
        description='Generate a Nav2 ground-truth occupancy map from SDF collisions.')
    parser.add_argument('sdf', type=Path, help='Input model.sdf or inline world SDF')
    parser.add_argument(
        '--output-prefix', required=True, type=Path,
        help='Output path without extension; .pgm and .yaml are created')
    parser.add_argument('--resolution', type=float, default=0.02, help='Map resolution in metres')
    parser.add_argument(
        '--margin', type=float, default=0.5,
        help='Automatic bound margin in metres')
    parser.add_argument(
        '--bounds', nargs=4, type=float, metavar=('X_MIN', 'Y_MIN', 'X_MAX', 'Y_MAX'),
        help='Explicit map bounds; otherwise collision bounds plus margin are used')
    parser.add_argument(
        '--z-min', type=float, default=0.0,
        help='Robot collision envelope minimum Z')
    parser.add_argument(
        '--z-max', type=float, default=0.5,
        help='Robot collision envelope maximum Z')
    parser.add_argument(
        '--center-sampling', action='store_true',
        help='Mark only cells whose centres are inside geometry instead of conservative overlap')
    parser.add_argument('--force', action='store_true', help='Replace existing output files')
    return parser


def run(arguments):
    if arguments.resolution <= 0.0:
        raise ValueError('resolution must be positive')
    if arguments.margin < 0.0:
        raise ValueError('margin must not be negative')
    if arguments.z_max <= arguments.z_min:
        raise ValueError('z-max must be greater than z-min')
    if not arguments.sdf.is_file():
        raise ValueError(f'SDF file does not exist: {arguments.sdf}')

    shapes = load_collision_shapes(arguments.sdf)
    selected_shapes = select_height_intersections(
        shapes, arguments.z_min, arguments.z_max)
    if not selected_shapes:
        raise ValueError('No collision geometry intersects the requested Z envelope')

    if arguments.bounds:
        bounds = tuple(arguments.bounds)
        if bounds[2] <= bounds[0] or bounds[3] <= bounds[1]:
            raise ValueError('bounds must satisfy X_MAX>X_MIN and Y_MAX>Y_MIN')
    else:
        bounds = compute_bounds(selected_shapes, arguments.resolution, arguments.margin)

    grid, width, height, occupied_cells = rasterize(
        selected_shapes, bounds, arguments.resolution,
        conservative=not arguments.center_sampling)
    output_prefix = arguments.output_prefix
    if output_prefix.suffix in ('.pgm', '.yaml'):
        output_prefix = output_prefix.with_suffix('')
    pgm_path, yaml_path = write_map(
        output_prefix, grid, width, height, arguments.resolution,
        bounds[:2], arguments.force)

    print(f'Loaded collision shapes: {len(shapes)}')
    print(f'Rasterized shapes in Z [{arguments.z_min}, {arguments.z_max}): {len(selected_shapes)}')
    print(f'Map size: {width} x {height} cells at {arguments.resolution} m/cell')
    print(f'Map origin: [{bounds[0]}, {bounds[1]}, 0.0]')
    print(f'Occupied cells: {occupied_cells}')
    print(f'Wrote: {pgm_path}')
    print(f'Wrote: {yaml_path}')


def main():
    parser = create_argument_parser()
    try:
        run(parser.parse_args())
    except (OSError, ET.ParseError, ValueError) as error:
        parser.exit(1, f'error: {error}\n')


if __name__ == '__main__':
    main()

"""Read and validate the explicit inertia required by C1 model converters."""

import math

import numpy as np


def rot_xyz(r, p, y):
    """URDF roll, pitch, yaw to a column-vector rotation matrix."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def read_inertials(robot):
    """Return mass, center of mass and tensor in each URDF link frame."""
    result = {}
    for link in robot.findall('link'):
        name = link.attrib['name']
        if name in result:
            raise ValueError(f'Duplicate URDF link {name!r}')
        nodes = link.findall('inertial')
        if len(nodes) != 1:
            raise ValueError(f'URDF link {name!r} must define exactly one <inertial>, found {len(nodes)}')
        node = nodes[0]
        mass_elem, tensor_elem = node.find('mass'), node.find('inertia')
        if mass_elem is None or tensor_elem is None:
            raise ValueError(f'URDF link {name!r} must define <mass> and <inertia>')
        origin = node.find('origin')
        origin = {} if origin is None else origin.attrib
        try:
            mass = float(mass_elem.attrib['value'])
            com = np.array([float(x) for x in origin.get('xyz', '0 0 0').split()])
            rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]
            xx, yy, zz, xy, xz, yz = (
                float(tensor_elem.attrib[k]) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')
            )
        except (ValueError, KeyError) as exc:
            raise ValueError(f'URDF link {name!r} has incomplete or invalid inertia values') from exc
        if len(com) != 3 or len(rpy) != 3 or not np.isfinite([mass, *com, *rpy, xx, yy, zz, xy, xz, yz]).all():
            raise ValueError(f'URDF link {name!r} has invalid inertia values')
        if mass <= 0:
            raise ValueError(f'URDF link {name!r} must have positive mass')
        tensor = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
        moments = np.linalg.eigvalsh(tensor)
        if moments[0] <= 0 or moments[2] > moments[0] + moments[1] + 1e-12:
            raise ValueError(f'URDF link {name!r} has invalid principal moments {moments}')
        rotation = rot_xyz(*rpy)
        result[name] = (mass, com, rotation @ tensor @ rotation.T)
    if not result:
        raise ValueError('URDF contains no links with inertia')
    return result


def check_body_coverage(expected, names, context):
    actual = set(names)
    missing, extra = expected.keys() - actual, actual - expected.keys()
    if missing or extra or len(names) != len(actual):
        raise ValueError(
            f'{context} body coverage differs from URDF '
            f'(missing: {sorted(missing)}, extra: {sorted(extra)}, duplicate count: {len(names) - len(actual)})'
        )

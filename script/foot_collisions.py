"""C1 foot collision layout shared by the model generators."""
import xml.etree.ElementTree as ET


CAPSULE_CORE = 'C1 sole capsule core. The cylinder length excludes the hemispherical ends.'
SOLE_Z_MM = -50.5
RADIUS_MM = 11.5
# Straight segment limits fitted inside both C1 STL sole outlines, in ankle coordinates.
ROWS_MM = (
    ('front_outer', -41, 66.5, 124.5),
    ('front_inner', 41, 67, 124.5),
    ('mid_outer', -36, -57.5, 142.5),
    ('mid_inner', 36, -56.5, 142),
    ('strip_outer', -21, -67, 161),
    ('strip_center', 0, -69, 167),
    ('strip_inner', 21, -66.5, 161.5),
)


def is_capsule_core(shape):
    return any(child.tag is ET.Comment and child.text.strip() == CAPSULE_CORE
               for child in shape)


def apply_foot_collisions(robot):
    """Replace the source foot mesh collisions and retain the visual STLs."""
    for side, label in (('L', 'left'), ('R', 'right')):
        link = robot.find(f"link[@name='{side}_ankle_roll_link']")
        for collision in link.findall('collision'):
            link.remove(collision)
        for name, y, x_start, x_end in ROWS_MM:
            collision = ET.SubElement(link, 'collision', name=f'foot_{label}_{name}')
            collision.append(ET.Comment(f' {CAPSULE_CORE} '))
            x = (x_start + x_end) / 2
            z = SOLE_Z_MM + RADIUS_MM
            ET.SubElement(collision, 'origin', xyz=f'{x / 1000:g} {y / 1000:g} {z / 1000:g}',
                          rpy='0 1.570796326794897 0')
            geometry = ET.SubElement(collision, 'geometry')
            ET.SubElement(geometry, 'cylinder', radius=f'{RADIUS_MM / 1000:g}',
                          length=f'{(x_end - x_start) / 1000:g}')

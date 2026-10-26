#!/usr/bin/env python3
"""最小可用 URDF -> USD 转换器（usd-core only）。

关键设计：所有位姿都在 USD 的行向量约定下用 4x4 矩阵推导，
joint 的 localPos/localRot 直接由 W_parent * W_child^-1 分解得到，
因此 link 位姿与关节帧在构造上必然一致，不存在"关节与视觉打架"的问题。
"""
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

AXES = 'XYZ'


# ---------------------------------------------------------------- 基础数学
def rot_xyz(r, p, y):
    """URDF rpy -> 3x3 列向量约定矩阵 (R = Rz(y)Ry(p)Rx(r))"""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def vec(s, default=(0.0, 0.0, 0.0)):
    if s is None:
        return np.array(default, dtype=float)
    return np.array([float(x) for x in s.split()], dtype=float)


def mat_row(Rc, t):
    """列向量旋转 + 平移 -> USD 行向量 4x4（线性部分取转置，平移放在第 3 行）"""
    m = Gf.Matrix4d(1.0)
    for i in range(3):
        m.SetRow(i, Gf.Vec4d(float(Rc[0][i]), float(Rc[1][i]), float(Rc[2][i]), 0.0))
    m.SetRow(3, Gf.Vec4d(float(t[0]), float(t[1]), float(t[2]), 1.0))
    return m


def rot180_about(v):
    """绕任意单位向量旋转 180° 的 3x3（列约定）"""
    v = np.array(v, dtype=float)
    v /= np.linalg.norm(v)
    return 2.0 * np.outer(v, v) - np.eye(3)


def to_quatf(q):
    return Gf.Quatf(float(q.GetReal()),
                    Gf.Vec3f(*[float(v) for v in q.GetImaginary()]))


# ---------------------------------------------------------------- STL
def read_stl(path):
    size = os.path.getsize(path)
    with open(path, 'rb') as f:
        head = f.read(84)
        if len(head) < 84:
            raise ValueError(f'STL 太短: {path}')
        n = struct.unpack('<I', head[80:84])[0]
        if size == n * 50 + 84:                     # 二进制
            arr = np.frombuffer(f.read(n * 50), dtype=np.dtype([
                ('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')]))
            return arr['v'].reshape(-1, 3).astype(float)
    # ASCII
    vals = []
    for tok in open(path, 'r', errors='ignore'):
        for w in tok.split():
            try:
                vals.append(float(w))
            except ValueError:
                pass
    v = np.array(vals, dtype=float)
    v = v.reshape(-1, 3)
    return v


def add_mesh(stage, path, pts):
    mesh = UsdGeom.Mesh.Define(stage, path)
    n = len(pts) // 3
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * n))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(list(range(n * 3))))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mn, mx = pts.min(axis=0), pts.max(axis=0)
    mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(x) for x in mn]),
                                         Gf.Vec3f(*[float(x) for x in mx])]))
    return mesh


def resolve_mesh(urdf_dir, filename):
    if filename.startswith('package://'):
        rel = filename[len('package://'):]
        pkg, _, rest = rel.partition('/')
        cand = [os.path.join(urdf_dir, '..', rest),
                os.path.join(urdf_dir, '..', pkg, rest)]
    elif filename.startswith('file://'):
        cand = [filename[7:]]
    else:
        cand = [os.path.join(urdf_dir, filename)]
    for c in cand:
        c = os.path.normpath(c)
        if os.path.exists(c):
            return c
    return None


# ---------------------------------------------------------------- 转换
def convert(urdf_path, usd_path, root_name='walker_c1'):
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    r = ET.parse(urdf_path).getroot()
    links = {l.get('name'): l for l in r.findall('link')}
    joints = r.findall('joint')

    parent_of = {}
    for j in joints:
        parent_of[j.find('child').get('link')] = j
    roots = [n for n in links if n not in parent_of]
    if len(roots) != 1:
        raise RuntimeError(f'非单根 URDF: {roots}')
    root_link = roots[0]

    # ---- FK（行向量）：W_child = L * W_parent
    W = {root_link: Gf.Matrix4d(1.0)}
    order = [root_link]
    queue = [root_link]
    while queue:
        cur = queue.pop(0)
        for j in joints:
            if j.find('parent').get('link') != cur:
                continue
            o = j.find('origin')
            xyz = vec(o.get('xyz')) if o is not None else np.zeros(3)
            rpy = vec(o.get('rpy')) if o is not None else np.zeros(3)
            L = mat_row(rot_xyz(*rpy), xyz)
            ch = j.find('child').get('link')
            W[ch] = L * W[cur]
            order.append(ch)
            queue.append(ch)

    if os.path.exists(usd_path):
        os.remove(usd_path)
    stage = Usd.Stage.CreateNew(usd_path)
    UsdGeom.SetStageUpAxis(stage, 'Z')
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    rp = UsdGeom.Xform.Define(stage, f'/{root_name}')
    stage.SetDefaultPrim(rp.GetPrim())

    stats = dict(mesh=0, missing=[], no_visual=[], mass=0)

    # ---- links
    for name in order:
        l = links[name]
        p = UsdGeom.Xform.Define(stage, f'/{root_name}/{name}')
        p.AddTransformOp().Set(W[name])

        inertial = l.find('inertial')
        if inertial is not None:
            mass = float(inertial.find('mass').get('value'))
            o = inertial.find('origin')
            com = vec(o.get('xyz')) if o is not None else np.zeros(3)
            rpy = vec(o.get('rpy')) if o is not None else np.zeros(3)
            Ri = rot_xyz(*rpy)
            ie = inertial.find('inertia')
            I = np.array([
                [float(ie.get('ixx')), float(ie.get('ixy')), float(ie.get('ixz'))],
                [float(ie.get('ixy')), float(ie.get('iyy')), float(ie.get('iyz'))],
                [float(ie.get('ixz')), float(ie.get('iyz')), float(ie.get('izz'))],
            ])
            I = Ri @ I @ Ri.T
            I = 0.5 * (I + I.T)
            w, V = np.linalg.eigh(I)
            if np.linalg.det(V) < 0:        # 特征向量符号任意，翻一列保证右手系
                V[:, 2] *= -1.0
            api = UsdPhysics.MassAPI.Apply(p.GetPrim())
            api.CreateMassAttr(float(mass))
            api.CreateCenterOfMassAttr(Gf.Vec3f(*[float(x) for x in com]))
            api.CreateDiagonalInertiaAttr(Gf.Vec3f(*[float(x) for x in w]))
            api.CreatePrincipalAxesAttr(to_quatf(_quat_from_R(V)))
            stats['mass'] += 1
        UsdPhysics.RigidBodyAPI.Apply(p.GetPrim())

        vs = l.findall('visual')
        cs = l.findall('collision')
        if not vs:
            stats['no_visual'].append(name)
        for tag, elems, purpose, is_col in (
                ('visuals', vs, UsdGeom.Tokens.default_, False),
                ('collisions', cs, UsdGeom.Tokens.proxy, True)):
            if not elems:
                continue
            UsdGeom.Xform.Define(stage, f'/{root_name}/{name}/{tag}')
            for k, e in enumerate(elems):
                g = e.find('geometry')
                child = list(g)[0]
                gp = f'/{root_name}/{name}/{tag}/{tag[:-1]}_{k}'
                if child.tag == 'mesh':
                    fn = resolve_mesh(urdf_dir, child.get('filename'))
                    if fn is None:
                        stats['missing'].append(child.get('filename'))
                        continue
                    prim = add_mesh(stage, gp, read_stl(fn))
                    stats['mesh'] += 1
                elif child.tag == 'box':
                    sx, sy, sz = vec(child.get('size'))
                    prim = UsdGeom.Cube.Define(stage, gp)
                    prim.CreateSizeAttr(1.0)
                    UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
                elif child.tag == 'cylinder':
                    rad = float(child.get('radius'))
                    ln = float(child.get('length'))
                    prim = UsdGeom.Cylinder.Define(stage, gp)
                    prim.CreateRadiusAttr(1.0)
                    prim.CreateHeightAttr(1.0)
                    prim.CreateAxisAttr(UsdGeom.Tokens.z)
                    UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3f(rad, rad, ln))
                else:
                    continue
                prim.CreatePurposeAttr(purpose)
                if is_col:
                    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(gp))

    # ---- joints
    UsdGeom.Xform.Define(stage, f'/{root_name}/joints')
    for j in joints:
        jtype = j.get('type')
        parent = j.find('parent').get('link')
        child = j.find('child').get('link')
        path = f"/{root_name}/joints/{j.get('name')}"
        L = W[child] * W[parent].GetInverse()          # 行向量：W_child = L * W_parent

        if jtype in ('revolute', 'continuous', 'prismatic'):
            ax = j.find('axis')
            axis = vec(ax.get('xyz')) if ax is not None else np.array([1.0, 0, 0])
            axis = axis / np.linalg.norm(axis)
            idx = int(np.argmax(np.abs(axis)))
            sign = 1.0 if axis[idx] > 0 else -1.0
            if sign > 0:
                E = np.eye(3)
            else:
                perp = [0.0, 0.0, 0.0]
                perp[(idx + 1) % 3] = 1.0
                E = rot180_about(perp)
            if jtype == 'prismatic':
                jp = UsdPhysics.PrismaticJoint.Define(stage, path)
            else:
                jp = UsdPhysics.RevoluteJoint.Define(stage, path)
            jp.CreateAxisAttr(AXES[idx])
            lim = j.find('limit')
            if lim is not None and jtype != 'continuous':
                lo = float(lim.get('lower'))
                hi = float(lim.get('upper'))
                if jtype == 'revolute':
                    lo, hi = math.degrees(lo), math.degrees(hi)
                if sign < 0:
                    lo, hi = -hi, -lo
                jp.CreateLowerLimitAttr(lo)
                jp.CreateUpperLimitAttr(hi)
                eff = lim.get('effort')
                if eff is not None:
                    d = UsdPhysics.DriveAPI.Apply(jp.GetPrim(), 'angular')
                    d.CreateTypeAttr('force')
                    d.CreateMaxForceAttr(float(eff))
        elif jtype == 'fixed':
            jp = UsdPhysics.FixedJoint.Define(stage, path)
            E = np.eye(3)
        else:
            continue

        W_J = mat_row(E, np.zeros(3)) * W[child]      # 绕子体原点旋转得到关节帧
        T0 = W[parent].GetInverse() * W_J
        T1 = W[child].GetInverse() * W_J
        jp.CreateBody0Rel().SetTargets([Sdf.Path(f'/{root_name}/{parent}')])
        jp.CreateBody1Rel().SetTargets([Sdf.Path(f'/{root_name}/{child}')])
        p0 = T0.ExtractTranslation()
        p1 = T1.ExtractTranslation()
        jp.CreateLocalPos0Attr(Gf.Vec3f(float(p0[0]), float(p0[1]), float(p0[2])))
        jp.CreateLocalRot0Attr(to_quatf(T0.ExtractRotationQuat()))
        jp.CreateLocalPos1Attr(Gf.Vec3f(float(p1[0]), float(p1[1]), float(p1[2])))
        jp.CreateLocalRot1Attr(to_quatf(T1.ExtractRotationQuat()))

    UsdPhysics.ArticulationRootAPI.Apply(
        stage.GetPrimAtPath(f'/{root_name}/{root_link}'))
    stage.GetRootLayer().Save()
    return dict(links=len(links), joints=len(joints), **stats)


def _quat_from_R(R):
    """列约定 3x3 特征向量矩阵 -> Gf.Quatd（Gf.Rotation 采用行向量，内部自动转置）"""
    m = Gf.Matrix4d(1.0)
    for i in range(3):
        m.SetRow(i, Gf.Vec4d(float(R[0][i]), float(R[1][i]), float(R[2][i]), 0.0))
    return m.ExtractRotationQuat()


if __name__ == '__main__':
    info = convert(sys.argv[1], sys.argv[2])
    print('转换完成:', sys.argv[2])
    for k, v in info.items():
        if k in ('missing', 'no_visual') and not v:
            continue
        print(f'  {k}: {v}')

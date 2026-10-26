#!/usr/bin/env python3
"""把传感器（Camera / IsaacImuSensor）从参考 USD 搬到目标 USD 的对应 link 下。

用法: augment_sensors.py <参考.usd> <参考根prim> <目标.usd> <目标根prim>

位姿一律用单个 xformOp:transform 矩阵写入（与目标模型一致），
避免 translate/orient 的顺序歧义。
"""
import sys

from pxr import Gf, Sdf, Usd, UsdGeom

SKIP_ATTRS = {'xformOpOrder', 'xformOp:translate', 'xformOp:orient', 'xformOp:scale',
              'xformOp:transform', 'visibility', 'purpose'}


def main(src, src_root, dst, dst_root):
    ss = Usd.Stage.Open(src)
    ds = Usd.Stage.Open(dst)

    links = {p.GetName() for p in ds.GetPrimAtPath(f'/{dst_root}').GetChildren()
             if UsdGeom.Xform(p)}

    def world(stage, path):
        return UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())

    def find_link(prim):
        p = prim.GetParent()
        while p and str(p.GetPath()) != '/':
            if p.GetName() in links:
                return p
            p = p.GetParent()
        return None

    # 先收集，再统一写入，避免遍历过程中修改
    jobs = []
    for pr in ss.Traverse():
        if pr.GetTypeName() not in ('Camera', 'IsaacImuSensor'):
            continue
        link = find_link(pr)
        if link is None:
            print(f'  跳过（找不到所属 link）: {pr.GetPath()}')
            continue
        # 子 prim 嵌套在 link 之下：W_child = L * W_link（行向量，local 先作用）
        L = world(ss, pr.GetPath()) * world(ss, link.GetPath()).GetInverse()
        jobs.append((pr, link.GetName(), L))

    n_cam = n_imu = 0
    for pr, link_name, L in jobs:
        tname = pr.GetTypeName()
        new_path = f'/{dst_root}/{link_name}/{pr.GetName()}'
        if ds.GetPrimAtPath(new_path):
            ds.RemovePrim(new_path)

        if tname == 'Camera':
            cam = UsdGeom.Camera.Define(ds, new_path)
            xf = UsdGeom.Xformable(cam)
            xf.ClearXformOpOrder()
            xf.AddTransformOp().Set(L)
            target = cam.GetPrim()
            n_cam += 1
        else:
            spec = Sdf.CreatePrimInLayer(ds.GetRootLayer(), Sdf.Path(new_path))
            spec.specifier = Sdf.SpecifierDef
            spec.typeName = tname
            a = Sdf.AttributeSpec(spec, 'xformOp:transform', Sdf.ValueTypeNames.Matrix4d)
            a.default = L
            a = Sdf.AttributeSpec(spec, 'xformOpOrder', Sdf.ValueTypeNames.TokenArray)
            a.default = ['xformOp:transform']
            target = None
            n_imu += 1

        for at in pr.GetAttributes():
            name = at.GetName()
            if name in SKIP_ATTRS:
                continue
            val = at.Get()
            if val is None:
                continue
            if target is not None:
                if not target.HasAttribute(name):
                    target.CreateAttribute(name, at.GetTypeName())
                target.GetAttribute(name).Set(val)
            else:
                aspec = ds.GetRootLayer().GetPrimAtPath(Sdf.Path(new_path))
                s = Sdf.AttributeSpec(aspec, name, at.GetTypeName())
                s.default = val

        if target is not None:
            for rel in pr.GetRelationships():
                tg = rel.GetTargets()
                if tg:
                    target.CreateRelationship(rel.GetName()).SetTargets(tg)

    ds.GetRootLayer().Save()
    print(f'完成: camera {n_cam} 个, imu {n_imu} 个')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])

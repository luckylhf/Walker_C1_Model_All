# Walker C1 连续布颈套

这是无手版 Walker C1 的 MuJoCo 外观样件。黑色面罩随头刚性转动，灰布从后脑和下巴延伸到胸甲。抬头时前颈展开，转头时布面扭转，领边始终固定在胸口。样件使用 MuJoCo 的可视蒙皮，新增物理质量为 **0 kg**。

## 怎么看

**在本机 Mac 上**，进入本目录，双击 [查看布颈套.command](./查看布颈套.command)。窗口会对准头颈，循环演示平视、抬头、低头和左右转头。点击窗口使其获得焦点，按**空格**暂停或继续。根目录的“查看无手 MuJoCo”和“查看带手 MuJoCo”打开的是原版模型。

**从终端启动**，先进入本 README 所在目录。Mac 双击启动器优先使用 `PATH` 中的 `mjpython`，其次查找 `~/miniconda3/envs/xSIM/bin/mjpython`。也可以在自己的 MuJoCo Python 环境下直接运行下表中的命令。已验证的版本为 **MuJoCo 3.10.0**。

| 环境 | 在本目录执行 |
| --- | --- |
| macOS | `mjpython view_neck.py` |
| Linux / Windows（有图形界面） | `python view_neck.py`（这两个平台未做窗口实测） |

在已有的原生 MuJoCo 程序中，请加载 [walker_c1_neck.xml](./walker_c1_neck.xml)。搬到另一台机器时，按模型包原目录结构保留本目录、同级 `meshes/` 和 `mujoco/meshes_collision/`，XML 会通过相对路径查找这些资源。只想看效果可以直接打开[原领口对照图](./原始领口与连续灰布.png)或[动作对照视频](./头颈动作对照.mp4)。

## 布面怎样随头运动

1. **分出面罩和布面。** 生成脚本从原头部 STL 中分出硬质面罩，写入 `head_shell.stl`。后脑、下巴到胸口的灰布写入 `neck.skn`，织物纹理来自 `weave.png`。
2. **固定原领口。** 从原网格保留沿下摆向上约 10 mm 的一圈三角面，将其绑定到承载胸甲的 `waist_roll_link`。胸前弧线和肩侧拐角取自原模型，内折布边从后方遮住支架。收细发生在领口上方的颈部中段。
3. **让中段变形。** 后脑处的布随 `head_pitch_link` 运动，中段顶点按位置混合胸甲、`head_yaw_link` 和 `head_pitch_link` 的运动。这里的“权重”表示各刚体对顶点位置的影响比例。渲染器据此更新布面；拉伸力、摩擦、布料自碰和外物挤压均不在该蒙皮的计算范围内。[MuJoCo 蒙皮说明](https://mujoco.readthedocs.io/en/3.10.0/XMLreference.html#deformable-skin)

## 质量、碰撞与性能

| 项目 | 本样件实际使用的数据 |
| --- | --- |
| 质量、质心、惯量 | MJCF 各刚体的 `<inertial>`。原版与样件逐项一致，无手整机均为 **55.31013662 kg**。两个头部刚体的质量分别为 yaw **0.24151544 kg**、pitch **1.13856043 kg**。 |
| 头胸碰撞 | [碰撞凸包目录](../mujoco/meshes_collision/body/)中的 `head_yaw_link.STL`、`head_pitch_link.STL`、`waist_roll_link.STL`。面罩是关闭接触的 visual geom，灰布是 skin。物体与头部接触时，接触位置取决于刚性凸包，可能与画面上的布面有偏差。XML 显式排除了头部与胸甲之间的内部自碰撞。 |
| 仿真与渲染 | 样件增加 1 张 skin，沿用原有刚体、自由度和接触设置。查看脚本按关节角度做运动学回放。显示布面会增加顶点更新和绘制开销，帧率和训练吞吐尚未做基准测试。 |

用半径 10 mm 的球从上方探测头部，原版和样件均命中 `head_pitch_link_collision_0`；球心位于该凸包最高点上方 5 mm 时，两者报告的穿入深度均为 5 mm。

## 哪些入口能显示

| 入口 | 结果 |
| --- | --- |
| 原生 MuJoCo 加载本目录的 XML | 支持连续布颈套。自定义查看器需启用 skin 显示（`mjVIS_SKIN`），本目录的启动脚本已启用。 |
| 项目原有的无手、灵巧手、假手 MJCF | 三份正式 XML 都引用原来的刚性头颈。当前只有本目录的无手样件接入了蒙皮。 |
| URDF / RViz | 正式 URDF 仍引用原头部 STL。标准 URDF 的 link visual 随所属刚体运动，无法直接表达这里的跨刚体蒙皮。 |
| 项目随附的 USD / Isaac Sim | 现有 USD 未接入本样件；如需在 Isaac Sim 复现，需另行制作或驱动对应的 USD 蒙皮。 |
| RobotsFan 在线查看器 | 2026-11-27 实测网页 v1.5.0。最小样例将蒙皮顶点直接写在 XML 内，普通预览和“仿真”都只画出了刚性部件；相同形状改用刚性 geom 后可以显示。[RobotsFan](https://viewer.robotsfan.com/) |

**外观复核范围。** 在 247 个头部姿态中，胸甲领边顶点相对原位的最大偏差小于 0.001 mm。200 个近水平视图均未看到转头支架；扩大到 1,512 个视图后，67 个画面仍可见 1–3 个支架像素。这些数字描述样件的可见遮挡效果。

## 重新生成样件

日常查看直接使用随附文件。调整布面形状或蒙皮权重时，编辑 [build_sample.py](./build_sample.py)，在本目录执行以下命令。脚本使用 `mujoco`、`numpy`、`trimesh`、`networkx` 和 `Pillow`。

```sh
python build_sample.py
```

输入是[无手版 MJCF](../mujoco/walker_c1.xml)和[原头部网格](../meshes/body/head_pitch_link.STL)。脚本会重新写入本目录的 `walker_c1_neck.xml`、`head_shell.stl`、`neck.skn`、`weave.png`。重新打开查看器即可看到结果；对照图和视频是单独录制的演示文件。

#!/usr/bin/env python3
"""生成 RoboCup 多机器人场景 XML（Z01_std_robocup）。

做什么
------
把单机模板 Z01_std_parallel_ankle 的机器人本体复制 N 份（默认 6 台），
每份的 body/joint/site/geom/camera/sensor/actuator/tendon/equality 名字统一加
`robot_0X_` 前缀（MuJoCo 3.1.1 里重名对象会直接导致整模型加载失败），
再复用 Z01_std_football_player 的足球场场景（草坪/白线/双方球门/足球/预览点）。

为什么用生成器而不是手写
------------------------
一台机器人的 body 子树有 34 个 body、60 个 geom、19 个 site，6 台手写无法维护；
生成脚本 + check-in 的产物是唯一可重放的做法。

资产复用方式（零网格重复）
--------------------------
输出 XML 的 <compiler meshdir> 指向 Z01_std_football_player/meshes/（该目录同时
含机器人 STL 与 center_circle_ring.obj），纹理/材质/中圈网格用 <include> 引入
football_player 的 scene/soccer_field_assets.xml；球场的 worldbody（草坪/白线/
双方球门/足球/预览点）则是**内联**复制进来的——只有这样才能把足球放到场心
（football_player 把它藏在 z=-100 交给"足球模式"显隐控制），同时不改动
football_player 的资产。include 都写在主文件里，因而不依赖"include 相对哪个目录解析"。

用法
----
    python3 robot_assets/utils/gen_robocup_multi_xml.py            # 生成
    python3 robot_assets/utils/gen_robocup_multi_xml.py --check    # 只校验产物
"""

import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ASSETS_DIR = os.path.dirname(SCRIPT_DIR)  # robot_assets/

DEFAULT_TEMPLATE = os.path.join(ASSETS_DIR, "Z01_std_parallel_ankle/xml/Z01_std_parallel_ankle.xml")
DEFAULT_FIELD = os.path.join(ASSETS_DIR, "Z01_std_football_player/xml/scene/soccer_field.xml")
DEFAULT_OUTPUT = os.path.join(ASSETS_DIR, "Z01_std_robocup/xml/Z01_std_robocup.xml")
# 机器人片段的子目录（相对主 XML 所在目录）：每台一个完整子模型文件。
ROBOTS_SUBDIR = "robots"

# 生成的 XML 到场景/网格的相对路径（相对于 xml/ 目录）。
FIELD_XML_DIR = "../../Z01_std_football_player/xml/scene"
FIELD_MESH_DIR = "../../Z01_std_football_player/meshes/"

# 足球的初始位置：RoboCup 场景里球常显，必须放在场心而不是"藏"到地下。
# football_player 的 soccer_field.xml 把球放在 z=-100（由足球模式显隐控制），
# 这里内联场地时把它改到场心，从而不改动 football_player 的资产。
BALL_SPAWN = (0.0, 0.0, 0.11)
# 放置预览点（mocap body，平时不可见）：football_player 藏在 z=-100，会把模型的
# 包围盒撑到 107m（stat.extent）。放到地面下方 1m 即可（球半径 0.05，视觉上被
# 草坪遮住），包围盒回到场地量级；它带来的深度相机 frustum 变化由下面的
# VIS_MAP_* 显式对齐回 football_player 的标定。
PREVIEW_HIDDEN_Z = -1.0

# 引用型属性：它们的值是本机器人内部的 MuJoCo 对象名，必须与 name 同步加前缀。
# mesh/material/texture/class/file/objtype 指向共享资产或类型，不能加前缀。
REF_ATTRS = ("joint", "joint1", "joint2", "site", "site1", "site2", "objname",
             "tendon", "tendon1", "tendon2", "body", "body1", "body2",
             "geom", "geom1", "geom2")

# 6 台机器人的默认站位（XML 里的静态默认值；运行时由仿真节点按出生点规则重新随机）。
DEFAULT_SPAWNS = [
    (1.5, 5.2, -1.5707963267948966),   # robot_01：+y 边线外，面向场内（-y）
    (3.5, 5.2, -1.5707963267948966),   # robot_02
    (5.5, 5.2, -1.5707963267948966),   # robot_03
    (-1.5, -5.2, 1.5707963267948966),  # robot_04：-y 边线外，面向场内（+y）
    (-3.5, -5.2, 1.5707963267948966),  # robot_05
    (-5.5, -5.2, 1.5707963267948966),  # robot_06
]
BASE_Z = 0.9

# 深度相机 frustum 标定（与 sim_z01_std_football_player 对齐）。
#
# MuJoCo 的深度相机远近平面 = vis.map.znear/zfar * stat.extent，而 stat.extent 是
# 模型包围盒尺寸：football_player 把足球"藏"在 z=-100，extent=107.112，得到
# near=1.0711m / far=5355.6m；本模型球在场心、预览点只藏到地面下 1m，extent=23.000，
# 默认乘子会让深度图数值整体缩小 ~4.65 倍（实测深度值中位数 812 vs 3734），
# 导致在该仿真下训练的深度策略不能直接复用。
# 因此按 extent 比例换算乘子，使两者的实际远近平面完全一致。
FOOTBALL_EXTENT = 107.112
FOOTBALL_ZNEAR = 0.01
FOOTBALL_ZFAR = 50.0
ROBOCUP_EXTENT = 23.000
VIS_MAP_ZNEAR = FOOTBALL_ZNEAR * FOOTBALL_EXTENT / ROBOCUP_EXTENT   # ≈0.046569
VIS_MAP_ZFAR = FOOTBALL_ZFAR * FOOTBALL_EXTENT / ROBOCUP_EXTENT     # ≈232.847


def prefix_names(elem, prefix):
  """给子树内所有 name 属性与引用属性加前缀（原地修改）。

  模板里的注释在 ET.parse（未开启 insert_comments）时已被丢弃，因此不会
  跟着机器人复制 6 份。
  """
  if "name" in elem.attrib:
    elem.attrib["name"] = prefix + elem.attrib["name"]
  for attr in REF_ATTRS:
    if attr in elem.attrib:
      elem.attrib[attr] = prefix + elem.attrib[attr]
  for child in elem:
    prefix_names(child, prefix)


def copy_robot_body(template_body, index, spawn):
  """复制一台机器人的 body 子树：加前缀 + 写入默认站位。"""
  body = ET.fromstring(ET.tostring(template_body, encoding="unicode"))
  prefix_names(body, "robot_%02d_" % index)
  x, y, yaw = spawn
  body.set("pos", "%.4f %.4f %.4f" % (x, y, BASE_Z))
  # compiler angle="radian"，所以 euler 用弧度。
  body.set("euler", "0 0 %.10f" % yaw)
  return body


def prefix_block(block, prefix):
  """复制 actuator/sensor/tendon/equality 块并加前缀。"""
  clone = ET.fromstring(ET.tostring(block, encoding="unicode"))
  prefix_names(clone, prefix)
  return clone


def indent(elem, level=0):
  """把 ElementTree 结果格式化（等价于 ET.indent，但兼容更老的 python）。"""
  pad = "\n" + "  " * level
  if len(elem):
    if not elem.text or not elem.text.strip():
      elem.text = pad + "  "
    for child in elem:
      indent(child, level + 1)
      if child.tail or not child.tail.strip():
        child.tail = pad + "  "
    if not elem[-1].tail or not elem[-1].tail.strip():
      elem[-1].tail = pad
  if level and (not elem.tail or not elem.tail.strip()):
    elem.tail = pad


def field_worldbody_elements(field_path):
  """读取球场场景的 worldbody 片段，并把足球放到场心（常显场景）。"""
  field_root = ET.parse(field_path).getroot()
  # 片段文件本身就是 <worldbody> 根标签（无 <mujoco> 包装）。
  worldbody = field_root if field_root.tag == "worldbody" else field_root.find("worldbody")
  if worldbody is None:
    raise SystemExit("球场片段缺少 <worldbody>: %s" % field_path)
  elements = [ET.fromstring(ET.tostring(child, encoding="unicode")) for child in worldbody]
  seen_ball = seen_preview = False
  for body in elements:
    if body.get("name") == "soccer_ball":
      body.set("pos", "%.4f %.4f %.4f" % BALL_SPAWN)
      seen_ball = True
    elif body.get("name") == "ball_preview":
      pos = body.get("pos", "0 0 0").split()
      body.set("pos", "%.4f %.4f %.4f" % (float(pos[0]), float(pos[1]), PREVIEW_HIDDEN_Z))
      seen_preview = True
  if not seen_ball:
    raise SystemExit("球场片段里没有 soccer_ball（足球必须位于场心）")
  if not seen_preview:
    raise SystemExit("球场片段里没有 ball_preview（放置预览点）")
  return elements


def _extract_template_parts(template_path):
  """从单机模板里取出共享件（asset/default/光源/地面）与机器人本体、actuator/sensor 等。"""
  template_root = ET.parse(template_path).getroot()
  worldbody = template_root.find("worldbody")
  if worldbody is None:
    raise SystemExit("模板缺少 <worldbody>")

  robot_body = None
  shared_worldbody = []
  for child in list(worldbody):
    if child.tag == "body" and robot_body is None:
      robot_body = child
      continue
    shared_worldbody.append(child)
  if robot_body is None:
    raise SystemExit("模板 worldbody 里没有机器人 body")

  asset = template_root.find("asset")
  actuator = template_root.find("actuator")
  sensor = template_root.find("sensor")
  default = template_root.find("default")
  if asset is None or actuator is None or sensor is None or default is None:
    raise SystemExit("模板缺少 <asset>/<actuator>/<sensor>/<default>")
  return {
      "root": template_root,
      "robot_body": robot_body,
      "shared_worldbody": shared_worldbody,
      "asset": asset,
      "actuator": actuator,
      "sensor": sensor,
      "default": default,
      "tendon_blocks": list(template_root.findall("tendon")),
      "equality_blocks": list(template_root.findall("equality")),
  }


def build_robot_model(parts, index, spawn):
  """生成一台机器人的**完整子模型**（根标签 <mujoco>），供主 XML 用 <include> 引入。

  为什么是完整子模型而不是 body 片段：MuJoCo 的 <include> 是把片段的子元素合并到
  include 所在的位置，只有根为 <mujoco> 的片段才能让 worldbody / actuator /
  sensor / tendon / equality 各自归位（3.1.1 实测支持：body 落在 worldbody 前段、
  执行器与传感器按 include 顺序合并，qpos 顺序 = 文档顺序）。
  """
  prefix = "robot_%02d_" % (index + 1)
  root = ET.Element("mujoco", {"model": "robot_%02d" % (index + 1)})
  root.append(ET.Comment(
      " 由 robot_assets/utils/gen_robocup_multi_xml.py 生成，请勿手改；"
      "机器人描述的唯一真源是 Z01_std_parallel_ankle/xml/Z01_std_parallel_ankle.xml"))

  world = ET.SubElement(root, "worldbody")
  world.append(copy_robot_body(parts["robot_body"], index + 1, spawn))

  # actuator/sensor：模板里是"容器"标签，逐个复制其子元素并加前缀。
  actuators = ET.SubElement(root, "actuator")
  for child in list(parts["actuator"]):
    actuators.append(prefix_block(child, prefix))
  sensors = ET.SubElement(root, "sensor")
  for child in list(parts["sensor"]):
    sensors.append(prefix_block(child, prefix))
  # tendon/equality 同理：容器只保留一层，避免 <equality><equality> 这类嵌套。
  tendons = ET.SubElement(root, "tendon")
  for block in parts["tendon_blocks"]:
    for child in list(block):
      tendons.append(prefix_block(child, prefix))
  equalities = ET.SubElement(root, "equality")
  for block in parts["equality_blocks"]:
    for child in list(block):
      equalities.append(prefix_block(child, prefix))

  indent(root)
  return ET.ElementTree(root)


def build_main(parts, field_path, robot_count, njmax, nconmax):
  """生成主 XML：只留全局设置 + 6 台机器人的 <include> + 共享场地/光源/足球。

  机器人本体在 robots/robot_0X.xml（生成物），主文件不再包含 6 段重复描述。
  """
  root = ET.Element("mujoco", {"model": "Z01_std_robocup"})
  root.append(ET.Comment(
      " 由 robot_assets/utils/gen_robocup_multi_xml.py 生成，请勿手改；"
      "6 台机器人本体在 robots/robot_0X.xml（同样是生成物），"
      "唯一真源是 Z01_std_parallel_ankle/xml/Z01_std_parallel_ankle.xml"))

  ET.SubElement(root, "compiler", {"angle": "radian", "meshdir": FIELD_MESH_DIR})
  ET.SubElement(root, "size", {"njmax": str(njmax), "nconmax": str(nconmax)})
  # 深度相机 frustum 与 football_player 对齐（见文件头 VIS_MAP_* 的说明）。
  visual = ET.SubElement(root, "visual")
  ET.SubElement(visual, "map", {"znear": "%.6f" % VIS_MAP_ZNEAR,
                                "zfar": "%.6f" % VIS_MAP_ZFAR})

  default_clone = ET.fromstring(ET.tostring(parts["default"], encoding="unicode"))
  root.append(default_clone)

  # 共享资产（网格/纹理/材质）只写一份；球场资源用 <include> 引入。
  asset_block = ET.Element("asset")
  for child in list(parts["asset"]):
    asset_block.append(ET.fromstring(ET.tostring(child, encoding="unicode")))
  asset_block.append(ET.Element("include", {"file": "%s/soccer_field_assets.xml" % FIELD_XML_DIR}))
  root.append(asset_block)

  # 6 台机器人：每台一个完整子模型片段。include 必须排在主文件自己的 <worldbody>
  # 之前，这样 6 台基座的 free joint 才会连续占据 qpos 前段（球场/足球在最后）。
  for index in range(robot_count):
    root.append(ET.Element("include",
                           {"file": "%s/robot_%02d.xml" % (ROBOTS_SUBDIR, index + 1)}))

  # 共享地面/光源 + 球场（内联：唯一需要改的是足球初始位置，内联后不必动
  # football_player 的球场资产）。
  world = ET.Element("worldbody")
  for child in parts["shared_worldbody"]:
    world.append(ET.fromstring(ET.tostring(child, encoding="unicode")))
  for element in field_worldbody_elements(field_path):
    world.append(element)
  root.append(world)

  indent(root)
  return ET.ElementTree(root)


def build(template_path, field_path, robot_count, njmax, nconmax, spawns):
  """返回 (主模型 tree, {片段路径: tree})。"""
  parts = _extract_template_parts(template_path)
  main_tree = build_main(parts, field_path, robot_count, njmax, nconmax)
  output_dir = os.path.dirname(DEFAULT_OUTPUT)
  fragments = {}
  for index in range(robot_count):
    path = os.path.join(output_dir, ROBOTS_SUBDIR, "robot_%02d.xml" % (index + 1))
    fragments[path] = build_robot_model(parts, index, spawns[index])
  return main_tree, fragments


def write_xml(tree, output_path):
  os.makedirs(os.path.dirname(output_path), exist_ok=True)
  body = ET.tostring(tree.getroot(), encoding="unicode")
  with open(output_path, "w", encoding="utf-8") as handle:
    handle.write('<?xml version="1.0" encoding="utf-8"?>\n')
    handle.write(body.rstrip() + "\n")


def expand_includes(path, base=None, seen=None):
  """把 <include file="..."/> 就地展开。

  include 路径按主文件目录解析（与 MuJoCo/header_generator 一致）：被 include
  的片段若再 include 别的文件，也仍然相对主文件，而不是相对片段自己。
  """
  seen = set() if seen is None else seen
  base = os.path.dirname(os.path.abspath(path)) if base is None else base
  with open(path, encoding="utf-8") as handle:
    text = handle.read()
  out, pos = [], 0
  for match in re.finditer(r'<include\s+file="([^"]+)"\s*/>', text):
    out.append(text[pos:match.start()])
    fragment = os.path.normpath(os.path.join(base, match.group(1)))
    if fragment not in seen:
      seen.add(fragment)
      sub = expand_includes(fragment, base, seen)
      # 片段可能是"完整子模型"（根 <mujoco>）：展开后剥掉包装，让校验看到的
      # 等价于 MuJoCo 的段合并结果（worldbody/actuator/... 各归其位）。
      sub = re.sub(r'^\s*<\?xml[^>]*\?>', '', sub)
      sub = re.sub(r'<mujoco\b[^>]*>', '', sub, count=1)
      sub = re.sub(r'</mujoco>\s*$', '', sub)
      out.append(sub)
    pos = match.end()
  out.append(text[pos:])
  return "".join(out)


def collect_names(root):
  """收集所有具名对象，按类型返回 {type: [names]}（用于重名校验）。"""
  per_type = {}
  for elem in root.iter():
    if elem.tag is ET.Comment:
      continue
    name = elem.get("name")
    if not name:
      continue
    # body 里的 mesh/texture/material 是共享资产，也一并参与重名校验。
    per_type.setdefault(elem.tag, []).append(name)
  return per_type


def check(output_path):
  """校验产物：include 展开后无重名、打印模型规模与各 free joint 地址。"""
  text = expand_includes(output_path)
  # 展开后 <include> 已被替换，直接解析整份 XML。
  text = re.sub(r'^\s*<\?xml[^>]*\?>', '', text)
  root = ET.fromstring(text)

  per_type = collect_names(root)
  problems = []
  for tag, names in sorted(per_type.items()):
    duplicates = sorted({n for n in names if names.count(n) > 1})
    if duplicates:
      problems.append("%s 重名: %s" % (tag, duplicates[:5]))

  # 关节规模与 free joint 地址（qpos/dof 是全局偏移，不能假设从 0 开始）。
  dof_width = {"free": 7, "ball": 4, "slide": 1, "hinge": 1}
  qpos, dof = 0, 0
  free_joints = []
  bodies_with_free = []

  # 片段展开后文档里会有多个 <worldbody>（每个机器人子模型各带一层），因此直接
  # 遍历所有 body，只统计挂在它自己下面的关节（不会重复计入嵌套 body 的内容）。
  for body in root.iter("body"):
    for child in body:
      if child.tag == "joint":
        joint_type = child.get("type", "hinge")
        width_q = dof_width.get(joint_type, 1)
        width_v = 6 if joint_type == "free" else width_q
        if joint_type == "free":
          free_joints.append((child.get("name", "<unnamed>"), qpos, dof,
                              body.get("name") or "?"))
        qpos += width_q
        dof += width_v
      elif child.tag == "freejoint":
        free_joints.append((child.get("name", "<unnamed>"), qpos, dof,
                            body.get("name") or "?"))
        qpos += 7
        dof += 6

  ncam = len(per_type.get("camera", []))
  nu = len(per_type.get("motor", [])) + len(per_type.get("position", []))
  print("== %s" % os.path.relpath(output_path, ASSETS_DIR))
  print("   nq=%d nv=%d nu=%d ncam=%d" % (qpos, dof, nu, ncam))
  print("   free joints:")
  for name, qpos_adr, dof_adr, body_path in free_joints:
    print("     %-28s qpos=%3d dof=%3d (%s)" % (name, qpos_adr, dof_adr, body_path))
  sensor_tags = ("gyro", "framequat", "accelerometer", "velocimeter")
  print("   body=%d joint=%d geom=%d site=%d camera=%d motor=%d spatial=%d equality=%d sensor=%d"
        % (len(per_type.get("body", [])), len(per_type.get("joint", [])),
           len(per_type.get("geom", [])), len(per_type.get("site", [])),
           len(per_type.get("camera", [])), len(per_type.get("motor", [])),
           len(per_type.get("spatial", [])),
           # <equality> 的成员是 <tendon> 元素（在 per_type 里按 tag 归到 tendon）。
           len(per_type.get("tendon", [])),
           sum(len(per_type.get(tag, [])) for tag in sensor_tags)))

  if problems:
    print("   [FAIL] " + "; ".join(problems))
    return 1

  # 有 python mujoco 时再做一次真实加载校验（版本可能高于运行时，只作自洽性检查）。
  try:
    import mujoco  # noqa: PLC0415
  except ImportError:
    print("   [ok] 名字唯一（未安装 python mujoco，跳过模型加载校验）")
    return 0
  try:
    model = mujoco.MjModel.from_xml_path(os.path.abspath(output_path))
  except Exception as exc:  # pylint: disable=broad-except
    print("   [FAIL] MuJoCo 加载失败: %s" % exc)
    return 1
  print("   [ok] 名字唯一, MuJoCo 加载成功 (mj %s): nq=%d nv=%d nu=%d ncam=%d ngeom=%d"
        % (mujoco.__version__, model.nq, model.nv, model.nu, model.ncam, model.ngeom))
  near = model.vis.map.znear * model.stat.extent
  far = model.vis.map.zfar * model.stat.extent
  target_near = FOOTBALL_ZNEAR * FOOTBALL_EXTENT
  target_far = FOOTBALL_ZFAR * FOOTBALL_EXTENT
  print("   深度相机 frustum: near=%.4f m far=%.1f m（目标 near=%.4f far=%.1f，与 football_player 一致）"
        % (near, far, target_near, target_far))
  if abs(near - target_near) > 0.02 * target_near or abs(far - target_far) > 0.02 * target_far:
    print("   [FAIL] 深度相机 frustum 与目标偏差过大：模型包围盒（stat.extent=%.3f）变了？"
          "请按比例更新 VIS_MAP_ZNEAR/ZFAR" % model.stat.extent)
    return 1
  if model.nq != qpos:
    print("   [FAIL] 自查 nq=%d 与模型 nq=%d 不一致" % (qpos, model.nq))
    return 1
  return 0


def main():
  parser = argparse.ArgumentParser(description=__doc__,
                                   formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--template", default=DEFAULT_TEMPLATE, help="单机机器人模板 XML")
  parser.add_argument("--field", default=DEFAULT_FIELD, help="球场场景片段 XML（football_player）")
  parser.add_argument("--output", default=DEFAULT_OUTPUT, help="输出 XML")
  parser.add_argument("--count", type=int, default=len(DEFAULT_SPAWNS), help="机器人数量")
  parser.add_argument("--njmax", type=int, default=4000,
                      help="约束 Jacobian 行数上限（6 台人形接触/约束规模远大于单机）")
  parser.add_argument("--nconmax", type=int, default=2000, help="接触点上限")
  parser.add_argument("--check", action="store_true", help="只校验既有产物，不重新生成")
  args = parser.parse_args()

  if args.count != len(DEFAULT_SPAWNS):
    raise SystemExit("机器人数量只支持 %d（站位表写死）" % len(DEFAULT_SPAWNS))

  if not args.check:
    main_tree, fragments = build(args.template, args.field, args.count, args.njmax, args.nconmax,
                                 DEFAULT_SPAWNS)
    write_xml(main_tree, args.output)
    print("已生成 %s" % os.path.relpath(args.output, os.getcwd()))
    for path, tree in sorted(fragments.items()):
      write_xml(tree, path)
      print("已生成 %s" % os.path.relpath(path, os.getcwd()))

  return check(args.output)


if __name__ == "__main__":
  sys.exit(main())

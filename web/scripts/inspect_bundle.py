#!/usr/bin/env python3
"""Offline inventory check. Actual ONNX signatures are probed by the browser."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

p=argparse.ArgumentParser(description='Inspect a local RUBI MJCF + terrain ONNX bundle; no upload.')
p.add_argument('directory',type=Path)
args=p.parse_args()
root=args.directory.resolve()
xml=root/'rubi.xml'
if not xml.is_file():
    sys.exit('Missing rubi.xml')
source=xml.read_text()
if '<!DOCTYPE' in source or '<!ENTITY' in source:
    sys.exit('External XML entities are unsupported')
doc=ET.fromstring(source)
if doc.tag!='mujoco':
    sys.exit('Expected MuJoCo MJCF')
compiler=doc.find('compiler')
dirs={} if compiler is None else compiler.attrib
required=['encoder.onnx','policy.onnx']
for asset in doc.findall('./asset/*'):
    if asset.get('file'):
        prefix=dirs.get('meshdir','') if asset.tag=='mesh' else dirs.get('texturedir','') if asset.tag=='texture' else ''
        required.append(str(Path(prefix)/asset.get('file')))
missing=[name for name in required if not (root/name).is_file() and not (root/Path(name).name).is_file()]
print(json.dumps({'xml':str(xml),'requiredFiles':required,'missing':missing,'includes':[e.get('file') for e in doc.findall('./include')],'actuators':[e.get('name') for e in doc.findall('./actuator/*')],'expectedNetworks':{'encoder.onnx':'float32[330] -> float32[32]','policy.onnx':'float32[65] -> float32[6]'},'note':'File presence only. Browser checks tensor names, shapes, motors and simulation. Walking is not validated by this inventory.'},indent=2))
sys.exit(1 if missing else 0)

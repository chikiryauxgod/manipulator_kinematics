"""Aggregate numerical properties produced by GoogleTest (one or many XMLs)."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

parser=argparse.ArgumentParser()
parser.add_argument("xml_path",type=Path)
args=parser.parse_args()
files=[args.xml_path] if args.xml_path.is_file() else sorted(args.xml_path.rglob("*.xml"))
if not files: raise SystemExit("No GoogleTest XML results found")
errors={"max_position_error":[],"max_rotation_error":[]}
for path in files:
    root=ET.parse(path).getroot()
    if root.findall(".//failure"): raise SystemExit(f"Failed tests in {path}")
    for item in root.findall(".//property"):
        if item.attrib["name"] in errors:
            errors[item.attrib["name"]].append(float(item.attrib["value"]))
for name,values in errors.items():
    if not values: raise SystemExit(f"No numerical measurements for {name}")
    print(f"{name}: {max(values):.18e}")

#!/usr/bin/python
from __future__ import print_function
import yaml
import os

d = os.path.dirname(__file__)
if d: os.chdir(d)

f = open(".travis.yml", "r")
doc = yaml.load(f)
for cmd in doc["script"]:
    print(cmd)
    os.system(cmd)


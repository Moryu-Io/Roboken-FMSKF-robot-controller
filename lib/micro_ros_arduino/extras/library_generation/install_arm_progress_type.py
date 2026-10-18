#!/usr/bin/env python3
"""Install only ArmExecutionProgress headers and archive members from a build."""
from pathlib import Path
import shutil
import re
import subprocess
import sys
import tempfile

build, library = map(Path, sys.argv[1:])
source = build / 'include'
target = library / 'src'
# The new introspection object must use the bundled runtime's field layout.
def declaration_tokens(path):
    text = re.sub(r'/\*.*?\*/|//[^\n]*', '', path.read_text(), flags=re.DOTALL)
    return re.findall(r'\w+|[^\s]', text)

for package, header in [
    ('rosidl_typesupport_introspection_c', 'message_introspection.h'),
    ('rosidl_runtime_c', 'message_type_support_struct.h'),
    ('std_msgs', 'msg/detail/header__struct.h'),
]:
    if declaration_tokens(source / package / package / header) != declaration_tokens(target / package / header):
        raise RuntimeError(f'Bundled runtime ABI differs: {package}/{header}')
archive = target / 'imxrt1062/fpv5-d16-hard/libmicroros.a'
new_archive = build / 'libmicroros.a'
members = subprocess.check_output(['ar', 't', str(new_archive)], text=True).splitlines()
members = [name for name in members if 'arm_execution_progress' in name]
if len(members) != 4:
    raise RuntimeError(f'Expected four generated objects, got {members}')
with tempfile.TemporaryDirectory() as temporary:
    objects = []
    for member in members:
        obj = Path(temporary) / member
        obj.write_bytes(subprocess.check_output(['ar', 'p', str(new_archive), member]))
        objects.append(str(obj))
    replacement = Path(temporary) / 'libmicroros.a'
    shutil.copy2(archive, replacement)
    subprocess.run(['ar', 'r', str(replacement), *objects], check=True)
    subprocess.run(['ranlib', str(replacement)], check=True)
    shutil.copy2(replacement, archive)
shutil.copy2(source / 'interfaces/interfaces/msg/arm_execution_progress.h', target / 'interfaces/msg')
for header in (source / 'interfaces/interfaces/msg/detail').glob('arm_execution_progress*.h'):
    shutil.copy2(header, target / 'interfaces/msg/detail')

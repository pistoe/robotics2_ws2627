# separate script instead of heredoc, see containerfile
import subprocess
import yaml
import mujoco

with open('/opt/panda/versions.lock.yaml') as source:
    pinned = yaml.safe_load(source)['packages']

for name, expected in pinned.items():
    if name.startswith(('ros-', 'libgl')):
        actual = subprocess.check_output(
            ['dpkg-query', '-W', '-f=${Version}', name],
            text=True
        )
        assert actual == expected, (
            f'{name}: expected {expected}, installed {actual}'
        )

assert mujoco.__version__ == pinned['python-mujoco']

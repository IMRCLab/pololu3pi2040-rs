from setuptools import find_packages, setup
from setuptools.command.install_scripts import install_scripts
import os
import sys
from glob import glob

# class CustomInstallScripts(install_scripts):
#     def run(self):
#         install_scripts.run(self)
#         target_python = "/home/lndw/wmr-ros/.venv/bin/python3"
#         if not os.path.exists(target_python):
#             print(f"Custom python path {target_python} not found. Skipping shebang fix.")
#             return

#         for script in self.get_outputs():
#             # Only fix specific scripts to avoid over-patching
#             if os.path.basename(script) in ['cmcgs_plan', 'wmr_controller_node', 'mpc_controller_node']:
#                 print(f"Patching shebang for {script} to {target_python}")
#                 try:
#                     with open(script, 'r') as f:
#                         lines = f.readlines()
#                     if lines and lines[0].startswith('#!'):
#                         lines[0] = f'#!{target_python}\n'
#                         with open(script, 'w') as f:
#                             f.writelines(lines)
#                 except Exception as e:
#                     print(f"Failed to patch shebang for {script}: {e}")


class CustomInstallScripts(install_scripts):
    def run(self):
        install_scripts.run(self)

        default_python = "/home/lndw/wmr-ros/.venv/bin/python3"

        for script in self.get_outputs():
            script_name = os.path.basename(script)

            if script_name == 'rl_controller_node':
                target_python = (
                    "/home/lndw/wmr-ros/ROS/src/wmr_controller/"
                    "external/realtime-dbastar/baselines/"
                    "rl-examples/.venv/bin/python3"
                )
            elif script_name in [
                'cmcgs_plan',
                'wmr_controller_node',
                'mpc_controller_node',
            ]:
                target_python = default_python
            else:
                continue

            if not os.path.exists(target_python):
                print(
                    f"Python {target_python} not found; "
                    f"skipping {script_name}"
                )
                continue

            with open(script, 'r', encoding='utf-8') as file:
                lines = file.readlines()

            if lines and lines[0].startswith('#!'):
                lines[0] = f'#!{target_python}\n'

            with open(script, 'w', encoding='utf-8') as file:
                file.writelines(lines)
                

package_name = 'wmr_controller'

def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'deps/wmr-simulator/scripts'), glob('deps/wmr-simulator/scripts/*.py')),
        (os.path.join('share', package_name, 'external/realtime-dbastar/baselines/wmr-simulator/scripts'), glob('external/realtime-dbastar/baselines/wmr-simulator/scripts/*.py')),
        (os.path.join('share', package_name, 'external/realtime-dbastar/baselines/wmr-simulator/problems/benchmark'), glob('external/realtime-dbastar/baselines/wmr-simulator/problems/benchmark/*.yaml'))
    ]
    
    # Add external/ann-cmcgs-async recursively
    base_install_path = os.path.join('share', package_name, 'external/ann-cmcgs-async')
    base_source_path = 'external/ann-cmcgs-async'
    if os.path.exists(base_source_path):
        for root, dirs, files in os.walk(base_source_path):
            if 'venv' in root or '.git' in root or '__pycache__' in root:
                continue
            # Get relative path from base source
            rel_path = os.path.relpath(root, base_source_path)
            if rel_path == '.':
                install_path = base_install_path
            else:
                install_path = os.path.join(base_install_path, rel_path)
                
            file_list = [os.path.join(root, f) for f in files if not f.endswith('.pyc')]
            if file_list:
                data_files.append((install_path, file_list))

    # Add external/dbastar recursively
    base_install_path = os.path.join('share', package_name, 'external/realtime-dbastar')
    base_source_path = 'external/realtime-dbastar'
    if os.path.exists(base_source_path):
        for root, dirs, files in os.walk(base_source_path):
            # These directories contain generated, machine-local, or stale
            # files and must not become ROS package data.  Pruning dirs also
            # prevents os.walk() from descending into them.
            dirs[:] = [
                directory for directory in dirs
                if directory not in {
                    '.git', '__pycache__', 'target', 'venv', '.venv', 'results'
                }
            ]
            # Get relative path from base source
            rel_path = os.path.relpath(root, base_source_path)
            if rel_path == '.':
                install_path = base_install_path
            else:
                install_path = os.path.join(base_install_path, rel_path)
                
            file_list = [
                os.path.join(root, filename)
                for filename in files
                if not filename.endswith('.pyc')
                and os.path.isfile(os.path.join(root, filename))
            ]
            if file_list:
                data_files.append((install_path, file_list))
    
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lndw',
    maintainer_email='lndw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    cmdclass={'install_scripts': CustomInstallScripts},
    entry_points={
        'console_scripts': [
            'wmr_controller_node = wmr_controller.wmr_controller_node:main',
            'mpc_controller_node = wmr_controller.mpc_controller_node:main',
            'dbastar_controller_node = wmr_controller.dbastar_controller_node:main',
            'reference_publisher_node = wmr_controller.reference_publisher_example:main',
            'timing_monitor = wmr_controller.timing_monitor:main',
            'rl_controller_node = wmr_controller.rl_controller_node:main',
            'obstacle_monitor = wmr_controller.obstacle_monitor:main',
        ],
    },
)

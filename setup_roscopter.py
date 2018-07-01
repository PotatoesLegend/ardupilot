import os
import sys
import shutil
import utility

# Get the location of catkin_ws
catkin_workspaces = [loc for loc in os.environ['ROS_PACKAGE_PATH'].split(':') if 'src' in loc]
if len(catkin_workspaces) == 0:
  utility.print_error('Cannot find your catkin workspace. I assume it ends with a src/ folder.')
  sys.exit(-1)
workspace = catkin_workspaces[0]
if len(catkin_workspaces) > 1:
  utility.print_warning('Warning: multiple possible catkin workspace.')
print('Using %s as the catkin workspace' % workspace)

# Copy mit_msgs, roscopter, and mavlink to workspace.
ardupilot_root = os.path.dirname(os.path.realpath(sys.argv[0]))
target_mit_msgs = os.path.join(workspace, 'mit_msgs')
if os.path.isdir(target_mit_msgs):
  shutil.rmtree(target_mit_msgs)
shutil.copytree(os.path.join(ardupilot_root, 'mit_msgs'), target_mit_msgs)
target_roscopter = os.path.join(workspace, 'roscopter')
if os.path.isdir(target_roscopter):
  shutil.rmtree(target_roscopter)
shutil.copytree(os.path.join(ardupilot_root, 'roscopter'), target_roscopter)
shutil.copytree(os.path.join(ardupilot_root, 'modules/mavlink'), os.path.join(target_roscopter, 'mavlink'))

# Build.
os.chdir(os.path.dirname(workspace))
os.system('catkin_make --pkg mit_msgs')
os.system('catkin_make --pkg roscopter')
os.chdir(os.path.join(target_roscopter, 'mavlink'))
os.system('python -m pymavlink.tools.mavgen --lang=Python --wire-protocol=1.0 --output=pymavlink/dialects/v10/ardupilotmega.py message_definitions/v1.0/ardupilotmega.xml')
os.chdir(ardupilot_root)

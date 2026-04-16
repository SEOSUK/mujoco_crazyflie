import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/seosuk/mujoco_crazyflie/src/install/flyingpen_plotter'

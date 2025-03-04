import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/roombazon/Desktop/rosWorkspace/install/gamepad_control'

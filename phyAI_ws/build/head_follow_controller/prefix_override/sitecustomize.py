import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/SpotATS_ws/phyAI_ws/install/head_follow_controller'

import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/g0kh4n_2/tennis_bot_ws/install/tennis_bot_navigation'

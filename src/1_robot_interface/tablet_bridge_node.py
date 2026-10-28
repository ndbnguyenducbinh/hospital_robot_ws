#!/usr/bin/env python3
"""Direct executable wrapper for tablet_bridge_node."""

import os
import sys

# Ensure local package path is available
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from robot_interface.tablet_bridge_node import main

if __name__ == '__main__':
    main()


#!/usr/bin/env python3
"""Convenience launcher: python scripts/run_api.py [--port 8088]"""

import sys
from pathlib import Path

# Allow running from source tree without install
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from control_center_api.main import main

if __name__ == '__main__':
    main()

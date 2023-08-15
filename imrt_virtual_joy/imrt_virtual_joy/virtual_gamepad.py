#!/usr/bin/env python

import sys

sys.dont_write_bytecode = True

from .submodule import main


if __name__ == "__main__":
    main()

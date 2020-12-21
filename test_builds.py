import os
import platform
import sys
import pathlib
import argparse
import shutil

def checkPythonVersion():
    # python 3 must be the runtime
    if sys.version_info.major < 3: 
        raise Exception(os.path.basename(__file__) + " must be executed using Python 3")

if __name__ == "__main__":
    checkPythonVersion()

    build_types = [ "Debug", "Release"]

    for btype in build_types:
        os.system("python3 ./compile.py --rebuild --cross-compile --build-type \"%s\"" % (btype))
        os.system("python3 ./compile.py --rebuild --build-type \"%s\"" % (btype))
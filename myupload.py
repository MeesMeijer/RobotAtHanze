import time
import sys
import os
from contextlib import suppress
from typing import List, Iterable, TypeVar, Sequence, Set

from docopt import docopt
from ampy.pyboard import Pyboard
from ampy.files import Files, DirectoryExistsError

port = "COM3"

board = Pyboard(port)
files = Files(board)

PROJECT_PATH = os.getcwd()

# localDirFiles = os.listdir(".")
# excludeFileList = ["upload.py", "myupload.py"]
# localFiles = [file for file in localDirFiles if file.endswith('.py') and file not in excludeFileList]
# localDirs = [path for path in localDirFiles if path.find(".") == -1]
#
# remoteRootFiles = files.ls("/", recursive=False, long_format=False)
# remoteFiles = [file.removeprefix("/") for file in remoteRootFiles if file.endswith('.py') and file not in ["/boot.py"]]
# remoteDirs = [path.removeprefix("/") for path in remoteRootFiles if path.find(".") == -1]

def isDir(name):
    if name.find(".") == -1:
        return True
    return False

def isFile(name):
    if name.endswith('.py') and name not in ["/boot.py", "/myupload.py", "upload.py"]:
        return True
    return False


toAddFiles, toAddDirss = [], []
for item in os.listdir(PROJECT_PATH):
    if isDir(item):
        toAddDirss.append(f"/{item}")
        for i in os.listdir(f"{PROJECT_PATH}/{item}"):
            if isDir(i):
                raise Exception("Dirs cant be deeper than 1. Needs work")
            elif isFile(i):
                toAddFiles.append(f"/{item}/{i}")
    elif isFile(item):
        toAddFiles.append(f"/{item}")

print("Found ", len(toAddDirss), "dirs")
print("Found ", len(toAddFiles), "files")

for ldir in toAddDirss:
    print("mkdir", ldir)
    files.mkdir(ldir, True)

print(f"\rUploading {len(toAddFiles)} files")

stat, leng = 0, len(toAddFiles)
for lfile in toAddFiles:
    per = int(stat * 100 / leng)
    print(f"\r {lfile} {per}% ({stat}):({leng})")
    with open(PROJECT_PATH+lfile, "rb") as f:
        files.put(lfile, f.read())
    stat += 1;

print(f"\r {lfile} {per}% ({stat}):({leng})")
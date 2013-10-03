import os as _os
import fnmatch as _fnmatch

def get_root_dir():
    return _os.environ['OPENHUBO_DIR']

def find_files(directory, pattern):
    for root, dirs, files in _os.walk(directory):
        for basename in files:
            if _fnmatch.fnmatch(basename, pattern):
                filename = _os.path.join(root, basename)
                yield filename

def find(rawname, path=None):
    (fpath,fname)=_os.path.split(rawname)
    #TODO: make better assumptions about name
    if not path:
        path=get_root_dir()
    for root, dirs, files in _os.walk(path+'/'+fpath):
        if fname in files:
            return _os.path.join(root, fname)

def list_robots(pattern='*.robot.xml',directory='robots'):
    filenames=[]
    for root, dirs, files in _os.walk(directory):
        for basename in files:
            if _fnmatch.fnmatch(basename, pattern):
                filenames.append( _os.path.join(root, basename))
    return filenames


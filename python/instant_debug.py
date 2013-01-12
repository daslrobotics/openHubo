import signal
import pdb

def handler(signum, frame):
  pdb.set_trace()

signal.signal(signal.SIGQUIT, handler)

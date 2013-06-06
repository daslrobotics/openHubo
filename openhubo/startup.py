# Setup script copied and slightly modified from this location
# http://www.digitalprognosis.com/opensource/scripts/pythonrc
# loading this as a module is a quick way to add interactive functionality to a script.
# This method is a bit of a hack, and will be replaced with a more elegant solution in the future

#TODO: proper exception handling here
import readline
import os
import atexit
import rlcompleter

#Overload to pass the tab character through if no completion is possible
class irlcompleter(rlcompleter.Completer):
    def complete(self, text, state):
        if text == "":
            readline.insert_text('\t')
            return None
        else:
            return rlcompleter.Completer.complete(self,text,state)

historyPath = os.path.expanduser("~/.pyhistory")
def save_history(historyPath=historyPath):
    readline.write_history_file(historyPath)

atexit.register(save_history)
# You could change this line to bind another key instead of tab.
readline.parse_and_bind("tab: complete")
readline.set_completer(irlcompleter().complete)
# Restore our command-line history, and save it when Python exits.
try:
    readline.read_history_file(historyPath)
    print "Command history restored..."
except IOError:
    print "No prior command history..."
    pass

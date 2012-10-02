# ~/.pythonrc
# enable syntax completion
try:
    import readline
except ImportError:
    print "Module readline not available."
else:
    import rlcompleter
    readline.parse_and_bind("tab: complete")
    print "Tab completion enabled via readline module."

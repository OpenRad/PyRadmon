# -*- mode: text; fill-column: 75; coding: utf-8-unix; encoding: utf-8 -*-

A refactor of PyRadmon from http://sourceforge.net/projects/pyradmon-reborn/
to upload Geiger counter data to radmon.org; our thanks
to the Auseklis Corporation of Richmond, Virginia for their great code.

This is a refactor of PyRadmon.py from radmon.org to make it more Pythonic.
The functionality of the code should be about the same, but the config file
has been extended to allow better tuning of the audio functionality.
pyaudio can be tricky to get right, and you may have more than one sound
card or audio systems on a machine.  There have been no changes to the
non-audio classes, expect a couple of minor bugfixes.

The code has been extended with command-line arguments, including
a --config option, so you can easily use different config files for 
different sound cards or computers.

On the first run, a default configuration file config.cfg
will be created. Edit it to set the type of input, and
the parameters.


Command Line Options:
  -h, --help            show this help message and exit
  -v IVERBOSITY, --verbose=IVERBOSITY
                        the Verbosity (0 - quiet, 5 - debug)
  -c SCONFIG, --config=SCONFIG
                        the configuration file (config.cfg)
  -L, --list-devices    list the available audio devices (from pyaudio)
  -D, --devices-info    list the info on the default audio devices (from
                        pyaudio)
  -s, --stdout          print the results on stdout (default False)
  -f FCPMUSVHR, --factor=FCPMUSVHR
                        convert CPM to uSv/hr to display results on stdout
                        (default 0)

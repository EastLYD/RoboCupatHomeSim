#!/bin/sh -x

SIGVERSE_X3DPARSER_CONFIG=$SIGVERSE_PATH/share/sigverse/etc/X3DParser.cfg
SIGVERSE_RUNAC=$SIGVERSE_PATH/bin/sigrunac
SIGVERSE_DATADIR=$SIGVERSE_PATH/share/sigverse/data
export LD_LIBRARY_PATH SIGVERSE_X3DPARSER_CONFIG SIGVERSE_RUNAC SIGVERSE_DATADIR

sigserver.sh -w ./CleanUpTest.xml
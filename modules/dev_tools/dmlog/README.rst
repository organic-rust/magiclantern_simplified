Canon debug log
===============

:Author: m6ii-edmac project
:License: GPL
:Summary: Logs Canon's debug messages and CRaw (CR3) encoder parameters

Replaces DryosDebugMsg and the CRaw trace logger with a RAM logger, and
dumps the parameter blocks passed to the CRaw encoder (EncInit, SetParam).
Debug > Canon debug log: start, take a CR3 still or RAW burst, stop.
The log is saved to ML/LOGS/DMLOGnn.LOG.  M6II 1.1.1 only.

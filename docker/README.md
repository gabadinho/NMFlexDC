### Dockerfile

Use this Docker configuration to generate a (non-optimized) container of Debian running EPICS 7.0.5 with SynApps 6.1 and the head version of this Nanomotion FlexDC asyn motor support.

Shell scripts to help creating the container, run it, start it, are also provided

The FlexDC controller is expected to be configured with the IP address 192.168.100.10. If this is not the case, you need to:
- Edit *ip_flexdccmd.sed* and replace _129.129.10.10_ to the correct IP address.
- Edit *Dockerfile* and uncomment the _RUN_ command that uses the above file in order to patch the address.


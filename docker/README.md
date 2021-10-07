### Dockerfile

Use this Docker configuration to generate a (non-optimized) container of Debian running EPICS 7.0.5 with SynApps 6.1 and the head version of this Nanomotion FlexDC asyn motor support.

Shell scripts to help creating the container, run it, start it, are also provided.

The configuration of **flexdcIOC** is expecting the IP address 192.168.100.10. You'll need to edit a file to patch this to the proper address (patching is already being applied, to 129.129.10.10):
- Edit **ip_flexdccmd.sed** and replace _129.129.10.10_ with the correct IP address.


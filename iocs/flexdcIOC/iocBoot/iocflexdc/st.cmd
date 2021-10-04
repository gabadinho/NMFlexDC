#!../../bin/linux-x86_64/flexdc

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/flexdc.dbd"
flexdc_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

##
< flexdc.cmd

iocInit


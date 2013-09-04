Intel 82540 support for DPDK
============================

NOTE: the need for this driver has been obsoleted by support
for 82540 in DPDK 1.3.

This repository contains a driver for supporting the Intel 82540 NIC under
[DPDK](http://dpdk.org/).  It is based on the FreeBSD driver and the DPDK
igb driver.  To use, copy `librte_pmd_wm` into to DPDK `lib` subdir and
attach it to Makefile.  Then use it like any of the other NIC drivers
provided by DPDK.  The purpose of the driver is to make simple DPDK
testing in a VM possible.  I do not recommend use in production without
careful testing.  Other 8254x NICs should be relatively easy to support.

Fun fact: this is called "wm" because I first started porting the NetBSD
driver and didn't change the name when I switched to a FreeBSD base.

Proprietary VC31 HRM Algorithm
==============================

This directory contains the object files and header for VCare's proprietary
heart-rate algorithm for the VC31/VC31B sensor.

These artifacts were created by VCare (the VC31 manufacturer) and are **not**
part of the Apache-2.0 license that covers the rest of PebbleOS. They are
vendored here under the same terms Espruino uses them.

Files
-----
- `algo.h` — Algorithm API header (`Algo_Init`, `Algo_Input`, `Algo_Output`)
- `algo.o` — Main algorithm object
- `modle5_1.o` … `modle5_18.o` — Model data objects

Usage
-----
The PebbleOS VC31 driver (`drivers/hrm/vc31/vc31.c`) calls into
`vc31_algo_adapter.c`, which wraps these symbols and converts PebbleOS sensor
/accel data into the format the blob expects.

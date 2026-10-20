<h1 align="center">Sega System 1 / System 2 FPGA Core</h1>

<p align="center">
  Arcade hardware implementation for MiSTer FPGA
</p>

---

## Overview

FPGA implementation of the **Sega System 1 and System 2 hardware (1983-1988)** in a single core.

Every set in MAME's `sega/system1.cpp` driver (92) has an MRA: System 1 (Z80 PIO and 8255 PPI boards), System 2, the Sega 315/317 and NEC MC-8123 encrypted CPUs, and the 8751 MCU games. The 8751, dial, light gun and mahjong games are the newest and still being tested.

This core targets MiSTer FPGA and aims for accurate gameplay behavior, video timing, and sound reproduction.

---

## Supported Games

| Hardware | Games |
|----------|-------|
| System 1 (PIO / PPI) | Pitfall II, Star Jacker, Up'n Down, Regulus, Mister Viking, SWAT, Flicky, Water Match, Bull Fight, Sega Ninja, Ninja Princess, I'm Sorry, TeddyBoy Blues, My Hero, Wonder Boy, Wonder Boy Deluxe, 4-D Warriors, Rafflesia, Spatter, Brain, Heavy Metal, Gardia, and clones |
| System 1 with 8751 / dial | Noboranka, Block Gal (dial) |
| System 2 | Wonder Boy in Monster Land, Choplifter, Toki no Senshi, UFO Senshi Yohko Chan, Gardia, Wonder Boy (System 2), DakkoChan House (mahjong keyboard), Shooting Master (light gun on the mouse), and clones |

Clones and regional versions are in `releases/_alternatives/`. More boards will be added as they are completed.

---

This project contains **no copyrighted game data**.

Users are responsible for obtaining and using ROM files in accordance with applicable laws.

Do not request ROM files in issues or discussions.

---

## Credits

FPGA core development: RodimusFVC  
Memory maps, decryption tables and hardware notes per the MAME project (Nicola Salmoria, David Haywood, Aaron Giles and contributors)  
T80 CPU core: Daniel Wallner, with updates by Sorgelig  
JT89 sound core, JT8255 PPI, JTMC8123 decryption and the jtframe 8751 wrapper: Jose Tejada (jotego)  
MC8051 CPU core: Oregano Systems (LGPL)  
Hiscore and pause modules: Jim Gregory and Alan Steremberg  
Original arcade games © Sega, Coreland, Westone, Escape and their respective owners

---

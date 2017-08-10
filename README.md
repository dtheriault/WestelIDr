# WestelIDr
Westel Repeater Controller

This repository contains eagle files and arduino sketch files supporting a repeater
controller based on a Teensy3.2 chip.

The hardware was originally designed to support a Westel DRB25 repeater for WB7NFX
and then updated to support the K7YCA ARES/RACES team who also had one of these
machines.  The controller hardware while supporting the Westel repeater should be
easily modified for other repeater types.  It requires +12v @ approx 150ma, PTT,
Microphone and line audio to generate simple software based carrier detect.

The controller will generate a CW ID or Voice ID in the latest revision of the
sketch file for k7yca.  Can be operated in Ham or Commercial mode with different
timeouts.

Eagle files contain schematics and PCB layout supporting a 3 channel mixer and 3
transistor simplistic audio amplifier to drive a speaker mounted on the Westel
repeater.  There's Gerber files available to spin your own board.

NOTE IMPORTANT:  Current major pin-out issue with the voice ID chip.  I'll update
files and new layout once I fix the darn chip library file.

There's also some mechanical files for a mounting plate for the Westel repeater
which were contributed by Rob KG7LMI.

CW ID code was made possible by SV1DJG and very much appreciated for allowing me to
include his software in the sketch files.

For latest software please refer to the Sketch/k7yca/k7yca.ino file as this version
matches the hardware.  The other two versions were for earlier revisions of the
board for WB7NFX and our Sheriff's Jeep Posse team where I used the controller in
an AmmoCan portable repeater I built for them.

Contact info:

email:  no1d.doug@gmail.com
repository:  https://github.com/dtheriault/WestelIDr

 License:

 Copyright (C) 2017 Douglas Theriault - NO1D

 Software is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This is distributed in the hope that it will be useful to someone or ham club/group,
 but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 more details.

 You should have received a copy of the GNU General Public License
 in the repository.  If not, see <http://www.gnu.org/licenses/>.


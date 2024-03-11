# EPOS2 Motor Controller

EPOS2 Motor Controller vor ROS2 - humble.

## Dependencies

FTDI

```console
user@pc:~$ sudo apt-get install libftdipp1-dev
```

Copy the udev rules file to your udev rules folder:

```console
sudo cp udev/91-epos2.rules /etc/udev/rules.d/.
```

Make sure that your current user is a member of the `dialout` group by running groups. If not, add the user to the group and login again.

## License

Copyright (C) 2009-2010 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Author Martí Morta Garriga  (mmorta@iri.upc.edu)
All rights reserved.

Copyright (C) 2013 Jochen Sprickerhof <jochen@sprickerhof.de>

This file is part of IRI EPOS2 Driver
IRI EPOS2 Driver is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>

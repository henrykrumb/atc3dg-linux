# atc3dg-linux #

Linux interface to NDI (formerly Ascension) trakSTAR 3D Guidance systems.
Not an official driver, please use with caution.

## Requirements ##

```bash
sudo apt install libusb-dev
```

## Installation ##

After installing the requirements, run

```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

from within the project directory.
You may need to restart your computer after installation.


## Drop-in replacement for PlusServer ##

The "igtlink_server" (compiles to "atcigtlink" executable) serves as a drop-in replacement for what would usually be done with PlusLib's PlusServer, and follows a naming convention similar to most examples you'll find on the internet.

The sensor on 1st port is assigned "Reference", and the 2nd sensor is assigned "Tool". A transform "ToolToReference" is computed as well, and all transforms
are sent via IGTLink protocol.
This is taylored to a specific application, feel free to edit the example to make it more flexible (== more sensors).

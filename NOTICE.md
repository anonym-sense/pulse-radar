# Third-party notice

This repository contains no third-party source code, firmware, models or
images. The screenshot under `docs/` was captured from this project's own GUI.

It depends on the following packages at run time. They are installed
separately and are not covered by this project's MIT license:

| Package | Source | License |
|---|---|---|
| numpy | https://github.com/numpy/numpy | BSD-3-Clause |
| scipy | https://github.com/scipy/scipy | BSD-3-Clause |
| tornado | https://github.com/tornadoweb/tornado | Apache-2.0 |
| pyserial | https://github.com/pyserial/pyserial | BSD-3-Clause |
| pyadi-iio | https://github.com/analogdevicesinc/pyadi-iio | Analog Devices BSD-style license (see its LICENSE file) |
| libiio / pylibiio | https://github.com/analogdevicesinc/libiio | LGPL-2.1-or-later |

libiio is copyleft (LGPL). This project only imports it; it does not modify
or redistribute it.

The debug-serial back-end runs the `iio_readdev`, `iio_writedev` and
`iio_attr` tools that ship with PlutoSDR firmware on the board itself, and
logs in with that firmware's publicly documented default credentials.

This project is not affiliated with or endorsed by Analog Devices. AD9361,
ADALM-PLUTO and PlutoSDR are their names for their products.

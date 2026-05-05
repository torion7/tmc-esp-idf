# tmc-esp-idf

Trinamic motion-control library for ESP-IDF (ESP32-P4 tested).

This repository is meant to be consumed as an ESP-IDF "extra components" tree
(via `EXTRA_COMPONENT_DIRS`) — typically as a git submodule of an application
project.

## Components

| Component               | Purpose                                                           |
|-------------------------|-------------------------------------------------------------------|
| `cs_mux`                | 3-bit chip-select multiplexer (e.g. 74HC138) for daisy-chain CS   |
| `tmc5160`               | Low-level register/SPI driver for TMC5160                         |
| `tmc5240`               | Low-level register/SPI driver for TMC5240                         |
| `tmc_motion`            | High-level motion API on top of `tmc5160` / `tmc5240` (open-loop) |
| `tmc_motion_closedloop` | Closed-loop encoder regulator (TMC5240 hardware ABN encoder)      |

Dependency graph:

```
tmc_motion_closedloop  -->  tmc_motion  -->  tmc5160
                                         \-> tmc5240
cs_mux                  (independent — used by application code)
```

## Using this library in a project

In the consuming application's top-level `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS lib/tmc-esp-idf)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(my_app)
```

Add this repository as a submodule:

```bash
git submodule add <repo-url> lib/tmc-esp-idf
```

In your `main/CMakeLists.txt`, list the components you actually use under
`REQUIRES`, e.g.:

```cmake
idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES cs_mux tmc5160 tmc5240 tmc_motion tmc_motion_closedloop
)
```

## Updating the library from a consumer project

```bash
# pull latest library changes into your project
git submodule update --remote --merge lib/tmc-esp-idf
git add lib/tmc-esp-idf && git commit -m "bump tmc-esp-idf"
```

## Publishing local changes back to the library

```bash
cd lib/tmc-esp-idf
# edit, commit
git push
cd -
git add lib/tmc-esp-idf && git commit -m "bump tmc-esp-idf"
```

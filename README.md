# SC-OBC Module A1 Samples

This repository contains sample applications for developers getting started
with the SC-OBC Module A1 Development Board, along with Zephyr drivers that
have not yet been upstreamed.

Detailed information about the samples, driver specifications, and build instructions
is available in the following documentation:

[SC-OBC Module A1 Product Manual (Software)](https://docs.spacecubics.com/scobc-a1/latest/software/setup.html).

## Samples

| Sample | Behavior |
| --- | --- |
| [hello_world](samples/hello_world) | Prints a greeting with the board name and revision, then exits. |
| [blinky](samples/blinky) | Toggles the user LED every second and prints its state. Uses the `led0` alias on GPIO 15. |
| [dip_switch](samples/dip_switch) | Prints the initial state of SW1[9] and subsequent changes, polling every 200 ms. Uses the active-low `sw1` alias on GPIO 14. |
| [i2c](samples/i2c) | Reads the TMP175 temperature sensor once on `i2c0` at address `0x4B` and prints the temperature. |
| [sysmon](samples/sysmon) | Enables the Board Health Monitor and reports on-board temperature 1 and the 3V3SYS bus voltage every second. |
| [sysreg](samples/sysreg) | Provides shell commands to show the boot and current Configuration Memory banks (`cfgmem show`), select bank 0 or 1 (`cfgmem select <bank>`), and request TRCH to perform a power cycle via FPGA (`pwrcycle`). |

## Repository layout

| Path | Contents |
| --- | --- |
| `samples/` | Standalone Zephyr applications, each with its own CMake and Kconfig settings. |
| `boards/shields/scobc_a1_dev/` | Development board shield definition and devicetree overlay. |
| `drivers/` | Space Cubics I2C, CAN, System Monitor, and System Register drivers. |
| `zephyr/` | Zephyr module metadata and custom devicetree bindings. |
| `west.yml` | Manifest selecting the Zephyr and OpenOCD forks and imported dependencies. |
| `.github/workflows/` | Build, commit-message, and end-of-file newline checks. |

## License

Licensed under the [Apache License 2.0](LICENSE).

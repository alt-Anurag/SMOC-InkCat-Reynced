# STM32 Firmware — Vesper Main Controller & FluxCruiser Actuator Nodes

> ⚠️ **Work in progress.** This folder is a placeholder. Firmware for both the STM32H743-based Vesper main controller and the STM32G431-based FluxCruiser actuator nodes is currently being developed and validated on evaluation modules (TMC9660-STP-EVKIT, TMC5160-BOB, ADIN1200BCP32Z-R7, CAN-FD MAX33012EASA+) before being ported to the custom PCBs. Code will be committed here incrementally as each sub-system passes hardware-in-the-loop testing.

## Planned contents

- `vesper/` — STM32H743ZIT6 supervisory firmware: sensor fusion (EKF/EWMA), self-calibration engine, inverse kinematics, quintic trajectory planning, ADRC control loop, CAN-FD/Ethernet/USB-C communication stack.
- `fluxcruiser/` — STM32G431 actuator-node firmware: TMC9660 FOC interfacing over SPI, encoder/IMU readout, CAN-FD telemetry back to Vesper, end-effector servo and brake-chopper control.
- `drivers/` — Shared low-level drivers (TMC9660 SPI driver, MSC500/PMW3389 sensor drivers, CAN-FD messaging layer).
- `tests/` — Hardware-in-the-loop test scripts used during eval-module bring-up.

No functional firmware is guaranteed to run yet. If you're cloning this repo to reference the *design*, see the root [`README.md`](../../README.md) and the POC document instead — this folder will be updated as modules pass testing.

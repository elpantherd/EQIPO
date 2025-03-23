# EQIPO
Electrostatic Quantum Inspired Path Optimization (Autonomous Navigation) for UAVs. This project is still under further refinement and optimization for high level expected performances

---

# Electrostatic Quantum-Inspired Path Optimization (EQIPO) for UAV Path Planning

This project implements an integrated UAV navigation system that combines:

- **TENG sensor data processing** for terrain mapping via triboelectric effects.
- A **sliding electrostatic grid map** construction.
- **Quantum-inspired global path planning**.
- **Pigeon-inspired local path refinement**.
- **MAVLink communication** with a Pixhawk flight controller.

The system is designed to run on a **Raspberry Pi** as a companion computer, interfacing with a Pixhawk flight controller (e.g., Pixhawk 2.4.8). It includes both simulation capabilities for testing and development, as well as support for real hardware deployment.

---

## Features

- Real-time TENG sensor data acquisition and calibration.
- Probabilistic grid mapping of terrain safety.
- Quantum-inspired candidate path generation and selection.
- Pigeon-inspired optimization for local path refinement.
- Simulation of UAV movement with battery and safety monitoring.
- MAVLink integration for flight control and telemetry.
- Extensive logging and diagnostics for development and debugging.

---

## Requirements

To run this project, you will need:

- A **Raspberry Pi** (tested with Raspberry Pi 4 Model B).
- A **Pixhawk flight controller** (e.g., Pixhawk 2.4.8) for real hardware deployment.
- **TENG sensors** connected via ADC channels for real hardware.
- **Python 3.7** or higher.
- Required Python packages: `numpy`, `scipy`, `pymavlink` (for real hardware).
- For simulation: No additional hardware is required.

---

## Installation

1. **Clone the repository:**

   ```bash
   git clone https://github.com/yourusername/eqipo.git
   cd eqipo
   ```

2. **Install dependencies:**

   ```bash
   pip install -r requirements.txt
   ```

   > **Note**: For real hardware, ensure `pymavlink` is installed. For simulation, it is optional.

3. **Set up hardware (if using real UAV):**

   - Connect the Raspberry Pi to the Pixhawk via serial (e.g., `/dev/ttyAMA0`).
   - Connect TENG sensors to ADC channels on the Raspberry Pi.

---

## Usage

To run the EQIPO navigation system in **simulation mode**:

```bash
python eqipo.py --log_level INFO
```

For **real hardware**, ensure the correct serial port is set in the code and uncomment hardware-specific lines (e.g., GPIO, smbus, pymavlink imports).

### Command-Line Arguments

- `--num_paths`: Number of candidate paths to generate (default: 50).
- `--grid_width`: Grid map width in cells (default: 20).
- `--grid_height`: Grid map height in cells (default: 20).
- `--log_level`: Logging level (DEBUG, INFO, WARNING, ERROR).
- `--log_file`: Path to log file (optional).
- `--sim_duration`: Simulation duration in seconds (default: 60.0).

---

## Configuration

The system can be configured via global parameters in the code, including:

- ADC settings (e.g., number of channels, read interval).
- Calibration constants for TENG sensors.
- Grid dimensions and cell size.
- Optimization parameters for path planning and refinement.

Adjust these parameters in the code as needed for your specific setup.

---

## Examples

- **Run with a custom grid size:**

  ```bash
  python eqipo.py --grid_width 30 --grid_height 30
  ```

- **Enable debug logging and save to a file:**

  ```bash
  python eqipo.py --log_level DEBUG --log_file debug.log
  ```

---

## Simulation vs. Real Hardware

This project includes **simulation capabilities** for testing and development without hardware. In simulation mode:

- TENG sensor readings are simulated.
- MAVLink communication is mocked.
- UAV movement is simulated with battery drain and safety checks.

For **real hardware**:

- Uncomment hardware-specific lines in the code (e.g., GPIO, smbus, pymavlink imports).
- Ensure the correct serial port and baud rate are set for the MAVLink connection.
- Calibrate TENG sensors and adjust parameters accordingly.

---

## Troubleshooting

- **Sensor readings are noisy:** Adjust calibration factors or increase grid smoothing sigma.
- **Path planning fails:** Ensure the grid map has sufficient safe areas or increase the number of candidate paths.
- **MAVLink connection issues:** Verify the serial port and baud rate in the connection string.

---
## Licensing
This software is proprietary and not open source. Please see the [LICENSE](LICENSE) file for terms of use.

---

## Contact Information

For questions or support, contact TEAM QUASARS at [dthayalan760@gmail.com].

---

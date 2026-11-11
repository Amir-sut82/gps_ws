# Wiring diagram sources

These diagrams were drawn for this workspace. The PNG files are embedded in
the main README; SVG files contain editable vector artwork and text.

- [Complete Here4 / Pixhawk / Orange Pi wiring](system-wiring.png)
  ([SVG source](system-wiring.svg)).
- [Orange Pi UART6 physical pin reference](uart6-pin-reference.png)
  ([SVG source](uart6-pin-reference.svg)).

The board-specific reference is **Orange Pi 5 Plus**, using UART6-M1 at
physical pins **8 (TX)**, **10 (RX)**, and **6 (GND)**. The Pixhawk endpoint is
**Pixhawk 6C Mini TELEM2**, using pins **2 (TX)**, **3 (RX)**, and **6 (GND)**.
TX and RX are crossed. TELEM2 pins 1, 4, and 5 are unconnected.

Drawings describe logical connections and header numbering, not the viewing
side of a particular JST cable. Identify pin 1 and check connector orientation
on the actual hardware. Here4 uses the supplied CAN cable into CAN1; the Orange
Pi UART connection does not supply either board with power.

References:

- [Holybro Pixhawk 6C Mini port pinouts](https://docs.holybro.com/autopilot/pixhawk-6c-mini/pixhawk-6c-mini-ports)
- [Orange Pi 5 Plus manufacturer manual / UART pinout](https://www.orangepi.org/orangepiwiki/index.php/Orange_Pi_5_Plus#40_Pin_Expansion_Interface_Pin_Instructions)
- [CubePilot Here4 manual](https://docs.cubepilot.org/here-4/here-4-manual)
- [PX4 1.17 DroneCAN setup and CAN1 allocation behavior](https://docs.px4.io/v1.17/en/dronecan/)

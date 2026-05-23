# ST3215 Servo Protocol Documentation

## Overview

The ST3215 servo uses a serial communication protocol that allows control and monitoring of various servo parameters. This protocol is part of the Feetech servo family and follows the STS series specification.

**Reference:** For complete protocol details, refer to the [Feetech STS3215 Servo User Manual](https://www.feetechrc.com/service-programmable-servo.html).

## Bus topology

Every wheel servo lives on a single half-duplex serial bus, distinguished only by its hardware ID.
The host writes a packet, then the addressed servo writes its reply back on the same wires.

```{graphviz}
:caption: Single serial bus, four servos
:align: center

digraph st3215_bus {
    graph [rankdir=LR, bgcolor="transparent", fontname="Roboto", nodesep=0.3, ranksep=0.6];
    node  [fontname="Roboto", fontsize=11, style="filled,rounded", penwidth=1.1];
    edge  [fontname="Roboto", fontsize=9, color="#7a6cad"];

    host [label="Host\n(perseus_lite_hardware)", shape=box, fillcolor="#1a237e", fontcolor="white"];
    bus  [label="/dev/ttyACM0\nhalf-duplex serial @ 1 Mbit", shape=cylinder, fillcolor="#37474f", fontcolor="white"];
    s1 [label="ID 1\nFL", shape=circle, fixedsize=true, width=0.75, fillcolor="#ec407a", fontcolor="white"];
    s2 [label="ID 2\nFR", shape=circle, fixedsize=true, width=0.75, fillcolor="#ec407a", fontcolor="white"];
    s3 [label="ID 3\nRL", shape=circle, fixedsize=true, width=0.75, fillcolor="#ec407a", fontcolor="white"];
    s4 [label="ID 4\nRR", shape=circle, fixedsize=true, width=0.75, fillcolor="#ec407a", fontcolor="white"];

    host -> bus  [dir=both, label="TX / RX", penwidth=2.0, color="#ec407a"];
    bus  -> s1;
    bus  -> s2;
    bus  -> s3;
    bus  -> s4;
}
```

## Memory Map

Each servo exposes two register banks — non-volatile EPROM for configuration and volatile SRAM for live command/status data.
The figure below visualises the layout; the tables that follow give the exact addresses.

```{graphviz}
:caption: Logical layout of the servo register file
:align: center

digraph st3215_mem {
    graph [rankdir=TB, bgcolor="transparent", fontname="Roboto",
           nodesep=0.25, ranksep=0.4, compound=true];
    node  [shape=box, style="filled,rounded", fontname="Roboto",
           fontsize=10, margin="0.15,0.06"];
    edge  [color="transparent"];

    subgraph cluster_eprom {
        label=<<b>EPROM</b><br/><font point-size="9">non-volatile, write-rare</font>>;
        labeljust="l"; style="rounded,filled"; fillcolor="#1a237e";
        fontcolor="#d6c8ff"; fontsize=11; color="#3949ab";

        ep_ro [label="0–4   read-only\nfirmware + hardware version", fillcolor="#311b92", fontcolor="white"];
        ep_id [label="5–8   bus identity\nID, baud, return-delay, status-level", fillcolor="#311b92", fontcolor="white"];
        ep_lim [label="9–17  motion limits\nposition / torque ceilings", fillcolor="#311b92", fontcolor="white"];
        ep_env [label="13–15 environment limits\ntemperature, voltage range", fillcolor="#311b92", fontcolor="white"];
        ep_cal [label="26–33 calibration\ndead-band, offset, operating mode", fillcolor="#311b92", fontcolor="white"];
    }

    subgraph cluster_sram {
        label=<<b>SRAM</b><br/><font point-size="9">volatile, every-tick traffic</font>>;
        labeljust="l"; style="rounded,filled"; fillcolor="#4a148c";
        fontcolor="#ffd6ea"; fontsize=11; color="#6a1b9a";

        sr_cmd  [label="40–55 commands\ntorque, goal, speed, accel", fillcolor="#880e4f", fontcolor="white"];
        sr_stat [label="56–66 live status\nposition, speed, load, moving", fillcolor="#880e4f", fontcolor="white"];
        sr_envt [label="62–63 environment\nvoltage, temperature", fillcolor="#880e4f", fontcolor="white"];
        sr_cur  [label="69–70 current draw\n(mA)", fillcolor="#880e4f", fontcolor="white"];
    }

    ep_ro -> ep_id -> ep_lim -> ep_env -> ep_cal [style=invis];
    sr_cmd -> sr_stat -> sr_envt -> sr_cur     [style=invis];
}
```

### EPROM Memory Tables

#### Read-Only Registers

| Address | Name                       | Size (bytes) | Range | Description                   |
| ------- | -------------------------- | ------------ | ----- | ----------------------------- |
| 0       | Firmware Main Version      | 1            | -     | Read-only firmware version    |
| 1       | Firmware Secondary Version | 1            | -     | Read-only firmware subversion |
| 3       | Servo Main Version         | 1            | -     | Hardware main version         |
| 4       | Servo Sub Version          | 1            | -     | Hardware subversion           |

#### Read/Write Registers

| Address | Name                  | Size (bytes) | Range      | Default | Units/Notes        |
| ------- | --------------------- | ------------ | ---------- | ------- | ------------------ |
| 5       | ID                    | 1            | 0-253      | 1       | -                  |
| 6       | Baud Rate             | 1            | 0-7        | 4       | Index value        |
| 7       | Return Delay Time     | 1            | 0-254      | 250     | 2μs per unit       |
| 8       | Status Return Level   | 1            | 0-1        | 1       | -                  |
| 9-10    | Min Position Limit    | 2            | 0-1023     | 0       | Encoder steps      |
| 11-12   | Max Position Limit    | 2            | 0-1023     | 1023    | Encoder steps      |
| 13      | Max Temperature Limit | 1            | 0-100      | 80      | °C                 |
| 14      | Max Input Voltage     | 1            | 0-254      | 140     | 0.1V units (14.0V) |
| 15      | Min Input Voltage     | 1            | 0-254      | 80      | 0.1V units (8.0V)  |
| 16-17   | Max Torque Limit      | 2            | 0-1000     | 1000    | 0.1% of max torque |
| 26      | CW Dead Band          | 1            | 0-32       | 2       | Encoder steps      |
| 27      | CCW Dead Band         | 1            | 0-32       | 2       | Encoder steps      |
| 31-32   | Position Offset       | 2            | -2047-2047 | 0       | Encoder steps      |
| 33      | Operating Mode        | 1            | 0-3        | 0       | Mode index         |

### SRAM Memory Tables

#### Read/Write Registers

| Address | Name          | Size (bytes) | Range        | Default | Units/Notes          |
| ------- | ------------- | ------------ | ------------ | ------- | -------------------- |
| 40      | Torque Enable | 1            | 0-2          | 0       | 0=off, 1=on, 2=hold  |
| 41      | Acceleration  | 1            | 0-254        | 0       | Steps/s² (approx)    |
| 42-43   | Goal Position | 2            | 0-1023       | -       | Encoder steps        |
| 44-45   | Running Time  | 2            | -32766-32766 | 0       | ms                   |
| 46-47   | Goal Speed    | 2            | -1000-1000   | 0       | RPM (approx)         |
| 48-49   | Torque Limit  | 2            | 0-1000       | 1000    | 0.1% of max torque   |
| 55      | Lock          | 1            | 0-1          | 1       | 0=unlocked, 1=locked |

#### Read-Only Status Registers

| Address | Name                | Size (bytes) | Range        | Description                                |
| ------- | ------------------- | ------------ | ------------ | ------------------------------------------ |
| 56-57   | Present Position    | 2            | 0-1023       | Current servo position (encoder steps)     |
| 58-59   | Present Speed       | 2            | -32768-32767 | Current speed (RPM, approx)                |
| 60-61   | Present Load        | 2            | -1000-1000   | Current load (0.1% of max torque)          |
| 62      | Present Voltage     | 1            | -            | Current voltage (0.1V units)               |
| 63      | Present Temperature | 1            | -            | Current temperature (°C)                   |
| 66      | Moving Status       | 1            | 0-1          | Movement status flag (0=stopped, 1=moving) |
| 69-70   | Present Current     | 2            | -            | Current draw (mA)                          |

## Communication Protocol

### Packet Structure

Every packet — host-to-servo and servo-to-host — uses the same byte layout.
A read of the current position therefore looks like this on the wire:

```{graphviz}
:caption: Packet anatomy (example: READ position from ID 3)
:align: center

digraph packet {
    graph [rankdir=LR, bgcolor="transparent", fontname="Roboto",
           nodesep=0.05, ranksep=0.4];
    node  [shape=record, fontname="Roboto", fontsize=10,
           style="filled", fillcolor="#1a237e", fontcolor="white",
           penwidth=1.0, color="#5e35b1"];

    pkt [label=<{
        <font color="#d6c8ff">0xFF</font>|<font color="#d6c8ff">0xFF</font>|<b>ID</b>|<b>Len</b>|<b>Inst</b>|<b>Params&hellip;</b>|<b>Sum</b>
    }>];

    annot [shape=plaintext, fillcolor="transparent", fontcolor="#5e35b1", label=<
        <table border="0" cellspacing="0" cellpadding="3">
        <tr>
            <td align="center" width="55"><font point-size="9">header</font></td>
            <td align="center" width="55"><font point-size="9">header</font></td>
            <td align="center" width="42"><font point-size="9">target<br/>id</font></td>
            <td align="center" width="42"><font point-size="9">N&nbsp;params<br/>+&nbsp;2</font></td>
            <td align="center" width="44"><font point-size="9">opcode</font></td>
            <td align="center" width="80"><font point-size="9">addr / length<br/>or payload</font></td>
            <td align="center" width="60"><font point-size="9">~&Sigma; bytes</font></td>
        </tr></table>
    >];

    pkt -> annot [style=invis];
}
```

### Instructions

```{graphviz}
:caption: Instruction set — opcodes, direction and intent
:align: center

digraph instr_set {
    graph [rankdir=LR, bgcolor="transparent", fontname="Roboto", nodesep=0.4, ranksep=0.4];
    node  [shape=box, style="filled,rounded", fontname="Roboto",
           fontsize=10, margin="0.18,0.10", penwidth=1.1];
    edge  [color="#7a6cad", fontname="Roboto", fontsize=9];

    host  [label="Host", fillcolor="#1a237e", fontcolor="white"];
    one   [label="Single servo", fillcolor="#ec407a", fontcolor="white"];
    many  [label="Many servos", fillcolor="#ad1457", fontcolor="white"];

    host -> one  [label="PING (0x01)\nexists?"];
    one  -> host [label="ACK", style=dashed];

    host -> one  [label="READ (0x02)\naddr, len"];
    one  -> host [label="bytes", style=dashed];

    host -> one  [label="WRITE (0x03)\ncommit now"];
    host -> one  [label="REG_WRITE (0x04)\nstage"];
    host -> many [label="ACTION (0x05)\ncommit all staged", color="#ec407a"];
    host -> many [label="SYNC_WRITE (0x83)\nfan-out", color="#ec407a", penwidth=2.0];
}
```

`SYNC_WRITE` is the path the diff-drive controller actually uses every tick — one packet on the wire sets goal speeds for all four wheels.

## Operation Modes

### Position Control

- 16-bit position value (0-1023)
- Supports negative positions using bit 15
- Can specify speed and acceleration
- Position limits can be set in EPROM

### Speed Control (Wheel Mode)

- 16-bit speed value
- Sign bit (bit 15) determines direction
- Acceleration parameter for smooth transitions

### Open Loop Control

- Direct PWM control
- Used for manual torque/speed control
- No position feedback

## Status Monitoring

- Real-time feedback of:
  - Position (0-1023)
  - Speed (-32768 to 32767)
  - Load (0-1000, percentage of max torque)
  - Voltage (0.1V resolution)
  - Temperature (°C)
  - Moving status
  - Current draw

## Error Handling

- Hardware status reporting
- Protection features:
  - Temperature limit
  - Voltage limits
  - Current limit
  - Load protection
  - Dead band protection

## Implementation Notes

- Default baud rate setting: 4
- Status return level configurable
- EPROM can be locked to prevent accidental changes
- Temperature limit default: 80°C
- Voltage operating range: 8-14V

## Protocol Examples

The two flows below trace single transactions byte-by-byte.
The first reads `Present Position` from one servo; the second writes a new goal speed to another.

```{graphviz}
:caption: Two complete request / reply transactions on the bus
:align: center

digraph proto_examples {
    graph [rankdir=LR, bgcolor="transparent", fontname="Roboto", ranksep=0.5];
    node  [fontname="Roboto", fontsize=10, style="filled,rounded",
           shape=box, penwidth=1.1];
    edge  [fontname="Roboto", fontsize=9, color="#7a6cad"];

    subgraph cluster_a {
        label=<<b>READ position</b>  <font point-size="9">addr 0x38, 2 bytes</font>>;
        labeljust="l"; style="rounded,filled"; color="#3949ab";
        fillcolor="#1a237e"; fontcolor="#d6c8ff";

        host_a  [label="Host", fillcolor="#311b92", fontcolor="white"];
        servo_a [label="Servo ID 3", fillcolor="#ec407a", fontcolor="white"];

        host_a  -> servo_a [label="FF FF 03 04 02 38 02 B8\n(8 bytes)", color="#ec407a"];
        servo_a -> host_a  [label="FF FF 03 04 00 E8 03 0E\n0x03E8 = 1000 steps", color="#00bcd4", style=dashed];
    }

    subgraph cluster_b {
        label=<<b>WRITE goal speed</b>  <font point-size="9">addr 0x2E, 2 bytes, value 50</font>>;
        labeljust="l"; style="rounded,filled"; color="#6a1b9a";
        fillcolor="#4a148c"; fontcolor="#ffd6ea";

        host_b  [label="Host", fillcolor="#311b92", fontcolor="white"];
        servo_b [label="Servo ID 4", fillcolor="#ec407a", fontcolor="white"];

        host_b  -> servo_b [label="FF FF 04 05 03 2E 32 00 8E\n(9 bytes)", color="#ec407a"];
        servo_b -> host_b  [label="FF FF 04 02 00 F9\nerror = 0 (OK)", color="#00bcd4", style=dashed];
    }
}
```

### Reading Position (ID #3)

**TX Packet:**

| Byte | Field       | Hex Value | Description                         |
| ---- | ----------- | --------- | ----------------------------------- |
| 0-1  | Header      | 0xFF 0xFF | Start of packet                     |
| 2    | ID          | 0x03      | Servo #3                            |
| 3    | Length      | 0x04      | 4 bytes following                   |
| 4    | Instruction | 0x02      | READ command                        |
| 5    | Address     | 0x38      | Position register (address 56)      |
| 6    | Size        | 0x01      | Read 2 bytes                        |
| 7    | Checksum    | 0xB8      | ~(0x03 + 0x04 + 0x02 + 0x38 + 0x01) |

**RX Packet:**

| Byte | Field      | Hex Value | Description           |
| ---- | ---------- | --------- | --------------------- |
| 0-1  | Header     | 0xFF 0xFF | Start of packet       |
| 2    | ID         | 0x03      | Servo #3              |
| 3    | Length     | 0x04      | 4 bytes following     |
| 4    | Error      | 0x00      | No error              |
| 5    | Position L | 0xE8      | Low byte of position  |
| 6    | Position H | 0x03      | High byte of position |
| 7    | Checksum   | 0x0E      | Calculated checksum   |

Response shows position 0x03E8 (1000 in decimal).

### Writing Speed (ID #4)

**TX Packet:**

| Byte | Field       | Hex Value | Description                                |
| ---- | ----------- | --------- | ------------------------------------------ |
| 0-1  | Header      | 0xFF 0xFF | Start of packet                            |
| 2    | ID          | 0x04      | Servo #4                                   |
| 3    | Length      | 0x05      | 5 bytes following                          |
| 4    | Instruction | 0x03      | WRITE command                              |
| 5    | Address     | 0x2E      | Speed register (address 46)                |
| 6    | Speed L     | 0x32      | Low byte of speed (50 decimal)             |
| 7    | Speed H     | 0x00      | High byte of speed                         |
| 8    | Checksum    | 0x8E      | ~(0x04 + 0x05 + 0x03 + 0x2E + 0x32 + 0x00) |

**RX Packet:**

| Byte | Field    | Hex Value | Description         |
| ---- | -------- | --------- | ------------------- |
| 0-1  | Header   | 0xFF 0xFF | Start of packet     |
| 2    | ID       | 0x04      | Servo #4            |
| 3    | Length   | 0x02      | 2 bytes following   |
| 4    | Error    | 0x00      | No error - success  |
| 5    | Checksum | 0xF9      | Calculated checksum |

Success response (Error = 0x00).

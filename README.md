# vd100-bare-metal-ma-aie-interactive

Interactive bare-metal PS application for the VD100 board (Versal AI Edge XCVE2302).
UART command interface to trigger the AIE-ML v1 MA crossover pipeline on demand.

Combines the UART echo pattern from `vd100-bare-metal-uart-echo` with the AIE pipeline
from `vd100-bare-metal-ma-aie-app`. The AIE graph runs on 'S' command and results are
printed immediately — no reboot required between runs.

---

## What It Does

```
PC terminal → type 'S' → Enter
    → PS A72 receives command
    → configures S2MM + MM2S
    → runs AIE graph (4 iterations)
    → prints MA crossover results
    → returns to command prompt
```

Terminal session:
```
========================================
  VD100 Bare-Metal MA Crossover Client
  XCVE2302 AIE-ML v1 | mygraph
  Available commands: S=Start
  Version 1.0 2026
========================================

[Command] >> S
[VD100] >> [DMA]  Configuring S2MM (sink) at 0x001C2B30
[VD100] >> [DMA]  Configuring MM2S (source) at 0x001C27A0
[VD100] >> [AIE]  Running graph for 4 iteration(s)
[VD100] >> [WAIT] S2MM done
[VD100] >> [WAIT] MM2S done
[VD100] >> [AIE]  Graph ended

[VD100] >> +---------+----------+----------+--------+
[VD100] >> |  Block  | fast_ma  | slow_ma  | Signal |
[VD100] >> +---------+----------+----------+--------+
|      1  |    5000  |    5000  |  HOLD  |
|      2  |    4990  |    4990  |  HOLD  |
|      3  |    5051  |    5002  |  BUY   |
|      4  |    5600  |    5600  |  HOLD  |
+---------+----------+----------+--------+
[VD100] >> [PASS] All 4 blocks match golden output
[VD100] >> [DONE] 4 block(s) processed via AIE-ML v1 (XCVE2302)

[Command] >>
```

Press 'S' again to rerun — `gr.run()` after `gr.end()` works correctly on bare-metal.

---

## Commands

| Key | Action |
|-----|--------|
| `S` | Run MA crossover graph with golden test vector, print results |
| `H` | Show help |
| other | Unknown command message |

---

## Hardware

| Item | Value |
|------|-------|
| Board | VD100 |
| Device | XCVE2302-SFVA784-1LP-E-S |
| AIE tile | col=8, row=0 |
| PL clock | 100 MHz |
| Vitis | 2025.2 |
| UART | PS UART0 — 115200 baud via CP2102 |

---

## Repository Structure

```
vd100-bare-metal-ma-aie-interactive/
├── src/
│   ├── main.cpp                       # Interactive UART command handler + AIE pipeline
│   └── baremetal_metadata_compile.cpp # Copied from vd100-aie-ma-crossover
└── README.md
```

---

## Build

Same as `vd100-bare-metal-ma-aie-app` — swap the app component ELF in `package.cfg`.

```
vd100-bare-metal-system-project / package.cfg:
    Baremetal Elf: vd100-bare-metal-ma-aie-interactive.elf,a72-0
```

---

## Critical Notes

### Startup initialisation order is mandatory

```cpp
int main(void) {
    Xil_DCacheDisable();          // 1. FIRST — before any xil_printf
    xil_printf("...\r\n");        // 2. UART now safe to use
    int32_t* in_buf  = malloc();  // 3. Allocate once
    int32_t* out_buf = malloc();  // 4. Allocate once
    fill_golden_input(in_buf);    // 5. Fill once
    BaremetalGraph gr("mygraph"); // 6. Construct once — visible errors now
    // 7. Enter command loop
}
```

Violations of this order cause silent hangs:
- `Xil_DCacheDisable()` after `xil_printf()` → UART FIFO corruption → system lock
- `BaremetalGraph` as global → constructor errors invisible before UART init
- `malloc()` inside command handler → heap fragmentation on repeated runs

### gr.run() is repeatable after gr.end()

`BaremetalGraph::run(N)` followed by `gr.end()` can be called again without
constructing a new graph instance. The 'S' command can be pressed multiple times
in the same session.

### Command dispatch on Enter, not on keypress

Commands are buffered until Enter is pressed. Single-character commands like 'S'
still require Enter to dispatch. This matches standard terminal behaviour and
allows multi-character commands to be added in future.

### SIZE_OFFSET: word count not bytes

See `vd100-bare-metal-ma-aie-app` README for full explanation.

---

## Versal Bare-Metal Series

| Repo | Description |
|------|-------------|
| `vd100-bare-metal-uart-echo` | PS UART echo foundation |
| `vd100-bare-metal-ma-aie-app` | Auto-run AIE MA crossover |
| `vd100-bare-metal-ma-aie-interactive` | **This repo** — UART-commanded AIE MA crossover |

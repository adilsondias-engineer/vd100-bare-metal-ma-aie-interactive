/**
 * vd100-bare-metal-ma-aie-app — Bare-Metal PS Application
 * VD100 (XCVE2302) | MA Crossover via AIE-ML v1
 *
 * Pipeline:
 *   PS DDR → mm2s (HLS) → AIE graph (mygraph) → s2mm (HLS) → PS DDR
 *   Results printed via PS UART0 (LPD_MIO16/17 → CP2102 → USB)
 *
 * AIE kernel (kernels.cc — ma_crossover):
 *   Input:  int32 price ticks — BLOCK_SIZE=56 samples per iteration
 *   Output: int32[3] per iteration — { fast_ma, slow_ma, signal }
 *           signal: 1=BUY  -1=SELL  0=HOLD
 *
 * HLS kernel register map (confirmed from vd100-ps-ma-client.cpp):
 *   CTRL_OFFSET = 0x00  — control/status (bit0=start, bit1=idle, bit2=done)
 *   MEM_OFFSET  = 0x10  — memory address low 32 bits
 *   MEM_OFFSET+4= 0x14  — memory address high 32 bits (always 0 for <4GB)
 *   SIZE_OFFSET = 0x1C  — transfer size in BYTES
 *
 * Golden test vector (must match golden.txt):
 *   Block 1: fast_ma=5000  slow_ma=5000  HOLD
 *   Block 2: fast_ma=4990  slow_ma=4990  HOLD
 *   Block 3: fast_ma=5051  slow_ma=5002  BUY   <- crossover
 *   Block 4: fast_ma=5600  slow_ma=5600  HOLD
 *
 * Bare-metal notes:
 *   - No OS, no XRT, no device tree — direct MMIO via Xil_In32/Xil_Out32
 *   - Cache disabled (Xil_DCacheDisable) — critical for DMA coherency
 *   - UART output via xil_printf → PS UART0 → CP2102 USB-UART → minicom
 *   - BaremetalGraph from aeg_baremetal_api.h wraps AIE ELF loading + graph
 * control
 *
 * Build: Vitis bare-metal BSP, component linked into system project
 * Terminal: minicom -D /dev/ttyUSB0 -b 115200
 */

#include "aeg_baremetal_api.h"
#include "xaiengine.h"
#include "xil_cache.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xuartpsv.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ── HLS kernel base addresses (from xparameters.h) ───────────────────────────
#define MM2S_BASE XPAR_XMM2S_0_BASEADDR
#define S2MM_BASE XPAR_XS2MM_0_BASEADDR

// ── HLS AXI-Lite register offsets ────────────────────────────────────────────
#define CTRL_OFFSET 0x00 // control: bit0=start, bit1=idle, bit2=done
#define MEM_OFFSET 0x10  // DDR address low
#define SIZE_OFFSET 0x1C // transfer size in bytes

// ── MA kernel constants (must match kernels.cc)
// ───────────────────────────────
#define BLOCK_SIZE                                                             \
  56 // int32 samples per AIE iteration (56×4=224B, multiple of 32)
#define OUTPUT_VALS 3 // fast_ma, slow_ma, signal per block
#define NUM_BLOCKS 4  // golden test vector: 4 blocks

#define INPUT_SAMPLES (NUM_BLOCKS * BLOCK_SIZE)         // 224 int32s
#define OUTPUT_SAMPLES (NUM_BLOCKS * OUTPUT_VALS)       // 12  int32s
#define INPUT_BYTES (INPUT_SAMPLES * sizeof(int32_t))   // 896 bytes
#define OUTPUT_BYTES (OUTPUT_SAMPLES * sizeof(int32_t)) // 48  bytes

// ── Signal strings
// ────────────────────────────────────────────────────────────
static const char *signal_str(int32_t s) {
  if (s == 1)
    return "BUY ";
  if (s == -1)
    return "SELL";
  return "HOLD";
}

// ── Golden test vector
// ──────────────────────────────────────────────────────── 4 blocks × 56
// samples — matches vd100-ps-ma-client.cpp make_golden_input()
static void fill_golden_input(int32_t *buf) {
  int idx = 0;
  // Block 1: 56 × 5000
  for (int i = 0; i < 56; i++)
    buf[idx++] = 5000;
  // Block 2: 56 × 4990
  for (int i = 0; i < 56; i++)
    buf[idx++] = 4990;
  // Block 3: 55 × 4990 + 1 × 5600 (spike triggers BUY crossover)
  for (int i = 0; i < 55; i++)
    buf[idx++] = 4990;
  buf[idx++] = 5600;
  // Block 4: 56 × 5600
  for (int i = 0; i < 56; i++)
    buf[idx++] = 5600;
}

// ── Configure and start HLS DMA kernel ───────────────────────────────────────
static void start_kernel(uint32_t base, uintptr_t addr, uint32_t size_bytes) {
  Xil_Out32(base + MEM_OFFSET, (uint32_t)(addr & 0xFFFFFFFF));
  Xil_Out32(base + MEM_OFFSET + 4, (uint32_t)(addr >> 32)); // high bits
  Xil_Out32(base + SIZE_OFFSET, size_bytes);
  Xil_Out32(base + CTRL_OFFSET, 1);
}

// ── Poll HLS kernel until done (bits 1 or 2 set in status) ───────────────────
static void wait_kernel_done(uint32_t base, const char *name) {
  int polls = 0;
  while (1) {
    uint32_t v = Xil_In32(base + CTRL_OFFSET);
    if (v & 6)
      break; // bit1=idle or bit2=done
    polls++;
    if (polls % 100 == 0)
      xil_printf("[WAIT] %s: still running (polls=%d status=0x%08X)\r\n", name,
                 polls, v);
    usleep(1000); // 1ms poll interval
  }
}

void run_ma_graph(BaremetalGraph &gr, int32_t *in_buf, int32_t *out_buf) {
  // int32_t *in_buf = (int32_t *)malloc(INPUT_BYTES);
  // if (!in_buf) {
  //    xil_printf("[VD100] >> [ERROR] Failed to allocate input buffer\r\n");
  //  }

  // 3. Allocate output buffer
  // xil_printf("[VD100] >> [INIT] Allocating output buffer (%d int32s = %d "
  //           "bytes)\r\n",
  //           OUTPUT_SAMPLES, (int)OUTPUT_BYTES);
  // int32_t *out_buf = (int32_t *)malloc(OUTPUT_BYTES);
  // if (!out_buf) {
  //   xil_printf("[VD100] >> [ERROR] Failed to allocate output buffer\r\n");
  //  free(in_buf);
  // }

  // 4. Fill golden test vector
  // xil_printf("[VD100] >> [INIT] Filling golden test vector (%d blocks x %d "
  //           "samples)\r\n",
  //           NUM_BLOCKS, BLOCK_SIZE);
  // fill_golden_input(in_buf);
  // memset(out_buf, 0, OUTPUT_BYTES);

  // 5. Configure S2MM first — must be ready to sink before mm2s fires
  //    Otherwise AIE output stream backs up and stalls the graph
  xil_printf("[VD100] >> [DMA]  Configuring S2MM (sink) at 0x%08lX\r\n",
             (uintptr_t)out_buf);
  start_kernel(S2MM_BASE, (uintptr_t)out_buf, OUTPUT_SAMPLES);

  // 6. Configure MM2S
  xil_printf("[VD100] >> [DMA]  Configuring MM2S (source) at 0x%08lX\r\n",
             (uintptr_t)in_buf);
  start_kernel(MM2S_BASE, (uintptr_t)in_buf, INPUT_SAMPLES);

  // 7. Run AIE graph for NUM_BLOCKS iterations
  xil_printf("[VD100] >> [AIE]  Running graph for %d iteration(s)\r\n",
             NUM_BLOCKS);
  gr.run(4);

  // 8. Wait for completion — S2MM done = all output collected
  xil_printf("[VD100] >> [WAIT] Waiting for S2MM done...\r\n");
  wait_kernel_done(S2MM_BASE, "s2mm");
  xil_printf("[VD100] >> [WAIT] S2MM done\r\n");

  xil_printf("[VD100] >> [WAIT] Waiting for MM2S done...\r\n");
  wait_kernel_done(MM2S_BASE, "mm2s");
  xil_printf("[VD100] >> [WAIT] MM2S done\r\n");

  // 9. End graph
  gr.end();
  xil_printf("[VD100] >> [AIE]  Graph ended\r\n\r\n");

  // 10. Print results
  xil_printf("[VD100] >> +---------+----------+----------+--------+\r\n");
  xil_printf("[VD100] >> |  Block  | fast_ma  | slow_ma  | Signal |\r\n");
  xil_printf("[VD100] >> +---------+----------+----------+--------+\r\n");

  int errors = 0;

  // Expected golden output
  static const int32_t golden_fast[NUM_BLOCKS] = {5000, 4990, 5051, 5600};
  static const int32_t golden_slow[NUM_BLOCKS] = {5000, 4990, 5002, 5600};
  static const int32_t golden_sig[NUM_BLOCKS] = {0, 0, 1, 0};

  for (int b = 0; b < NUM_BLOCKS; b++) {
    int32_t fast_ma = out_buf[b * OUTPUT_VALS + 0];
    int32_t slow_ma = out_buf[b * OUTPUT_VALS + 1];
    int32_t signal = out_buf[b * OUTPUT_VALS + 2];

    xil_printf("|  %5d  |  %6d  |  %6d  |  %s  |\r\n", b + 1, fast_ma, slow_ma,
               signal_str(signal));

    // Validate against golden
    if (fast_ma != golden_fast[b] || slow_ma != golden_slow[b] ||
        signal != golden_sig[b]) {
      errors++;
      xil_printf("[VD100] >> [MISMATCH] Block %d: got fast=%d slow=%d sig=%d  "
                 "expected fast=%d slow=%d sig=%d\r\n",
                 b + 1, fast_ma, slow_ma, signal, golden_fast[b],
                 golden_slow[b], golden_sig[b]);
    }
  }

  xil_printf("+---------+----------+----------+--------+\r\n");
  xil_printf("\r\n");

  if (errors == 0) {
    xil_printf("[VD100] >> [PASS] All %d blocks match golden output\r\n",
               NUM_BLOCKS);
    xil_printf("[VD100] >> [PASS] MA Crossover AIE-ML v1 bare-metal test "
               "PASSED\r\n");
  } else {
    xil_printf("[VD100] >> [FAIL] %d block(s) mismatched golden output\r\n",
               errors);
  }

  xil_printf("\r\n[VD100] >> [DONE] %d block(s) processed via AIE-ML v1 "
             "(XCVE2302)\r\n",
             NUM_BLOCKS);

  free(in_buf);
  free(out_buf);
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main(void) {

  // MUST be first — before UART, before anything
  // 1. Disable D-cache — mandatory for DMA coherency on bare-metal
  xil_printf("[VD100] >> [INIT] Disabling D-cache for DMA coherency\r\n");
  Xil_DCacheDisable();

  xil_printf("\r\n");
  xil_printf("========================================\r\n");
  xil_printf("  VD100 Bare-Metal MA Crossover Client  \r\n");
  xil_printf("  XCVE2302 AIE-ML v1 | mygraph          \r\n");
  xil_printf(" Available commands: S=Start            \r\n");
  xil_printf("  Version 1.0.1 26                      \r\n");
  xil_printf("========================================\r\n");
  xil_printf("\r\n");

  // 2. Allocate input buffer
  xil_printf("[VD100] >> [INIT] Allocating input buffer (%d int32s = %d "
             "bytes)\r\n",
             INPUT_SAMPLES, (int)INPUT_BYTES);

  // Allocate buffers once at startup too
  int32_t *in_buf = (int32_t *)malloc(INPUT_BYTES);
  int32_t *out_buf = (int32_t *)malloc(OUTPUT_BYTES);
  fill_golden_input(in_buf);
  memset(out_buf, 0, OUTPUT_BYTES);

  // BaremetalGraph also constructed once at startup
  BaremetalGraph gr("mygraph");
  xil_printf("[VD100] >> [AIE]  BaremetalGraph ready\r\n");

  xil_printf("[Command] >> ");
  std::string text;
  while (1) {
    while (!XUartPsv_IsReceiveData(XPAR_XUARTPSV_0_BASEADDR))
      ;

    u8 c = XUartPsv_RecvByte(XPAR_XUARTPSV_0_BASEADDR);

    if (c == '\r') {
      //xil_printf("[VD100] >> enter pressed text=%s\r\n ", text.c_str());
      if (!text.empty()) {
        xil_printf("\r\n");

        // Command dispatch on Enter
        if (text == "S" || text == "s") {
          //xil_printf("[VD100] >> Starting ...\r\n");
          run_ma_graph(gr, in_buf, out_buf);
        } else if (text == "H" || text == "h") {
          xil_printf("[VD100] >> Commands: S=Start MA graph\r\n");
        } else {
          xil_printf("[VD100] >> Unknown command '%s'. H for help.\r\n",
                     text.c_str());
        }

        text = "";
        xil_printf("[Command] >> ");
      }
      //xil_printf("[VD100] >> end text.empty() ...\r\n");

    } else if (c == 0x7F || c == 0x08) {
      // Backspace — remove last char and clear from terminal
      if (!text.empty()) {
        text.pop_back();
        xil_printf("\b \b"); // erase character on terminal
      }
    } else {
      text += (char)c;
      XUartPsv_SendByte(XPAR_XUARTPSV_0_BASEADDR, c); // local echo while typing
    }
  }
  return -1;
}
//}

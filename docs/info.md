<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

A 4x4 unsigned 8-bit matrix multiply accelerator controlled via SPI. A single MAC (multiply-accumulate) unit iterates through all 64 operations (i, j, k) to compute C = A * B in 64 clock cycles.

The SPI command byte format is: `{R/W[7], SEL[6:5], ROW[4:3], COL[2:1], 0}`

- **SEL=00**: Write matrix A element (1 data byte)
- **SEL=01**: Write matrix B element (1 data byte)
- **SEL=10**: Control/status. Write bit 0 to start. Read returns `{6'b0, done, busy}`.
- **SEL=11**: Read matrix C element (3 bytes out, MSB first, 20-bit result zero-padded to 24 bits)

The accumulator is 20 bits wide, sufficient for the maximum result of 4 x 255 x 255 = 260,100.

## How to test

Connect an SPI master (e.g. microcontroller or RP2040 on the TT demo board) to the SPI pins:

1. Load matrix A by sending write commands with SEL=00 for each element A[row][col].
2. Load matrix B by sending write commands with SEL=01 for each element B[row][col].
3. Start computation by writing 0x01 to the control register (SEL=10).
4. Poll the status register (read SEL=10) until the done bit (bit 1) is set.
5. Read results from C[row][col] using read commands with SEL=11 (3 bytes per element).

## External hardware

SPI master (directly from the RP2040 on the TT demo board, or any external microcontroller).

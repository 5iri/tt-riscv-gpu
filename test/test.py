# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# SPI command byte: {R/W[7], SEL[6:5], ROW[4:3], COL[2:1], 0}
# SEL: 00=A (uint8), 01=B (3-bit sign-magnitude), 10=ctrl/status, 11=C (13-bit signed)
# W3A8: weights in {-3,-2,-1,0,+1,+2,+3}, activations uint8

def make_cmd(rw, sel, row, col):
    return ((rw & 1) << 7) | ((sel & 3) << 5) | ((row & 3) << 3) | ((col & 3) << 1)


def encode_weight(value):
    """Encode weight {-3..+3} into 3-bit sign-magnitude (bit2=sign, bits[1:0]=magnitude)."""
    if value < -3 or value > 3:
        raise ValueError(f"Weight {value} out of range -3..+3")
    mag = abs(value)
    sign = 1 if value < 0 else 0
    return (sign << 2) | mag


def to_signed(value, width):
    """Interpret unsigned integer as signed two's-complement with 'width' bits."""
    sign_bit = 1 << (width - 1)
    return (value ^ sign_bit) - sign_bit


# Pin indices within ui_in
SCLK_BIT = 0
CS_N_BIT = 1
MOSI_BIT = 2


async def spi_transfer(dut, cmd_byte, write_bytes=None, read_count=0):
    """Perform an SPI transaction. Returns list of read bytes."""
    read_result = []

    # Assert CS_N low
    ui = dut.ui_in.value.integer
    ui &= ~(1 << CS_N_BIT)
    dut.ui_in.value = ui
    await ClockCycles(dut.clk, 4)

    # Send command byte
    await _spi_send_byte(dut, cmd_byte)

    # Send write data bytes
    if write_bytes:
        for b in write_bytes:
            await _spi_send_byte(dut, b)

    # Read data bytes
    for _ in range(read_count):
        rb = await _spi_read_byte(dut)
        read_result.append(rb)

    # Deassert CS_N high
    await ClockCycles(dut.clk, 2)
    ui = dut.ui_in.value.integer
    ui |= (1 << CS_N_BIT)
    dut.ui_in.value = ui
    await ClockCycles(dut.clk, 4)

    return read_result


async def _spi_send_byte(dut, byte_val):
    """Clock out 8 bits on MOSI, MSB first."""
    for i in range(7, -1, -1):
        bit = (byte_val >> i) & 1
        ui = dut.ui_in.value.integer
        # Set MOSI
        if bit:
            ui |= (1 << MOSI_BIT)
        else:
            ui &= ~(1 << MOSI_BIT)
        # SCLK low
        ui &= ~(1 << SCLK_BIT)
        dut.ui_in.value = ui
        await ClockCycles(dut.clk, 4)
        # SCLK high (rising edge — slave samples)
        ui |= (1 << SCLK_BIT)
        dut.ui_in.value = ui
        await ClockCycles(dut.clk, 4)

    # SCLK low after byte
    ui = dut.ui_in.value.integer
    ui &= ~(1 << SCLK_BIT)
    dut.ui_in.value = ui
    await ClockCycles(dut.clk, 2)


async def _spi_read_byte(dut):
    """Clock in 8 bits from MISO, MSB first."""
    byte_val = 0
    for i in range(7, -1, -1):
        ui = dut.ui_in.value.integer
        # SCLK low (falling edge — slave drives MISO)
        ui &= ~(1 << SCLK_BIT)
        dut.ui_in.value = ui
        await ClockCycles(dut.clk, 4)
        # SCLK high (rising edge — master samples MISO)
        ui |= (1 << SCLK_BIT)
        dut.ui_in.value = ui
        await ClockCycles(dut.clk, 4)
        # Sample MISO (uo_out[0])
        miso = (dut.uo_out.value.integer >> 0) & 1
        byte_val = (byte_val << 1) | miso

    # SCLK low after byte
    ui = dut.ui_in.value.integer
    ui &= ~(1 << SCLK_BIT)
    dut.ui_in.value = ui
    await ClockCycles(dut.clk, 2)

    return byte_val


async def spi_write_matrix(dut, sel, matrix):
    """Write a 4x4 matrix (sel=0 for uint8 A, sel=1 for 3-bit weight B)."""
    for r in range(4):
        for c in range(4):
            cmd = make_cmd(0, sel, r, c)
            value = matrix[r][c]
            if sel == 1:
                value = encode_weight(value)
            await spi_transfer(dut, cmd, write_bytes=[value & 0xFF])


async def spi_read_c(dut, row, col):
    """Read a 13-bit signed C element from 3 SPI bytes (sign-extended to 24 bits)."""
    cmd = make_cmd(1, 3, row, col)
    result = await spi_transfer(dut, cmd, read_count=3)
    raw = (result[0] << 16) | (result[1] << 8) | result[2]
    return to_signed(raw & 0xFFFFFF, 24)


async def spi_start(dut):
    """Trigger computation."""
    cmd = make_cmd(0, 2, 0, 0)
    await spi_transfer(dut, cmd, write_bytes=[0x01])


async def spi_read_status(dut):
    """Read status byte: {6'b0, done, busy}."""
    cmd = make_cmd(1, 2, 0, 0)
    result = await spi_transfer(dut, cmd, read_count=1)
    return result[0]


def matmul_4x4(a, b):
    """Reference 4x4 matrix multiply (uint8 A, W3 weights B in {-3..+3})."""
    c = [[0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            for k in range(4):
                c[i][j] += a[i][k] * b[k][j]
    return c


@cocotb.test()
async def test_identity(dut):
    """Multiply by identity matrix."""
    dut._log.info("Start identity test")

    clock = Clock(dut.clk, 20, unit="ns")  # 50 MHz
    cocotb.start_soon(clock.start())

    # Reset
    dut.ena.value = 1
    dut.ui_in.value = (1 << CS_N_BIT)  # CS_N high, others low
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 20)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    A = [[1,  2,  3,  4],
         [5,  6,  7,  8],
         [9,  10, 11, 12],
         [13, 14, 15, 16]]

    # Identity matrix (valid W3 weights: all 0 or +1)
    B = [[1, 0, 0, 0],
         [0, 1, 0, 0],
         [0, 0, 1, 0],
         [0, 0, 0, 1]]

    expected = matmul_4x4(A, B)

    dut._log.info("Loading matrices")
    await spi_write_matrix(dut, 0, A)
    await spi_write_matrix(dut, 1, B)

    dut._log.info("Starting computation")
    await spi_start(dut)

    # Wait for done
    for _ in range(200):
        await ClockCycles(dut.clk, 10)
        status = await spi_read_status(dut)
        if status & 0x02:  # done bit
            break
    else:
        assert False, "Timed out waiting for done"

    dut._log.info("Reading results")
    for i in range(4):
        for j in range(4):
            val = await spi_read_c(dut, i, j)
            dut._log.info(f"C[{i}][{j}] = {val} (expected {expected[i][j]})")
            assert val == expected[i][j], f"C[{i}][{j}]: got {val}, expected {expected[i][j]}"

    dut._log.info("Identity test PASSED")


@cocotb.test()
async def test_matmul(dut):
    """General matrix multiply with 3-bit weights {-3..+3}."""
    dut._log.info("Start matmul test")

    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ena.value = 1
    dut.ui_in.value = (1 << CS_N_BIT)
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 20)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    A = [[  2,  3,  1,  4],
         [  5,  1,  0,  2],
         [  3,  7,  2,  1],
         [  0,  6,  4,  3]]

    B = [[ 3,  0, -1,  2],
         [ 0,  2,  1, -3],
         [-2,  1,  0,  1],
         [ 1, -1,  3,  0]]

    expected = matmul_4x4(A, B)

    await spi_write_matrix(dut, 0, A)
    await spi_write_matrix(dut, 1, B)
    await spi_start(dut)

    for _ in range(200):
        await ClockCycles(dut.clk, 10)
        status = await spi_read_status(dut)
        if status & 0x02:
            break
    else:
        assert False, "Timed out waiting for done"

    for i in range(4):
        for j in range(4):
            val = await spi_read_c(dut, i, j)
            assert val == expected[i][j], f"C[{i}][{j}]: got {val}, expected {expected[i][j]}"

    dut._log.info("Matmul test PASSED")

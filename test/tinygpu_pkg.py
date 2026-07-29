
# SPDX-FileCopyrightText: © 2026 TinyGPU-RV32
# SPDX-License-Identifier: Apache-2.0
#
# Minimal RV32I instruction encoder used only by test_crv.py to build
# randomized programs. Deliberately NOT generated from or shared with
# rv32_core.sv -- it's a from-scratch encoding of the public RV32I spec, so
# it can't inherit a misunderstanding of the ISA from the RTL it's checking.
#
# The one thing it DOES need to match the RTL on is the debug register
# address map (DBG_REG_PASSFAIL), since that's a design-specific MMIO
# convention, not part of the ISA. See tinygpu_pkg.sv's own
# DBG_REG_PASSFAIL = DEBUG_BASE + 32'h14, DEBUG_BASE = 32'h9000_0000.
 
DEBUG_BASE = 0x90000000
DBG_REG_PASSFAIL = DEBUG_BASE + 0x14
 
OPCODE_OP = 0b0110011
OPCODE_OP_IMM = 0b0010011
OPCODE_LUI = 0b0110111
OPCODE_BRANCH = 0b1100011
OPCODE_STORE = 0b0100011
OPCODE_LOAD = 0b0000011
OPCODE_SYSTEM = 0b1110011
 
# (funct3, funct7) for each R-type ALU op, matching rv32_core.sv's decode
# table exactly (this one fact has to match the RTL -- it's the ISA
# encoding, not a behavioral assumption).
ALU_OPS = {
    "add": (0b000, 0b0000000),
    "sub": (0b000, 0b0100000),
    "sll": (0b001, 0b0000000),
    "slt": (0b010, 0b0000000),
    "sltu": (0b011, 0b0000000),
    "xor": (0b100, 0b0000000),
    "srl": (0b101, 0b0000000),
    "sra": (0b101, 0b0100000),
    "or": (0b110, 0b0000000),
    "and": (0b111, 0b0000000),
}
# Restricted to the ops golden_alu() in test_crv.py implements today (shift
# ops need extra shamt-encoding care and aren't covered yet -- see test_crv.py
# module docstring for what's intentionally out of scope).
ALU_OPS = {k: v for k, v in ALU_OPS.items() if k in
           ("add", "sub", "and", "or", "xor", "slt", "sltu")}
 
 
def _mask(v, bits):
    return v & ((1 << bits) - 1)
 
 
def encode_r_type(opcode, rd, funct3, rs1, rs2, funct7):
    return (
        (_mask(funct7, 7) << 25)
        | (_mask(rs2, 5) << 20)
        | (_mask(rs1, 5) << 15)
        | (_mask(funct3, 3) << 12)
        | (_mask(rd, 5) << 7)
        | _mask(opcode, 7)
    )
 
 
def encode_i_type(opcode, rd, funct3, rs1, imm12):
    return (
        (_mask(imm12, 12) << 20)
        | (_mask(rs1, 5) << 15)
        | (_mask(funct3, 3) << 12)
        | (_mask(rd, 5) << 7)
        | _mask(opcode, 7)
    )
 
 
def encode_s_type(opcode, funct3, rs1, rs2, imm12):
    imm12 &= 0xFFF
    imm_hi = (imm12 >> 5) & 0x7F
    imm_lo = imm12 & 0x1F
    return (
        (imm_hi << 25)
        | (_mask(rs2, 5) << 20)
        | (_mask(rs1, 5) << 15)
        | (_mask(funct3, 3) << 12)
        | (imm_lo << 7)
        | _mask(opcode, 7)
    )
 
 
def encode_b_type(opcode, funct3, rs1, rs2, imm13):
    """imm13 must be even (bit 0 is implicitly 0, branch targets are
    2-byte aligned minimum; RV32I aligns to 4 here since we never emit
    compressed instructions)."""
    assert imm13 % 2 == 0, "branch offset must be even"
    imm13 &= 0x1FFF
    bit12 = (imm13 >> 12) & 1
    bit11 = (imm13 >> 11) & 1
    bits10_5 = (imm13 >> 5) & 0x3F
    bits4_1 = (imm13 >> 1) & 0xF
    return (
        (bit12 << 31)
        | (bits10_5 << 25)
        | (_mask(rs2, 5) << 20)
        | (_mask(rs1, 5) << 15)
        | (_mask(funct3, 3) << 12)
        | (bits4_1 << 8)
        | (bit11 << 7)
        | _mask(opcode, 7)
    )
 
 
def encode_u_type(opcode, rd, imm20):
    return (_mask(imm20, 20) << 12) | (_mask(rd, 5) << 7) | _mask(opcode, 7)
 
 
def encode_alu_op(op, rd, rs1, rs2):
    funct3, funct7 = ALU_OPS[op]
    return encode_r_type(OPCODE_OP, rd, funct3, rs1, rs2, funct7)
 
 
def encode_addi(rd, rs1, imm12):
    return encode_i_type(OPCODE_OP_IMM, rd, 0b000, rs1, imm12)
 
 
def encode_lui(rd, imm20):
    return encode_u_type(OPCODE_LUI, rd, imm20)
 
 
def encode_beq(rs1, rs2, imm):
    return encode_b_type(OPCODE_BRANCH, 0b000, rs1, rs2, imm)
 
 
def encode_sw(rs1, rs2, imm):
    return encode_s_type(OPCODE_STORE, 0b010, rs1, rs2, imm)
 
 
def encode_lw(rd, rs1, imm):
    return encode_i_type(OPCODE_LOAD, rd, 0b010, rs1, imm)
 
 
def encode_ebreak():
    # SYSTEM opcode, imm[11:0] == 1 selects EBREAK (vs. 0 for ECALL).
    return encode_i_type(OPCODE_SYSTEM, 0, 0b000, 0, 1)
 
 
# Zicsr (2026-07-29): CSR address is always the I-type imm12 field,
# regardless of register or immediate form. For the *I forms, the rs1
# field holds a 5-bit zero-extended immediate rather than a register index.
INSTR_MRET = 0x30200073
 
 
def encode_csrrw(rd, csr_addr, rs1):
    return encode_i_type(OPCODE_SYSTEM, rd, 0b001, rs1, csr_addr)
 
 
def encode_csrrs(rd, csr_addr, rs1):
    return encode_i_type(OPCODE_SYSTEM, rd, 0b010, rs1, csr_addr)
 
 
def encode_csrrc(rd, csr_addr, rs1):
    return encode_i_type(OPCODE_SYSTEM, rd, 0b011, rs1, csr_addr)
 
 
def encode_csrrwi(rd, csr_addr, zimm):
    return encode_i_type(OPCODE_SYSTEM, rd, 0b101, zimm, csr_addr)
 
 
def encode_csrrsi(rd, csr_addr, zimm):
    return encode_i_type(OPCODE_SYSTEM, rd, 0b110, zimm, csr_addr)
 
 
def encode_csrrci(rd, csr_addr, zimm):
    return encode_i_type(OPCODE_SYSTEM, rd, 0b111, zimm, csr_addr)
 
 
def encode_mret():
    return INSTR_MRET
 
 
# CSR addresses, matching tinygpu_pkg.sv exactly.
CSR_MSTATUS = 0x300
CSR_MIE = 0x304
CSR_MTVEC = 0x305
CSR_MSCRATCH = 0x340
CSR_MEPC = 0x341
CSR_MCAUSE = 0x342
CSR_MTVAL = 0x343
 
 
def materialize_imm32(rd, value):
    """LUI + ADDI sequence to load an arbitrary 32-bit constant into rd.
    Same pattern test.py's MINI_PROGRAM already uses by hand for the debug
    register address -- generalized here so test_crv.py can materialize any
    randomized operand, not just one fixed address.
 
    ADDI sign-extends its 12-bit immediate, so the low 12 bits must be
    biased by +0x800 before computing the LUI high bits, then corrected
    back down -- standard RISC-V constant-materialization idiom.
    """
    value &= 0xFFFFFFFF
    hi = (value + 0x800) >> 12
    lo = value - (hi << 12)  # signed 12-bit, in [-2048, 2047] by construction
    hi &= 0xFFFFF  # keep to 20 bits (materialize_imm32 assumes it fits)
    words = [encode_lui(rd, hi)]
    if lo != 0:
        words.append(encode_addi(rd, rd, lo))
    return words

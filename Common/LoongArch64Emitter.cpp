// Copyright (c) 2025- PPSSPP Project.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, version 2.0 or later versions.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License 2.0 for more details.

// A copy of the GPL 2.0 should have been included with the program.
// If not, see http://www.gnu.org/licenses/

// Official git repository and contact information can be found at
// https://github.com/hrydgard/ppsspp and http://www.ppsspp.org/.


#include "ppsspp_config.h"
#include <algorithm>
#include <cstring>
#include "Common/BitScan.h"
#include "Common/CPUDetect.h"
#include "Common/LoongArch64Emitter.h"

namespace LoongArch64Gen {

enum class Opcode32 {
    // Note: invalid, just used for FixupBranch.
    ZERO = 0x0,

    ADD_W = 0x00100000,
    ADD_D = 0x00108000,
    SUB_W = 0x00110000,
    SUB_D = 0x00118000,

    ADDI_W = 0x02800000,
    ADDI_D = 0x02c00000,
    ADDU16I_D = 0x10000000,

    ALSL_W = 0x00040000,
    ALSL_D = 0X002c0000,
    ALSL_WU = 0x00060000,

    LU12I_W = 0x14000000,
    LU32I_D = 0x16000000,
    LU52I_D = 0x03000000,

    SLT = 0x00120000,
    SLTU = 0x00128000,

    SLTI = 0x02000000,
    SLTUI = 0x02400000,

    PCADDI = 0x18000000,
    PCADDU12I = 0x1c000000,
    PCADDU18I = 0x1e000000,
    PCALAU12I = 0x1a000000,

    AND = 0x00148000,
    OR = 0x00150000,
    NOR = 0x00140000,
    XOR = 0x00158000,
    ANDN = 0x00168000,
    ORN = 0x00160000,

    ANDI = 0x03400000,
    ORI = 0x03800000,
    XORI = 0x03c00000,

    MUL_W = 0x001c0000,
    MULH_W = 0x001c8000,
    MULH_WU = 0x001d0000,
    MUL_D = 0x001d8000,
    MULH_D = 0x001e0000,
    MULH_DU = 0x001e8000,

    MULW_D_W = 0x001f0000,
    MULW_D_WU = 0x001f8000,

    DIV_W = 0x00200000,
    MOD_W = 0x00208000,
    DIV_WU = 0x00210000,
    MOD_WU = 0x00218000,
    DIV_D = 0x00220000,
    MOD_D = 0x00228000,
    DIV_DU = 0x00230000,
    MOD_DU = 0x00238000,

    SLL_W = 0x00170000,
    SRL_W = 0x00178000,
    SRA_W = 0x00180000,
    ROTR_W = 0x001b0000,

    SLLI_W = 0x00408000,
    SRLI_W = 0x00448000,
    SRAI_W = 0x00488000,
    ROTRI_W = 0x004c8000,

    SLL_D = 0x00188000,
    SRL_D = 0x00190000,
    SRA_D = 0x00198000,
    ROTR_D = 0x001b8000,

    SLLI_D = 0x00410000,
    SRLI_D = 0x00450000,
    SRAI_D = 0x00490000,
    ROTRI_D = 0x004d0000,

    EXT_W_B = 0x00005c00,
    EXT_W_H = 0x00005800,

    CLO_W = 0x00001000,
    CLO_D = 0x00002000,
    CLZ_W = 0x00001400,
    CLZ_D = 0x00002400,
    CTO_W = 0x00001800,
    CTO_D = 0x00002800,
    CTZ_W = 0x00001c00,
    CTZ_D = 0x00002c00,

    BYTEPICK_W = 0x00080000,
    BYTEPICK_D = 0x000c0000,

    REVB_2H = 0x00003000,
    REVB_4H = 0x00003400,
    REVB_2W = 0x00003800,
    REVB_D = 0x00003c00,

    BITREV_4B = 0x00004800,
    BITREV_8B = 0x00004c00,

    BITREV_W = 0x00005000,
    BITREV_D = 0x00005400,

    BSTRINS_W = 0x00600000,
    BSTRINS_D = 0x00800000,

    BSTRPICK_W = 0x00608000,
    BSTRPICK_D = 0x00c00000,

    MASKEQZ = 0x00130000,
    MASKNEZ = 0x00138000,

    BEQ = 0x58000000,
    BNE = 0x5c000000,
    BLT = 0x60000000,
    BGE = 0x64000000,
    BLTU = 0x68000000,
    BGEU = 0x6c000000,

    BEQZ = 0x40000000,
    BNEZ = 0x44000000,

    B = 0x50000000,
    BL = 0x54000000,

    JIRL = 0x4c000000,

    LD_B = 0x28000000,
    LD_H = 0x28400000,
    LD_W = 0x28800000,
    LD_D = 0x28c00000,
    LD_BU = 0x2a000000,
    LD_HU = 0x2a400000,
    LD_WU = 0x2a800000,
    ST_B = 0x29000000,
    ST_H = 0x29400000,
    ST_W = 0x29800000,
    ST_D = 0x29c00000,

    LDX_B = 0x38000000,
    LDX_H = 0x38040000,
    LDX_W = 0x38080000,
    LDX_D = 0x380c0000,
    LDX_BU = 0x38200000,
    LDX_HU = 0x38240000,
    LDX_WU = 0x38280000,
    STX_B = 0x38100000,
    STX_H = 0x38140000,
    STX_W = 0x38180000,
    STX_D = 0x381c0000,

    LDPTR_W = 0x24000000,
    LDPTR_D = 0x26000000,
    STPTR_W = 0x25000000,
    STPTR_D = 0x27000000,

    PRELD = 0x2ac00000,
    PRELDX = 0x382c0000,

    LDGT_B = 0x38780000,
    LDGT_H = 0x38788000,
    LDGT_W = 0x38790000,
    LDGT_D = 0x38798000,
    LDLE_B = 0x387a0000,
    LDLE_H = 0x387a8000,
    LDLE_W = 0x387b0000,
    LDLE_D = 0x387b8000,
    STGT_B = 0x387c0000,
    STGT_H = 0x387c8000,
    STGT_W = 0x387d0000,
    STGT_D = 0x387d8000,
    STLE_B = 0x387e0000,
    STLE_H = 0x387e8000,
    STLE_W = 0x387f0000,
    STLE_D = 0x387f8000,

    AMSWAP_W = 0x38600000,
    AMSWAP_DB_W = 0x38690000,
    AMSWAP_D = 0x38608000,
    AMSWAP_DB_D = 0x38698000,

    AMADD_W = 0x38610000,
    AMADD_DB_W = 0x386a0000,
    AMADD_D = 0x38618000,
    AMADD_DB_D = 0x386a8000,

    AMAND_W = 0x38620000,
    AMAND_DB_W = 0x386b0000,
    AMAND_D = 0x38628000,
    AMAND_DB_D = 0x386b8000,

    AMOR_W = 0x38630000,
    AMOR_DB_W = 0x386c0000,
    AMOR_D = 0x38638000,
    AMOR_DB_D = 0x386c8000,

    AMXOR_W = 0x38640000,
    AMXOR_DB_W = 0x386d0000,
    AMXOR_D = 0x38648000,
    AMXOR_DB_D = 0x386d8000,

    AMMAX_W = 0x38650000,
    AMMAX_DB_W = 0x386e0000,
    AMMAX_D = 0x38658000,
    AMMAX_DB_D = 0x386e8000,

    AMMIN_W = 0x38660000,
    AMMIN_DB_W = 0x386f0000,
    AMMIN_D = 0x38668000,
    AMMIN_DB_D = 0x386f8000,

    AMMAX_WU = 0x38670000,
    AMMAX_DB_WU = 0x38700000,
    AMMAX_DU = 0x38678000,
    AMMAX_DB_DU = 0x38708000,

    AMMIN_WU = 0x38680000,
    AMMIN_DB_WU = 0x38710000,
    AMMIN_DU = 0x38688000,
    AMMIN_DB_DU = 0x38718000,

    AMSWAP_B = 0x385c0000,
    AMSWAP_DB_B = 0x385e0000,
    AMSWAP_H = 0x385c8000,
    AMSWAP_DB_H = 0x385e8000,

    AMADD_B = 0x385d0000,
    AMADD_DB_B = 0x385f0000,
    AMADD_H = 0x385d8000,
    AMADD_DB_H = 0x385f8000,

    AMCAS_B = 0x38580000,
    AMCAS_DB_B = 0x385a0000,
    AMCAS_H = 0x38588000,
    AMCAS_DB_H = 0x385a8000,
    AMCAS_W = 0x38590000,
    AMCAS_DB_W = 0x385b0000,
    AMCAS_D = 0x38598000,
    AMCAS_DB_D = 0x38698000,

    CRC_W_B_W = 0x00240000,
    CRC_W_H_W = 0x00248000,
    CRC_W_W_W = 0x00250000,
    CRC_W_D_W = 0x00258000,
    CRCC_W_B_W = 0x00260000,
    CRCC_W_H_W = 0x00268000,
    CRCC_W_W_W = 0x00270000,
    CRCC_W_D_W = 0x00278000,

    SYSCALL = 0x002b0000,
    BREAK = 0x002a0000,

    ASRTLE_D = 0x00010000,
    ASRTGT_D = 0x00018000,

    RDTIMEL_W = 0x00006000,
    RDTIMEH_W = 0x00006400,
    RDTIME_D = 0x00006800,

    CPUCFG = 0x00006c00,

    FADD_S = 0x01008000,
    FADD_D = 0x01010000,
    FSUB_S = 0x01028000,
    FSUB_D = 0x01030000,
    FMUL_S = 0x01048000,
    FMUL_D = 0x01050000,
    FDIV_S = 0x01068000,
    FDIV_D = 0x01070000,

    FMADD_S = 0x08100000,
    FMADD_D = 0x08200000,
    FMSUB_S = 0x08500000,
    FMSUB_D = 0x08600000,
    FNMADD_S= 0x08900000,
    FNMADD_D= 0x08a00000,
    FNMSUB_S= 0x08d00000,
    FNMSUB_D= 0x08e00000,

    FMAX_S = 0x01088000,
    FMAX_D = 0x01090000,
    FMIN_S = 0x010a8000,
    FMIN_D = 0x010b0000,

    FMAXA_S = 0x010c8000,
    FMAXA_D = 0x010d0000,
    FMINA_S = 0x010e8000,
    FMINA_D = 0x010f0000,

    FABS_S = 0x01140400,
    FABS_D = 0x01140800,
    FNEG_S = 0x01141400,
    FNEG_D = 0x01141800,

    FSQRT_S = 0x01144400,
    FSQRT_D = 0x01144800,
    FRECIP_S = 0x01145400,
    FRECIP_D = 0x01145800,
    FRSQRT_S = 0x01146400,
    FRSQRT_D = 0x01146800,

    FSCALEB_S = 0x01108000,
    FSCALEB_D = 0x01110000,
    FLOGB_S = 0x01142400,
    FLOGB_D = 0x01142800,
    FCOPYSIGN_S = 0x01128000,
    FCOPYSIGN_D = 0x01130000,

    FCLASS_S = 0x01143400,
    FCLASS_D = 0x01143800,
    FRECIPE_S = 0x01147400,
    FRECIPE_D = 0x01147800,
    FRSQRTE_S = 0x01148400,
    FRSQRTE_D = 0x01148800,

    FCMP_COND_S = 0x0c100000,
    FCMP_COND_D = 0x0c200000,

    FCVT_S_D = 0x01191800,
    FCVT_D_S = 0x01192400,

    FFINT_S_W = 0x011d1000,
    FFINT_S_L = 0x011d1800,
    FFINT_D_W = 0x011d2000,
    FFINT_D_L = 0x011d2800,
    FTINT_W_S = 0x011b0400,
    FTINT_W_D = 0x011b0800,
    FTINT_L_S = 0x011b2400,
    FTINT_L_D = 0x011b2800,

    FTINTRM_W_S = 0x011a0400,
    FTINTRM_W_D = 0x011a0800,
    FTINTRM_L_S = 0x011a2400,
    FTINTRM_L_D = 0x011a2800,
    FTINTRP_W_S = 0x011a4400,
    FTINTRP_W_D = 0x011a4800,
    FTINTRP_L_S = 0x011a6400,
    FTINTRP_L_D = 0x011a6800,
    FTINTRZ_W_S = 0x011a8400,
    FTINTRZ_W_D = 0x011a8800,
    FTINTRZ_L_S = 0x011aa400,
    FTINTRZ_L_D = 0x011aa800,
    FTINTRNE_W_S = 0x011ac400,
    FTINTRNE_W_D = 0x011ac800,
    FTINTRNE_L_S = 0x011ae400,
    FTINTRNE_L_D = 0x011ae800,

    FRINT_S = 0x011e4400,
    FRINT_D = 0x011e4800,

    FMOV_S = 0x01149400,
    FMOV_D = 0x01149800,

    FSEL = 0x0d000000,

    MOVGR2FR_W = 0x0114a400,
    MOVGR2FR_D = 0x00114a800,
    MOVGR2FRH_W = 0x0114ac00,

    MOVFR2GR_S = 0x0114b400,
    MOVFR2GR_D = 0x0114b800,
    MOVFRH2GR_S = 0x0114bc00,

    MOVGR2FCSR = 0x0114c000,
    MOVFCSR2GR = 0x0114c800,

    MOVFR2CF = 0x0114d000,
    MOVCF2FR = 0x0114d400,

    MOVGR2CF = 0x0114d800,
    MOVCF2GR = 0x0114dc00,

    BCEQZ = 0x48000000,
    BCNEZ = 0x48000100,

    FLD_S = 0x2b000000,
    FLD_D = 0x2b800000,
    FST_S = 0x2b400000,
    FST_D = 0x2bc00000,

    FLDX_S = 0x38300000,
    FLDX_D = 0x38340000,
    FSTX_S = 0x38380000,
    FSTX_D = 0x383c0000,

    FLDGT_S = 0x38740000,
    FLDGT_D = 0x38748000,
    FLDLE_S = 0x38750000,
    FLDLE_D = 0x38758000,
    FSTGT_S = 0x38760000,
    FSTGT_D = 0x38768000,
    FSTLE_S = 0x38770000,
    FSTLE_D = 0x38778000,
};

static inline s32 SignReduce32(s32 v, int width) {
	int shift = 32 - width;
	return (v << shift) >> shift;
}

static inline s64 SignReduce64(s64 v, int width) {
	int shift = 64 - width;
	return (v << shift) >> shift;
}

static inline bool SupportsCPUCFG() {
    return cpu_info.LOONGARCH_CPUCFG;
}

static inline bool SupportsLAM() {
    return cpu_info.LOONGARCH_LAM;
}

static inline bool SupportsUAL() {
    return cpu_info.LOONGARCH_UAL;
}

static inline bool SupportsFPU() {
    return cpu_info.LOONGARCH_FPU;
}

static inline bool SupportsLSX() {
    return cpu_info.LOONGARCH_LSX;
}

static inline bool SupportsLASX() {
    return cpu_info.LOONGARCH_LASX;
}

static inline bool SupportsCRC32() {
    return cpu_info.LOONGARCH_CRC32;
}

static inline bool SupportsComplex() {
    return cpu_info.LOONGARCH_COMPLEX;
}

static inline bool SupportsCrypto() {
    return cpu_info.LOONGARCH_CRYPTO;
}

static inline bool SupportsLVZ() {
    return cpu_info.LOONGARCH_LVZ;
}

static inline bool SupportsLBT_X86() {
    return cpu_info.LOONGARCH_LBT_X86;
}

static inline bool SupportsLBT_ARM() {
    return cpu_info.LOONGARCH_LBT_ARM;
}

static inline bool SupportsLBT_MIPS() {
    return cpu_info.LOONGARCH_LBT_MIPS;
}

static inline bool SupportsPTW() {
    return cpu_info.LOONGARCH_PTW;
}

static inline LoongArch64Reg DecodeReg(LoongArch64Reg reg) { return (LoongArch64Reg)(reg & 0x1F); }
static inline bool IsGPR(LoongArch64Reg reg) { return (reg & ~0x1F) == 0; }
static inline bool IsFPR(LoongArch64Reg reg) { return (reg & ~0x1F) == 0x20; }
static inline bool IsVPR(LoongArch64Reg reg) { return (reg & ~0x1F) == 0x40; }
static inline bool IsXPR(LoongArch64Reg reg) { return (reg & ~0x1F) == 0x60; }
static inline bool IsCFR(LoongArch64CFR cfr) { return (cfr < 8); }
static inline bool IsFCSR(LoongArch64FCSR fcsr) { return (fcsr < 4); }

LoongArch64Emitter::LoongArch64Emitter(const u8 *ptr, u8 *writePtr) {
    SetCodePointer(ptr, writePtr);
}

void LoongArch64Emitter::SetCodePointer(const u8 *ptr, u8 *writePtr) {
	code_ = ptr;
	writable_ = writePtr;
    lastCacheFlushEnd_ = ptr;
}

const u8 *LoongArch64Emitter::GetCodePointer() const {
	return code_;
}

u8 *LoongArch64Emitter::GetWritableCodePtr() {
	return writable_;
}

static inline u32 EncodeDJK(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(IsGPR(rd), "DJK instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJK instruction rj must be GPR");
    _assert_msg_(IsGPR(rk), "DJK instruction rk must be GPR");
    if (!IsGPR(rj)) {
        ERROR_LOG(Log::JIT, "opcode=%x, rd=%d, rj=%d, rk=%d", opcode, rd, rj, rk);
    }
    return (u32)opcode | ((u32)rk << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeDJSk12(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(IsGPR(rd), "DJSk12 instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJSk12 instruction rj must be GPR");
    return (u32)opcode | ((u32)(si12 & 0xFFF) << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeDJUk12(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj, u16 ui12) {
    _assert_msg_(IsGPR(rd), "DJUk12 instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJUk12 instruction rj must be GPR");
    return (u32)opcode | ((u32)(ui12 & 0xFFF) << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeDJSk16(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj, s16 si16) {
    _assert_msg_(IsGPR(rd), "DJSk16 instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJSk16 instruction rj must be GPR");
    return (u32)opcode | ((u32)si16 << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeDJKUa2pp1(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk, u8 sa2) {
    _assert_msg_(IsGPR(rd), "DJKUa2pp1 instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJKUa2pp1 instruction rj must be GPR");
    _assert_msg_(IsGPR(rk), "DJKUa2pp1 instruction rk must be GPR");
    // cpu will perform as sa2 + 1
    return (u32)opcode | (u32)((sa2 - 1) & 0x3) << 15 | ((u32)rk << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeDJKUa3pp1(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk, u8 sa3) {
    _assert_msg_(IsGPR(rd), "DJKUa3pp1 instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJKUa3pp1 instruction rj must be GPR");
    _assert_msg_(IsGPR(rk), "DJKUa3pp1 instruction rk must be GPR");
    // cpu will perform as sa3 + 1
    return (u32)opcode | (u32)((sa3 - 1) & 0x7) << 15 | ((u32)rk << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeDSj20(Opcode32 opcode, LoongArch64Reg rd, s32 si20) {
    _assert_msg_(IsGPR(rd), "DSj20 instruction rd must be GPR");
    return (u32)opcode | ((u32)(si20 & 0xFFFFF) << 5) | (u32)rd;
}

static inline u32 EncodeDJUk5(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj, u8 ui5) {
    _assert_msg_(IsGPR(rd), "DJUk5 instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJUk5 instruction rj must be GPR");
    return (u32)opcode | ((u32)(ui5 & 0x1F) << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeDJUk6(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj, u8 ui6) {
    _assert_msg_(IsGPR(rd), "DJUk6 instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJUk6 instruction rj must be GPR");
    return (u32)opcode | ((u32)(ui6 & 0x3F) << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeDJ(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(IsGPR(rd), "DJ instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJ instruction rj must be GPR");
    return (u32)opcode | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeJK(Opcode32 opcode, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(IsGPR(rj), "JK instruction rj must be GPR");
    _assert_msg_(IsGPR(rk), "JK instruction rk must be GPR");
    return (u32)opcode | ((u32)rk << 5) | ((u32)rj << 5);
}

static inline u32 EncodeDJKUa2(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk, u8 sa2) {
    _assert_msg_(IsGPR(rd), "DJKUa2 instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJKUa2 instruction rj must be GPR");
    _assert_msg_(IsGPR(rk), "DJKUa2 instruction rk must be GPR");
    return (u32)opcode | ((u32)(sa2 & 0x3) << 15) | ((u32)rk << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeDJUk5Um5(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj, u8 msbw, u8 lsbw) {
    _assert_msg_(IsGPR(rd), "DJUk5Um5 instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJUk5Um5 instruction rj must be GPR");
    return (u32)opcode | ((u32)(msbw & 0x1F) << 16) | ((u32)(lsbw & 0x1f) << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeDJUk6Um6(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj, u8 msbd, u8 lsbd) {
    _assert_msg_(IsGPR(rd), "DJUk6Um6 instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJUk6Um6 instruction rj must be GPR");
    return (u32)opcode | ((u32)(msbd & 0x3F) << 16) | ((u32)(lsbd & 0x3f) << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeDJKUa3(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk, u8 sa3) {
    _assert_msg_(IsGPR(rd), "DJKUa3 instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJKUa3 instruction rj must be GPR");
    _assert_msg_(IsGPR(rk), "DJKUa3 instruction rk must be GPR");
    return (u32)opcode | (u32)(sa3 & 0x7) << 15 | ((u32)rk << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeJDSk16ps2(Opcode32 opcode, LoongArch64Reg rj, LoongArch64Reg rd, s32 offs16) {
    _assert_msg_(IsGPR(rd), "JDSk16ps2 instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "JDSk16ps2 instruction rj must be GPR");
    _assert_msg_((offs16 & 3) == 0, "offs16 immediate must be aligned to 4");
    u32 offs = (u32)(offs16 >> 2);
    return (u32)opcode | ((offs & 0xFFFF) << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeDJSk16ps2(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj, s32 offs16) {
    _assert_msg_(IsGPR(rd), "DJSk16ps2 instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJSk16ps2 instruction rj must be GPR");
    _assert_msg_((offs16 & 3) == 0, "offs16 immediate must be aligned to 4");
    u32 offs = (u32)(offs16 >> 2) & 0xFFFF;
    return (u32)opcode | ((offs & 0xFFFF) << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeJSd5k16(Opcode32 opcode, LoongArch64Reg rj, s32 offs21) {
    _assert_msg_(IsGPR(rj), "JSd5k16 instruction rj must be GPR");
    _assert_msg_((offs21 & 3) == 0, "offs21 immediate must be aligned to 4");
    u32 offs = (u32)(offs21 >> 2);
    return (u32)opcode | ((offs & 0xFFFF) << 10) | ((u32)rj << 5) | ((offs >> 16) & 0x1F);
}

static inline u32 EncodeSd10k16ps2(Opcode32 opcode, s32 offs26) {
    _assert_msg_((offs26 & 3) == 0, "offs21 immediate must be aligned to 4");
    u32 offs = (u32)(offs26 >> 2);
    return (u32)opcode | ((offs & 0xFFFF) << 10) | ((offs >> 16) & 0x3FF);
}

static inline u32 EncodeDJSk14ps2(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rj, s16 si14) {
    _assert_msg_(IsGPR(rd), "DJSk14ps2 instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DJSk14ps2 instruction rj must be GPR");
    _assert_msg_((si14 & 3) == 0, "offs21 immediate must be aligned to 4");
    u32 si = (u32)(si14 >> 2);
    return (u32)opcode | (si & 0x3FFF) << 10 | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeUd5JSk12(Opcode32 opcode, u32 hint, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(IsGPR(rj), "Ud5JSk12 instruction rj must be GPR");
    _assert_msg_((si12 & 3) == 0, "offs21 immediate must be aligned to 4");
    return (u32)opcode | ((u32)(si12 & 0xFFF) << 10) | ((u32)rj << 5) | (u32)(hint & 0x1F);
}

static inline u32 EncodeUd5JK(Opcode32 opcode, u32 hint, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(IsGPR(rj), "Ud5JK instruction rj must be GPR");
    _assert_msg_(IsGPR(rk), "Ud5JK instruction rk must be GPR");
    return (u32)opcode | ((u32)rk << 10) | ((u32)rj << 5) | (u32)(hint & 0x1F);
}

static inline u32 EncodeDKJ(Opcode32 opcode, LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(IsGPR(rd), "DKJ instruction rd must be GPR");
    _assert_msg_(IsGPR(rj), "DKJ instruction rj must be GPR");
    _assert_msg_(IsGPR(rk), "DKJ instruction rk must be GPR");
    return (u32)opcode | ((u32)rk << 10) | ((u32)rj << 5) | (u32)rd;
}

static inline u32 EncodeUd15(Opcode32 opcode, u16 code) {
    return (u32)opcode | (u32)(code & 0x7FFF);
}

static inline u32 EncodeFdFjFk(Opcode32 opcode, LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    _assert_msg_(IsFPR(fd), "FdFjFk instruction fd must be FPR");
    _assert_msg_(IsFPR(fj), "FdFjFk instruction fj must be FPR");
    _assert_msg_(IsFPR(fk), "FdFjFk instruction fk must be FPR");
    return (u32)opcode | ((u32)DecodeReg(fk) << 10) | ((u32)DecodeReg(fj) << 5) | (u32)DecodeReg(fd);
}

static inline u32 EncodeFdFjFkFa(Opcode32 opcode, LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa) {
    _assert_msg_(IsFPR(fd), "FdFjFkFa instruction fd must be FPR");
    _assert_msg_(IsFPR(fj), "FdFjFkFa instruction fj must be FPR");
    _assert_msg_(IsFPR(fk), "FdFjFkFa instruction fk must be FPR");
    _assert_msg_(IsFPR(fa), "FdFjFkFa instruction fk must be FPR");
    return (u32)opcode | ((u32)DecodeReg(fa) << 15) | ((u32)DecodeReg(fk) << 10) | ((u32)DecodeReg(fj) << 5) | (u32)DecodeReg(fd);
}

static inline u32 EncodeFdFj(Opcode32 opcode, LoongArch64Reg fd, LoongArch64Reg fj) {
    _assert_msg_(IsFPR(fd), "FdFj instruction fd must be FPR");
    _assert_msg_(IsFPR(fj), "FdFj instruction fj must be FPR");
    return (u32)opcode | ((u32)DecodeReg(fj) << 5) | (u32)DecodeReg(fd);
}

static inline u32 EncodeCdFjFkFcond(Opcode32 opcode, LoongArch64CFR cd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Fcond cond) {
    _assert_msg_(IsCFR(cd), "CdFjFkFcond instruction cd must be CFR");
    _assert_msg_(IsFPR(fj), "CdFjFkFcond instruction fj must be FPR");
    _assert_msg_(IsFPR(fk), "CdFjFkFcond instruction fk must be FPR");
    return (u32)opcode | ((u32)cond << 15) | ((u32)DecodeReg(fk) << 10) | ((u32)DecodeReg(fj) << 5) | (u32)cd;
}

static inline u32 EncodeFdFjFkCa(Opcode32 opcode, LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64CFR ca) {
    _assert_msg_(IsFPR(fd), "FdFjFkCa instruction fd must be FPR");
    _assert_msg_(IsFPR(fj), "FdFjFkCa instruction fj must be FPR");
    _assert_msg_(IsFPR(fk), "FdFjFkCa instruction fk must be FPR");
    _assert_msg_(IsCFR(ca), "FdFjFkCa instruction ca must be CFR");
    return (u32)opcode | ((u32)ca << 15) | ((u32)DecodeReg(fk) << 10) | ((u32)DecodeReg(fj) << 5) | (u32)DecodeReg(fd);
}

static inline u32 EncodeFdJ(Opcode32 opcode, LoongArch64Reg fd, LoongArch64Reg rj) {
    _assert_msg_(IsFPR(fd), "FdJ instruction fd must be FPR");
    _assert_msg_(IsGPR(rj), "FdJ instruction rj must be GPR");
    return (u32)opcode | ((u32)rj << 5) | (u32)DecodeReg(fd);
}

static inline u32 EncodeDFj(Opcode32 opcode, LoongArch64Reg rj, LoongArch64Reg fd) {
    _assert_msg_(IsFPR(fd), "DFj instruction fd must be FPR");
    _assert_msg_(IsGPR(rj), "DFj instruction rj must be GPR");
    return (u32)opcode | ((u32)rj << 5) | (u32)DecodeReg(fd);
}

static inline u32 EncodeJUd5(Opcode32 opcode, LoongArch64FCSR fcsr, LoongArch64Reg rj) {
    _assert_msg_(IsFCSR(fcsr), "JUd5 instruction fcsr must be FCSR");
    _assert_msg_(IsGPR(rj), "JUd5 instruction rj must be GPR");
    return (u32)opcode | ((u32)rj << 5) | (u32)fcsr;
}

static inline u32 EncodeDUj5(Opcode32 opcode, LoongArch64Reg rd, LoongArch64FCSR fcsr) {
    _assert_msg_(IsGPR(rd), "DUj5 instruction rd must be GPR");
    _assert_msg_(IsFCSR(fcsr), "DUj5 instruction fcsr must be FCSR");
    return (u32)opcode | ((u32)fcsr << 5) | (u32)rd;
}

static inline u32 EncodeCdFj(Opcode32 opcode, LoongArch64CFR cd, LoongArch64Reg fj) {
    _assert_msg_(IsCFR(cd), "CdFj instruction cd must be CFR");
    _assert_msg_(IsFPR(fj), "CdFj instruction fj must be FPR");
    return (u32)opcode | ((u32)DecodeReg(fj) << 5) | (u32)cd;
}

static inline u32 EncodeFdCj(Opcode32 opcode, LoongArch64Reg fd, LoongArch64CFR cj) {
    _assert_msg_(IsFPR(fd), "FdCj instruction fd must be FPR");
    _assert_msg_(IsCFR(cj), "FdCj instruction cj must be CFR");
    return (u32)opcode | ((u32)cj << 5) | (u32)DecodeReg(fd);
}

static inline u32 EncodeCdJ(Opcode32 opcode, LoongArch64CFR cd, LoongArch64Reg rj) {
    _assert_msg_(IsCFR(cd), "CdJ instruction cd must be CFR");
    _assert_msg_(IsGPR(rj), "CdJ instruction rj must be GPR");
    return (u32)opcode | ((u32)rj << 5) | (u32)cd;
}

static inline u32 EncodeDCj(Opcode32 opcode, LoongArch64Reg rd, LoongArch64CFR cj) {
    _assert_msg_(IsGPR(rd), "DCj instruction rd must be GPR");
    _assert_msg_(IsCFR(cj), "DCj instruction cj must be CFR");
    return (u32)opcode | ((u32)cj << 5) | (u32)rd;
}

static inline u32 EncodeCjSd5k16ps2(Opcode32 opcode, LoongArch64CFR cj, s32 offs21) {
    _assert_msg_(IsCFR(cj), "CjSd5k16ps2 instruction cj must be CFR");
    _assert_msg_((offs21 & 3) == 0, "offs21 immediate must be aligned to 4");
    u32 offs = (u32)(offs21 >> 2);
    return (u32)opcode | ((offs & 0xFFFF) << 10) | ((u32)cj << 5) | ((offs >> 16) & 0x1F);
}

static inline u32 EncodeFdJSk12(Opcode32 opcode, LoongArch64Reg fd, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(IsFPR(fd), "FdJSk12 instruction fd must be FPR");
    _assert_msg_(IsGPR(rj), "FdJSk12 instruction rj must be GPR");
    return (u32)opcode | ((u32)(si12 & 0xFFF) << 10) | ((u32)rj << 5) | (u32)DecodeReg(fd);
}

static inline u32 EncodeFdJK(Opcode32 opcode, LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(IsFPR(fd), "FdJK instruction fd must be FPR");
    _assert_msg_(IsGPR(rj), "FdJK instruction rj must be GPR");
    _assert_msg_(IsGPR(rk), "FdJK instruction rk must be GPR");
    return (u32)opcode | ((u32)rk << 10) | ((u32)rj << 5) | (u32)DecodeReg(fd);
}

void LoongArch64Emitter::ReserveCodeSpace(u32 bytes)
{
	for (u32 i = 0; i < bytes / 4; i++)
		BREAK(0);
}

const u8 *LoongArch64Emitter::AlignCode16() {
	int c = int((u64)code_ & 15);
	if (c)
		ReserveCodeSpace(16 - c);
	return code_;
}

const u8 *LoongArch64Emitter::AlignCodePage() {
	int page_size = GetMemoryProtectPageSize();
	int c = int((intptr_t)code_ & ((intptr_t)page_size - 1));
	if (c)
		ReserveCodeSpace(page_size - c);
	return code_;
}

void LoongArch64Emitter::FlushIcache() {
	FlushIcacheSection(lastCacheFlushEnd_, code_);
	lastCacheFlushEnd_ = code_;
}

void LoongArch64Emitter::FlushIcacheSection(const u8 *start, const u8 *end) {
#if PPSSPP_ARCH(LOONGARCH64)
#if defined(__clang__)
	__clear_cache(start, end);
#else
	__builtin___clear_cache((char *)start, (char *)end);
#endif
#endif
}

FixupBranch::FixupBranch(FixupBranch &&other) {
	ptr = other.ptr;
	type = other.type;
	other.ptr = nullptr;
}

FixupBranch::~FixupBranch() {
	_assert_msg_(ptr == nullptr, "FixupBranch never set (left infinite loop)");
}

FixupBranch &FixupBranch::operator =(FixupBranch &&other) {
	ptr = other.ptr;
	type = other.type;
	other.ptr = nullptr;
	return *this;
}

void LoongArch64Emitter::SetJumpTarget(FixupBranch &branch) {
	SetJumpTarget(branch, code_);
}

void LoongArch64Emitter::SetJumpTarget(FixupBranch &branch, const void *dst) {
	_assert_msg_(branch.ptr != nullptr, "Invalid FixupBranch (SetJumpTarget twice?)");

	const intptr_t srcp = (intptr_t)branch.ptr;
	const intptr_t dstp = (intptr_t)dst;
	const ptrdiff_t writable_delta = writable_ - code_;
	u32 *writableSrc = (u32 *)(branch.ptr + writable_delta);

	u32 fixup;

	_assert_msg_((dstp & 3) == 0, "Destination should be aligned (no compressed)");

	ptrdiff_t distance = dstp - srcp;
	_assert_msg_((distance & 3) == 0, "Distance should be aligned (no compressed)");

	switch (branch.type) {
	case FixupBranchType::B:
		_assert_msg_(BranchInRange(branch.ptr, dst), "B destination is too far away (%p -> %p)", branch.ptr, dst);
		memcpy(&fixup, writableSrc, sizeof(u32));
		fixup = (fixup & 0xFC0003FF) | EncodeJDSk16ps2(Opcode32::ZERO, R_ZERO, R_ZERO, (s32)distance);
		memcpy(writableSrc, &fixup, sizeof(u32));
		break;

	case FixupBranchType::J:
		_assert_msg_(JumpInRange(branch.ptr, dst), "J destination is too far away (%p -> %p)", branch.ptr, dst);
		memcpy(&fixup, writableSrc, sizeof(u32));
		fixup = (fixup & 0xFC000000) | EncodeSd10k16ps2(Opcode32::ZERO, (s32)distance);
		memcpy(writableSrc, &fixup, sizeof(u32));
		break;

    case FixupBranchType::BZ:
		_assert_msg_(BranchInRange(branch.ptr, dst), "B destination is too far away (%p -> %p)", branch.ptr, dst);
		memcpy(&fixup, writableSrc, sizeof(u32));
		fixup = (fixup & 0xFC0003E0) | EncodeJSd5k16(Opcode32::ZERO, R_ZERO, (s32)distance);
		memcpy(writableSrc, &fixup, sizeof(u32));
		break;
	}

	branch.ptr = nullptr;
}

bool LoongArch64Emitter::BranchInRange(const void *func) const {
	return BranchInRange(code_, func);
}

bool LoongArch64Emitter::BranchZeroInRange(const void *func) const {
	return BranchZeroInRange(code_, func);
}

bool LoongArch64Emitter::JumpInRange(const void *func) const {
	return JumpInRange(code_, func);
}

static inline bool BJInRange(const void *src, const void *dst, int bits) {
	ptrdiff_t distance = (intptr_t)dst - (intptr_t)src;
	// Get rid of bits and sign extend to validate range.
	s32 encodable = SignReduce32((s32)distance, bits);
	return distance == encodable;
}

bool LoongArch64Emitter::BranchInRange(const void *src, const void *dst) const {
	return BJInRange(src, dst, 18);
}

bool LoongArch64Emitter::BranchZeroInRange(const void *src, const void *dst) const {
	return BJInRange(src, dst, 23);
}

bool LoongArch64Emitter::JumpInRange(const void *src, const void *dst) const {
	return BJInRange(src, dst, 28);
}

void LoongArch64Emitter::QuickJump(LoongArch64Reg scratchreg, LoongArch64Reg rd, const u8 *dst) {
	if (!JumpInRange(GetCodePointer(), dst)) {
		int32_t lower = (int32_t)SignReduce64((int64_t)dst, 18);
		static_assert(sizeof(intptr_t) <= sizeof(int64_t));
        LI(scratchreg, dst - lower);
		JIRL(rd, scratchreg, lower);
	} else if (rd != R_ZERO) {
		BL(dst);
	} else {
        B(dst);
    }
}

void LoongArch64Emitter::SetRegToImmediate(LoongArch64Reg rd, uint64_t value) {
	int64_t svalue = (int64_t)value;
	_assert_msg_(IsGPR(rd), "SetRegToImmediate only supports GPRs");

	if (SignReduce64(svalue, 12) == svalue) {
		// Nice and simple, small immediate fits in a single ADDI against zero.
		ADDI_D(rd, R_ZERO, (s32)svalue);
		return;
	}

	if (svalue <= 0x7fffffffl && svalue >= -0x80000000l) {
        // Use lu12i.w/ori to load 32-bits immediate.
		LU12I_W(rd, (s32)((svalue & 0xffffffff) >> 12));
		ORI(rd, rd, (s16)(svalue & 0xFFF));
        return;
	} else if (svalue <= 0x7ffffffffffffl && svalue >= -0x8000000000000l) {
        // Use lu12i.w/ori/lu32i.d to load 52-bits immediate.
		LU12I_W(rd, (s32)((svalue & 0xffffffff) >> 12));
		ORI(rd, rd, (s16)(svalue & 0xFFF));
		LU32I_D(rd, (s32)((svalue >> 32) & 0xfffff));
        return;
	}
    // Use lu12i.w/ori/lu32i.d/lu52i.d to load 64-bits immediate.
	LU12I_W(rd, (s32)((svalue & 0xffffffff) >> 12));
	ORI(rd, rd, (s16)(svalue & 0xFFF));
	LU32I_D(rd, (s32)((svalue >> 32) & 0xfffff));
	return LU52I_D(rd, rd, (s16)(svalue >> 52));
}

void LoongArch64Emitter::ADD_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
	_assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::ADD_W, rd, rj, rk));
}

void LoongArch64Emitter::ADD_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::ADD_D, rd, rj, rk));
}

void LoongArch64Emitter::SUB_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::SUB_W, rd, rj, rk));
}

void LoongArch64Emitter::SUB_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::SUB_D, rd, rj, rk));
}

void LoongArch64Emitter::ADDI_W(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(rd != R_ZERO || (rj == R0 && si12 == 0), "%s write to zero is a HINT", __func__); // work as NOP
    Write32(EncodeDJSk12(Opcode32::ADDI_W, rd, rj, si12));
}

void LoongArch64Emitter::ADDI_D(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(rd != R_ZERO || (rj == R0 && si12 == 0), "%s write to zero is a HINT", __func__); // work as NOP
    Write32(EncodeDJSk12(Opcode32::ADDI_D, rd, rj, si12));
}

void LoongArch64Emitter::ADDU16I_D(LoongArch64Reg rd, LoongArch64Reg rj, s16 si16) {
    _assert_msg_(rd != R_ZERO || (rj == R0 && si16 == 0), "%s write to zero is a HINT", __func__); // work as NOP
    Write32(EncodeDJSk16(Opcode32::ADDU16I_D, rd, rj, si16));
}

void LoongArch64Emitter::ALSL_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk, u8 sa2) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    _assert_msg_( sa2 >= 1 && sa2 <= 4, "%s shift out of range", __func__);
    Write32(EncodeDJKUa2pp1(Opcode32::ALSL_W, rd, rj, rk, sa2));
}

void LoongArch64Emitter::ALSL_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk, u8 sa2) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    _assert_msg_( sa2 >= 1 && sa2 <= 4, "%s shift out of range", __func__);
    Write32(EncodeDJKUa2pp1(Opcode32::ALSL_D, rd, rj, rk, sa2));
}

void LoongArch64Emitter::ALSL_WU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk, u8 sa2) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    _assert_msg_( sa2 >= 1 && sa2 <= 4, "%s shift out of range", __func__);
    Write32(EncodeDJKUa2pp1(Opcode32::ALSL_WU, rd, rj, rk, sa2));
}

void LoongArch64Emitter::LU12I_W(LoongArch64Reg rd, s32 si20) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDSj20(Opcode32::LU12I_W, rd, si20));
}

void LoongArch64Emitter::LU32I_D(LoongArch64Reg rd, s32 si20) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDSj20(Opcode32::LU32I_D, rd, si20));
}

void LoongArch64Emitter::LU52I_D(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJSk12(Opcode32::LU52I_D, rd, rj, si12));
}

void LoongArch64Emitter::SLT(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::SLT, rd, rj, rk));
}

void LoongArch64Emitter::SLTU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::SLTU, rd, rj, rk));
}

void LoongArch64Emitter::SLTI(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJSk12(Opcode32::SLTI, rd, rj, si12));
}

void LoongArch64Emitter::SLTUI(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJSk12(Opcode32::SLTUI, rd, rj, si12));
}

void LoongArch64Emitter::PCADDI(LoongArch64Reg rd, s32 si20) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDSj20(Opcode32::PCADDI, rd, si20));
}

void LoongArch64Emitter::PCADDU12I(LoongArch64Reg rd, s32 si20) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDSj20(Opcode32::PCADDU12I, rd, si20));
}

void LoongArch64Emitter::PCADDU18I(LoongArch64Reg rd, s32 si20) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDSj20(Opcode32::PCADDU18I, rd, si20));
}

void LoongArch64Emitter::PCALAU12I(LoongArch64Reg rd, s32 si20) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDSj20(Opcode32::PCALAU12I, rd, si20));
}

void LoongArch64Emitter::AND(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::AND, rd, rj, rk));
}

void LoongArch64Emitter::OR(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::OR, rd, rj, rk));
}

void LoongArch64Emitter::NOR(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::NOR, rd, rj, rk));
}

void LoongArch64Emitter::XOR(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::XOR, rd, rj, rk));
}

void LoongArch64Emitter::ANDN(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::ANDN, rd, rj, rk));
}

void LoongArch64Emitter::ORN(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::ORN, rd, rj, rk));
}

void LoongArch64Emitter::ANDI(LoongArch64Reg rd, LoongArch64Reg rj, u16 ui12) {
    _assert_msg_(rd != R_ZERO || (rj == R0 && ui12 == 0), "%s write to zero is a HINT", __func__); // work as NOP
    Write32(EncodeDJUk12(Opcode32::ANDI, rd, rj, ui12));
}

void LoongArch64Emitter::ORI(LoongArch64Reg rd, LoongArch64Reg rj, u16 ui12) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJUk12(Opcode32::ORI, rd, rj, ui12));
}

void LoongArch64Emitter::XORI(LoongArch64Reg rd, LoongArch64Reg rj, u16 ui12) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJUk12(Opcode32::XORI, rd, rj, ui12));
}

void LoongArch64Emitter::MUL_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::MUL_W, rd, rj, rk));
}

void LoongArch64Emitter::MULH_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::MULH_W, rd, rj, rk));
}

void LoongArch64Emitter::MULH_WU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::MULH_WU, rd, rj, rk));
}

void LoongArch64Emitter::MUL_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::MUL_D, rd, rj, rk));
}

void LoongArch64Emitter::MULH_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::MULH_D, rd, rj, rk));
}

void LoongArch64Emitter::MULH_DU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::MULH_DU, rd, rj, rk));
}

void LoongArch64Emitter::MULW_D_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::MULW_D_W, rd, rj, rk));
}

void LoongArch64Emitter::MULW_D_WU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::MULW_D_WU, rd, rj, rk));
}

void LoongArch64Emitter::DIV_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::DIV_W, rd, rj, rk));
}

void LoongArch64Emitter::MOD_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::MOD_W, rd, rj, rk));
}

void LoongArch64Emitter::DIV_WU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::DIV_WU, rd, rj, rk));
}

void LoongArch64Emitter::MOD_WU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::MOD_WU, rd, rj, rk));
}

void LoongArch64Emitter::DIV_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::DIV_D, rd, rj, rk));
}

void LoongArch64Emitter::MOD_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::MOD_D, rd, rj, rk));
}

void LoongArch64Emitter::DIV_DU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::DIV_DU, rd, rj, rk));
}

void LoongArch64Emitter::MOD_DU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::MOD_DU, rd, rj, rk));
}

void LoongArch64Emitter::SLL_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::SLL_W, rd, rj, rk));
}

void LoongArch64Emitter::SRL_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::SRL_W, rd, rj, rk));
}

void LoongArch64Emitter::SRA_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::SRA_W, rd, rj, rk));
}

void LoongArch64Emitter::ROTR_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::ROTR_W, rd, rj, rk));
}

void LoongArch64Emitter::SLLI_W(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui5) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJUk5(Opcode32::SLLI_W, rd, rj, ui5));
}

void LoongArch64Emitter::SRLI_W(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui5) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJUk5(Opcode32::SRLI_W, rd, rj, ui5));
}

void LoongArch64Emitter::SRAI_W(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui5) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJUk5(Opcode32::SRAI_W, rd, rj, ui5));
}

void LoongArch64Emitter::ROTRI_W(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui5) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJUk5(Opcode32::ROTRI_W, rd, rj, ui5));
}

void LoongArch64Emitter::SLL_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::SLL_D, rd, rj, rk));
}

void LoongArch64Emitter::SRL_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::SRL_D, rd, rj, rk));
}

void LoongArch64Emitter::SRA_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::SRA_D, rd, rj, rk));
}

void LoongArch64Emitter::ROTR_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::ROTR_D, rd, rj, rk));
}

void LoongArch64Emitter::SLLI_D(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui6) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJUk6(Opcode32::SLLI_D, rd, rj, ui6));
}

void LoongArch64Emitter::SRLI_D(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui6) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJUk6(Opcode32::SRLI_D, rd, rj, ui6));
}

void LoongArch64Emitter::SRAI_D(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui6) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJUk6(Opcode32::SRAI_D, rd, rj, ui6));
}

void LoongArch64Emitter::ROTRI_D(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui6) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJUk6(Opcode32::ROTRI_D, rd, rj, ui6));
}

void LoongArch64Emitter::EXT_W_B(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::EXT_W_B, rd, rj));
}

void LoongArch64Emitter::EXT_W_H(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::EXT_W_H, rd, rj));
}

void LoongArch64Emitter::CLO_W(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::CLO_W, rd, rj));
}

void LoongArch64Emitter::CLO_D(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::CLO_D, rd, rj));
}

void LoongArch64Emitter::CLZ_W(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::CLZ_W, rd, rj));
}

void LoongArch64Emitter::CLZ_D(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::CLZ_D, rd, rj));
}

void LoongArch64Emitter::CTO_W(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::CTO_W, rd, rj));
}

void LoongArch64Emitter::CTO_D(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::CTO_D, rd, rj));
}

void LoongArch64Emitter::CTZ_W(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::CTZ_W, rd, rj));
}

void LoongArch64Emitter::CTZ_D(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::CTZ_D, rd, rj));
}

void LoongArch64Emitter::BYTEPICK_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk, u8 sa2) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJKUa2(Opcode32::BYTEPICK_W, rd, rj, rk, sa2));
}

void LoongArch64Emitter::BYTEPICK_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk, u8 sa3) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJKUa2(Opcode32::BYTEPICK_D, rd, rj, rk, sa3));
}

void LoongArch64Emitter::REVB_2H(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::REVB_2H, rd, rj));
}

void LoongArch64Emitter::REVB_4H(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::REVB_4H, rd, rj));
}

void LoongArch64Emitter::REVB_2W(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::REVB_2W, rd, rj));
}

void LoongArch64Emitter::REVB_D(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::REVB_D, rd, rj));
}

void LoongArch64Emitter::BITREV_4B(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::BITREV_4B, rd, rj));
}

void LoongArch64Emitter::BITREV_8B(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::BITREV_8B, rd, rj));
}

void LoongArch64Emitter::BITREV_W(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::BITREV_W, rd, rj));
}

void LoongArch64Emitter::BITREV_D(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::BITREV_D, rd, rj));
}

void LoongArch64Emitter::BSTRINS_W(LoongArch64Reg rd, LoongArch64Reg rj, u8 msbw, u8 lsbw) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJUk5Um5(Opcode32::BSTRINS_W, rd, rj, msbw, lsbw));
}

void LoongArch64Emitter::BSTRINS_D(LoongArch64Reg rd, LoongArch64Reg rj, u8 msbd, u8 lsbd) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJUk6Um6(Opcode32::BSTRINS_D, rd, rj, msbd, lsbd));
}

void LoongArch64Emitter::BSTRPICK_W(LoongArch64Reg rd, LoongArch64Reg rj, u8 msbw, u8 lsbw) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJUk5Um5(Opcode32::BSTRPICK_W, rd, rj, msbw, lsbw));
}

void LoongArch64Emitter::BSTRPICK_D(LoongArch64Reg rd, LoongArch64Reg rj, u8 msbd, u8 lsbd) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJUk6Um6(Opcode32::BSTRPICK_D, rd, rj, msbd, lsbd));
}

void LoongArch64Emitter::MASKEQZ(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::MASKEQZ, rd, rj, rk));
}

void LoongArch64Emitter::MASKNEZ(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::MASKNEZ, rd, rj, rk));
}

void LoongArch64Emitter::BEQ(LoongArch64Reg rj, LoongArch64Reg rd, const void *dst) {
    _assert_msg_(BranchInRange(GetCodePointer(), dst), "%s destination is too far away (%p -> %p)", __func__, GetCodePointer(), dst);
    _assert_msg_(((intptr_t)dst & 3) == 0, "%s destination should be aligned to 4", __func__);
	ptrdiff_t distance = (intptr_t)dst - (intptr_t)GetCodePointer();
	Write32(EncodeJDSk16ps2(Opcode32::BEQ, rj, rd, (s32)distance));
}

void LoongArch64Emitter::BNE(LoongArch64Reg rj, LoongArch64Reg rd, const void *dst) {
    _assert_msg_(BranchInRange(GetCodePointer(), dst), "%s destination is too far away (%p -> %p)", __func__, GetCodePointer(), dst);
	_assert_msg_(((intptr_t)dst & 3) == 0, "%s destination should be aligned to 4", __func__);
    ptrdiff_t distance = (intptr_t)dst - (intptr_t)GetCodePointer();
	Write32(EncodeJDSk16ps2(Opcode32::BNE, rj, rd, (s32)distance));
}

void LoongArch64Emitter::BLT(LoongArch64Reg rj, LoongArch64Reg rd, const void *dst) {
    _assert_msg_(BranchInRange(GetCodePointer(), dst), "%s destination is too far away (%p -> %p)", __func__, GetCodePointer(), dst);
	_assert_msg_(((intptr_t)dst & 3) == 0, "%s destination should be aligned to 4", __func__);
    ptrdiff_t distance = (intptr_t)dst - (intptr_t)GetCodePointer();
	Write32(EncodeJDSk16ps2(Opcode32::BLT, rj, rd, (s32)distance));
}

void LoongArch64Emitter::BGE(LoongArch64Reg rj, LoongArch64Reg rd, const void *dst) {
    _assert_msg_(BranchInRange(GetCodePointer(), dst), "%s destination is too far away (%p -> %p)", __func__, GetCodePointer(), dst);
	_assert_msg_(((intptr_t)dst & 3) == 0, "%s destination should be aligned to 4", __func__);
    ptrdiff_t distance = (intptr_t)dst - (intptr_t)GetCodePointer();
	Write32(EncodeJDSk16ps2(Opcode32::BGE, rj, rd, (s32)distance));
}

void LoongArch64Emitter::BLTU(LoongArch64Reg rj, LoongArch64Reg rd, const void *dst) {
    _assert_msg_(BranchInRange(GetCodePointer(), dst), "%s destination is too far away (%p -> %p)", __func__, GetCodePointer(), dst);
	_assert_msg_(((intptr_t)dst & 3) == 0, "%s destination should be aligned to 4", __func__);
    ptrdiff_t distance = (intptr_t)dst - (intptr_t)GetCodePointer();
	Write32(EncodeJDSk16ps2(Opcode32::BLTU, rj, rd, (s32)distance));
}

void LoongArch64Emitter::BGEU(LoongArch64Reg rj, LoongArch64Reg rd, const void *dst) {
    _assert_msg_(BranchInRange(GetCodePointer(), dst), "%s destination is too far away (%p -> %p)", __func__, GetCodePointer(), dst);
	_assert_msg_(((intptr_t)dst & 3) == 0, "%s destination should be aligned to 4", __func__);
    ptrdiff_t distance = (intptr_t)dst - (intptr_t)GetCodePointer();
	Write32(EncodeJDSk16ps2(Opcode32::BGEU, rj, rd, (s32)distance));
}

FixupBranch LoongArch64Emitter::BEQ(LoongArch64Reg rj, LoongArch64Reg rd) {
	FixupBranch fixup{ GetCodePointer(), FixupBranchType::B };
	Write32(EncodeJDSk16ps2(Opcode32::BEQ, rj, rd, 0));
	return fixup;
}

FixupBranch LoongArch64Emitter::BNE(LoongArch64Reg rj, LoongArch64Reg rd) {
	FixupBranch fixup{ GetCodePointer(), FixupBranchType::B };
	Write32(EncodeJDSk16ps2(Opcode32::BNE, rj, rd, 0));
	return fixup;
}

FixupBranch LoongArch64Emitter::BLT(LoongArch64Reg rj, LoongArch64Reg rd) {
	FixupBranch fixup{ GetCodePointer(), FixupBranchType::B };
	Write32(EncodeJDSk16ps2(Opcode32::BLT, rj, rd, 0));
	return fixup;
}

FixupBranch LoongArch64Emitter::BGE(LoongArch64Reg rj, LoongArch64Reg rd) {
	FixupBranch fixup{ GetCodePointer(), FixupBranchType::B };
	Write32(EncodeJDSk16ps2(Opcode32::BGE, rj, rd, 0));
	return fixup;
}

FixupBranch LoongArch64Emitter::BLTU(LoongArch64Reg rj, LoongArch64Reg rd) {
	FixupBranch fixup{ GetCodePointer(), FixupBranchType::B };
	Write32(EncodeJDSk16ps2(Opcode32::BLTU, rj, rd, 0));
	return fixup;
}

FixupBranch LoongArch64Emitter::BGEU(LoongArch64Reg rj, LoongArch64Reg rd) {
	FixupBranch fixup{ GetCodePointer(), FixupBranchType::B };
	Write32(EncodeJDSk16ps2(Opcode32::BGEU, rj, rd, 0));
	return fixup;
}

void LoongArch64Emitter::BEQZ(LoongArch64Reg rj, const void *dst) {
    _assert_msg_(BranchZeroInRange(GetCodePointer(), dst), "%s destination is too far away (%p -> %p)", __func__, GetCodePointer(), dst);
	_assert_msg_(((intptr_t)dst & 3) == 0, "%s destination should be aligned to 4", __func__);
    ptrdiff_t distance = (intptr_t)dst - (intptr_t)GetCodePointer();
	Write32(EncodeJSd5k16(Opcode32::BEQZ, rj, (s32)distance));
}

void LoongArch64Emitter::BNEZ(LoongArch64Reg rj, const void *dst) {
    _assert_msg_(BranchZeroInRange(GetCodePointer(), dst), "%s destination is too far away (%p -> %p)", __func__, GetCodePointer(), dst);
	_assert_msg_(((intptr_t)dst & 3) == 0, "%s destination should be aligned to 4", __func__);
    ptrdiff_t distance = (intptr_t)dst - (intptr_t)GetCodePointer();
	Write32(EncodeJSd5k16(Opcode32::BNEZ, rj, (s32)distance));
}

FixupBranch LoongArch64Emitter::BEQZ(LoongArch64Reg rj) {
    FixupBranch fixup{ GetCodePointer(), FixupBranchType::BZ };
	Write32(EncodeJSd5k16(Opcode32::BEQZ, rj, 0));
	return fixup;
}

FixupBranch LoongArch64Emitter::BNEZ(LoongArch64Reg rj) {
    FixupBranch fixup{ GetCodePointer(), FixupBranchType::BZ };
	Write32(EncodeJSd5k16(Opcode32::BNEZ, rj, 0));
	return fixup;
}

void LoongArch64Emitter::B(const void *dst) {
    _assert_msg_(JumpInRange(GetCodePointer(), dst), "%s destination is too far away (%p -> %p)", __func__, GetCodePointer(), dst);
	_assert_msg_(((intptr_t)dst & 3) == 0, "%s destination should be aligned to 4", __func__);
    ptrdiff_t distance = (intptr_t)dst - (intptr_t)GetCodePointer();
    Write32(EncodeSd10k16ps2(Opcode32::B, (s32)distance));
}

void LoongArch64Emitter::BL(const void *dst) {
    _assert_msg_(JumpInRange(GetCodePointer(), dst), "%s destination is too far away (%p -> %p)", __func__, GetCodePointer(), dst);
	_assert_msg_(((intptr_t)dst & 3) == 0, "%s destination should be aligned to 4", __func__);
    ptrdiff_t distance = (intptr_t)dst - (intptr_t)GetCodePointer();
    Write32(EncodeSd10k16ps2(Opcode32::BL, (s32)distance));
}

FixupBranch LoongArch64Emitter::B() {
    FixupBranch fixup{ GetCodePointer(), FixupBranchType::J };
	Write32(EncodeSd10k16ps2(Opcode32::B, 0));
	return fixup;
}

FixupBranch LoongArch64Emitter::BL() {
    FixupBranch fixup{ GetCodePointer(), FixupBranchType::J };
	Write32(EncodeSd10k16ps2(Opcode32::BL, 0));
	return fixup;
}

void LoongArch64Emitter::JIRL(LoongArch64Reg rd, LoongArch64Reg rj, s32 offs16) {
    Write32(EncodeDJSk16ps2(Opcode32::JIRL, rd, rj, offs16));
}

void LoongArch64Emitter::LD_B(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJSk12(Opcode32::LD_B, rd, rj, si12));
}

void LoongArch64Emitter::LD_H(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJSk12(Opcode32::LD_H, rd, rj, si12));
}

void LoongArch64Emitter::LD_W(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJSk12(Opcode32::LD_W, rd, rj, si12));
}

void LoongArch64Emitter::LD_D(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJSk12(Opcode32::LD_D, rd, rj, si12));
}

void LoongArch64Emitter::LD_BU(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJSk12(Opcode32::LD_BU, rd, rj, si12));
}

void LoongArch64Emitter::LD_HU(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJSk12(Opcode32::LD_HU, rd, rj, si12));
}

void LoongArch64Emitter::LD_WU(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJSk12(Opcode32::LD_WU, rd, rj, si12));
}

void LoongArch64Emitter::ST_B(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    Write32(EncodeDJSk12(Opcode32::ST_B, rd, rj, si12));
}

void LoongArch64Emitter::ST_H(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    Write32(EncodeDJSk12(Opcode32::ST_H, rd, rj, si12));
}

void LoongArch64Emitter::ST_W(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    Write32(EncodeDJSk12(Opcode32::ST_W, rd, rj, si12));
}

void LoongArch64Emitter::ST_D(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12) {
    Write32(EncodeDJSk12(Opcode32::ST_D, rd, rj, si12));
}

void LoongArch64Emitter::LDX_B(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDX_B, rd, rj, rk));
}

void LoongArch64Emitter::LDX_H(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDX_H, rd, rj, rk));
}

void LoongArch64Emitter::LDX_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDX_W, rd, rj, rk));
}

void LoongArch64Emitter::LDX_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDX_D, rd, rj, rk));
}

void LoongArch64Emitter::LDX_BU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDX_BU, rd, rj, rk));
}

void LoongArch64Emitter::LDX_HU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDX_HU, rd, rj, rk));
}

void LoongArch64Emitter::LDX_WU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDX_WU, rd, rj, rk));
}

void LoongArch64Emitter::STX_B(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::STX_B, rd, rj, rk));
}

void LoongArch64Emitter::STX_H(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::STX_H, rd, rj, rk));
}

void LoongArch64Emitter::STX_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::STX_W, rd, rj, rk));
}

void LoongArch64Emitter::STX_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::STX_D, rd, rj, rk));
}

void LoongArch64Emitter::LDPTR_W(LoongArch64Reg rd, LoongArch64Reg rj, s16 si14) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJSk14ps2(Opcode32::LDPTR_W, rd, rj, si14));
}

void LoongArch64Emitter::LDPTR_D(LoongArch64Reg rd, LoongArch64Reg rj, s16 si14) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJSk14ps2(Opcode32::LDPTR_W, rd, rj, si14));
}

void LoongArch64Emitter::STPTR_W(LoongArch64Reg rd, LoongArch64Reg rj, s16 si14) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJSk14ps2(Opcode32::LDPTR_W, rd, rj, si14));
}

void LoongArch64Emitter::STPTR_D(LoongArch64Reg rd, LoongArch64Reg rj, s16 si14) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJSk14ps2(Opcode32::LDPTR_W, rd, rj, si14));
}

void LoongArch64Emitter::PRELD(u32 hint, LoongArch64Reg rj, s16 si12) {
    _assert_msg_(rj != R_ZERO, "%s load from zero is a HINT", __func__);
    _assert_msg_(hint == 0 || hint == 8, "%s hint represents a NOP", __func__);
    Write32(EncodeUd5JSk12(Opcode32::PRELD, hint, rj, si12));
}

void LoongArch64Emitter::PRELDX(u32 hint, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rj != R_ZERO, "%s load from zero is a HINT", __func__);
    _assert_msg_(hint == 0 || hint == 8, "%s hint represents a NOP", __func__);
    Write32(EncodeUd5JK(Opcode32::PRELDX, hint, rj, rk));
}

void LoongArch64Emitter::LDGT_B(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDGT_B, rd, rj, rk));
}

void LoongArch64Emitter::LDGT_H(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDGT_H, rd, rj, rk));
}

void LoongArch64Emitter::LDGT_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDGT_W, rd, rj, rk));
}

void LoongArch64Emitter::LDGT_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDGT_D, rd, rj, rk));
}

void LoongArch64Emitter::LDLE_B(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDLE_B, rd, rj, rk));
}

void LoongArch64Emitter::LDLE_H(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDLE_H, rd, rj, rk));
}

void LoongArch64Emitter::LDLE_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDLE_W, rd, rj, rk));
}

void LoongArch64Emitter::LDLE_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::LDLE_D, rd, rj, rk));
}

void LoongArch64Emitter::STGT_B(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::STGT_B, rd, rj, rk));
}

void LoongArch64Emitter::STGT_H(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::STGT_H, rd, rj, rk));
}

void LoongArch64Emitter::STGT_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::STGT_W, rd, rj, rk));
}

void LoongArch64Emitter::STGT_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::STGT_D, rd, rj, rk));
}

void LoongArch64Emitter::STLE_B(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::STLE_B, rd, rj, rk));
}

void LoongArch64Emitter::STLE_H(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::STLE_H, rd, rj, rk));
}

void LoongArch64Emitter::STLE_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::STLE_W, rd, rj, rk));
}

void LoongArch64Emitter::STLE_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJK(Opcode32::STLE_D, rd, rj, rk));
}

void LoongArch64Emitter::AMSWAP_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMSWAP_W, rd, rk, rj));
}

void LoongArch64Emitter::AMSWAP_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMSWAP_DB_W, rd, rk, rj));
}

void LoongArch64Emitter::AMSWAP_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMSWAP_D, rd, rk, rj));
}

void LoongArch64Emitter::AMSWAP_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMSWAP_DB_D, rd, rk, rj));
}

void LoongArch64Emitter::AMADD_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMADD_W, rd, rk, rj));
}

void LoongArch64Emitter::AMADD_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMADD_DB_W, rd, rk, rj));
}

void LoongArch64Emitter::AMADD_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMADD_D, rd, rk, rj));
}

void LoongArch64Emitter::AMADD_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMADD_DB_D, rd, rk, rj));
}

void LoongArch64Emitter::AMAND_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMAND_W, rd, rk, rj));
}

void LoongArch64Emitter::AMAND_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMAND_DB_W, rd, rk, rj));
}

void LoongArch64Emitter::AMAND_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMAND_D, rd, rk, rj));
}

void LoongArch64Emitter::AMAND_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMAND_DB_D, rd, rk, rj));
}

void LoongArch64Emitter::AMOR_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMOR_W, rd, rk, rj));
}

void LoongArch64Emitter::AMOR_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMOR_DB_W, rd, rk, rj));
}

void LoongArch64Emitter::AMOR_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMOR_D, rd, rk, rj));
}

void LoongArch64Emitter::AMOR_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMOR_DB_D, rd, rk, rj));
}

void LoongArch64Emitter::AMXOR_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMXOR_W, rd, rk, rj));
}

void LoongArch64Emitter::AMXOR_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMXOR_DB_W, rd, rk, rj));
}

void LoongArch64Emitter::AMXOR_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMXOR_D, rd, rk, rj));
}

void LoongArch64Emitter::AMXOR_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMXOR_DB_D, rd, rk, rj));
}

void LoongArch64Emitter::AMMAX_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMAX_W, rd, rk, rj));
}

void LoongArch64Emitter::AMMAX_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMAX_DB_W, rd, rk, rj));
}

void LoongArch64Emitter::AMMAX_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMAX_D, rd, rk, rj));
}

void LoongArch64Emitter::AMMAX_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMAX_DB_D, rd, rk, rj));
}

void LoongArch64Emitter::AMMIN_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMIN_W, rd, rk, rj));
}

void LoongArch64Emitter::AMMIN_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMIN_DB_W, rd, rk, rj));
}

void LoongArch64Emitter::AMMIN_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMIN_D, rd, rk, rj));
}

void LoongArch64Emitter::AMMIN_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMIN_DB_D, rd, rk, rj));
}

void LoongArch64Emitter::AMMAX_WU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMAX_WU, rd, rk, rj));
}

void LoongArch64Emitter::AMMAX_DB_WU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMAX_DB_WU, rd, rk, rj));
}

void LoongArch64Emitter::AMMAX_DU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMAX_DU, rd, rk, rj));
}

void LoongArch64Emitter::AMMAX_DB_DU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMAX_DB_DU, rd, rk, rj));
}

void LoongArch64Emitter::AMMIN_WU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMIN_WU, rd, rk, rj));
}

void LoongArch64Emitter::AMMIN_DB_WU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMIN_DB_WU, rd, rk, rj));
}

void LoongArch64Emitter::AMMIN_DU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMIN_DU, rd, rk, rj));
}

void LoongArch64Emitter::AMMIN_DB_DU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMMIN_DB_DU, rd, rk, rj));
}

void LoongArch64Emitter::AMSWAP_B(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMSWAP_B, rd, rk, rj));
}

void LoongArch64Emitter::AMSWAP_DB_B(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMSWAP_DB_B, rd, rk, rj));
}

void LoongArch64Emitter::AMSWAP_H(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMSWAP_H, rd, rk, rj));
}

void LoongArch64Emitter::AMSWAP_DB_H(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMSWAP_DB_H, rd, rk, rj));
}

void LoongArch64Emitter::AMADD_B(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMADD_B, rd, rk, rj));
}

void LoongArch64Emitter::AMADD_DB_B(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMADD_DB_B, rd, rk, rj));
}

void LoongArch64Emitter::AMADD_H(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMADD_H, rd, rk, rj));
}

void LoongArch64Emitter::AMADD_DB_H(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMADD_DB_H, rd, rk, rj));
}

void LoongArch64Emitter::AMCAS_B(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMCAS_B, rd, rk, rj));
}

void LoongArch64Emitter::AMCAS_DB_B(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMCAS_DB_B, rd, rk, rj));
}

void LoongArch64Emitter::AMCAS_H(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMCAS_H, rd, rk, rj));
}

void LoongArch64Emitter::AMCAS_DB_H(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMCAS_DB_H, rd, rk, rj));
}

void LoongArch64Emitter::AMCAS_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMCAS_W, rd, rk, rj));
}

void LoongArch64Emitter::AMCAS_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMCAS_DB_W, rd, rk, rj));
}

void LoongArch64Emitter::AMCAS_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMCAS_D, rd, rk, rj));
}

void LoongArch64Emitter::AMCAS_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::AMCAS_DB_D, rd, rk, rj));
}

void LoongArch64Emitter::CRC_W_B_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::CRC_W_B_W, rd, rk, rj));
}

void LoongArch64Emitter::CRC_W_H_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::CRC_W_H_W, rd, rk, rj));
}

void LoongArch64Emitter::CRC_W_W_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::CRC_W_W_W, rd, rk, rj));
}

void LoongArch64Emitter::CRC_W_D_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::CRC_W_D_W, rd, rk, rj));
}

void LoongArch64Emitter::CRCC_W_B_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::CRCC_W_B_W, rd, rk, rj));
}

void LoongArch64Emitter::CRCC_W_H_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::CRCC_W_H_W, rd, rk, rj));
}

void LoongArch64Emitter::CRCC_W_W_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::CRCC_W_W_W, rd, rk, rj));
}

void LoongArch64Emitter::CRCC_W_D_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDKJ(Opcode32::CRCC_W_D_W, rd, rk, rj));
}

void LoongArch64Emitter::SYSCALL(u16 code) {
    Write32(EncodeUd15(Opcode32::SYSCALL, code));
}

void LoongArch64Emitter::BREAK(u16 code) {
    Write32(EncodeUd15(Opcode32::BREAK, code));
}

void LoongArch64Emitter::ASRTLE_D(LoongArch64Reg rj, LoongArch64Reg rk) {
    Write32(EncodeJK(Opcode32::ASRTLE_D, rj, rk));
}

void LoongArch64Emitter::ASRTGT_D(LoongArch64Reg rj, LoongArch64Reg rk) {
    Write32(EncodeJK(Opcode32::ASRTGT_D, rj, rk));
}

void LoongArch64Emitter::RDTIMEL_W(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::RDTIMEL_W, rd, rj));
}

void LoongArch64Emitter::RDTIMEH_W(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::RDTIMEH_W, rd, rj));
}

void LoongArch64Emitter::RDTIME_D(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::RDTIME_D, rd, rj));
}

void LoongArch64Emitter::CPUCFG(LoongArch64Reg rd, LoongArch64Reg rj) {
    _assert_msg_(rd != R_ZERO, "%s write to zero is a HINT", __func__);
    Write32(EncodeDJ(Opcode32::CPUCFG, rd, rj));
}

void LoongArch64Emitter::FADD_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FADD_S, fd, fj, fk));
}

void LoongArch64Emitter::FADD_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FADD_D, fd, fj, fk));
}

void LoongArch64Emitter::FSUB_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FSUB_S, fd, fj, fk));
}

void LoongArch64Emitter::FSUB_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FSUB_D, fd, fj, fk));
}

void LoongArch64Emitter::FMUL_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FMUL_S, fd, fj, fk));
}

void LoongArch64Emitter::FMUL_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FMUL_D, fd, fj, fk));
}

void LoongArch64Emitter::FDIV_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FDIV_S, fd, fj, fk));
}

void LoongArch64Emitter::FDIV_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FDIV_D, fd, fj, fk));
}

void LoongArch64Emitter::FMADD_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa) {
    Write32(EncodeFdFjFkFa(Opcode32::FMADD_S, fd, fj, fk, fa));
}

void LoongArch64Emitter::FMADD_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa) {
    Write32(EncodeFdFjFkFa(Opcode32::FMADD_D, fd, fj, fk, fa));
}

void LoongArch64Emitter::FMSUB_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa) {
    Write32(EncodeFdFjFkFa(Opcode32::FMSUB_S, fd, fj, fk, fa));
}

void LoongArch64Emitter::FMSUB_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa) {
    Write32(EncodeFdFjFkFa(Opcode32::FMSUB_D, fd, fj, fk, fa));
}

void LoongArch64Emitter::FNMADD_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa) {
    Write32(EncodeFdFjFkFa(Opcode32::FNMADD_S, fd, fj, fk, fa));
}

void LoongArch64Emitter::FNMADD_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa) {
    Write32(EncodeFdFjFkFa(Opcode32::FNMADD_D, fd, fj, fk, fa));
}

void LoongArch64Emitter::FNMSUB_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa) {
    Write32(EncodeFdFjFkFa(Opcode32::FNMSUB_S, fd, fj, fk, fa));
}

void LoongArch64Emitter::FNMSUB_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa) {
    Write32(EncodeFdFjFkFa(Opcode32::FNMSUB_D, fd, fj, fk, fa));
}

void LoongArch64Emitter::FMAX_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FMAX_S, fd, fj, fk));
}

void LoongArch64Emitter::FMAX_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FMAX_D, fd, fj, fk));
}

void LoongArch64Emitter::FMIN_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FMIN_S, fd, fj, fk));
}

void LoongArch64Emitter::FMIN_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FMIN_D, fd, fj, fk));
}

void LoongArch64Emitter::FMAXA_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FMAXA_S, fd, fj, fk));
}

void LoongArch64Emitter::FMAXA_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FMAXA_D, fd, fj, fk));
}

void LoongArch64Emitter::FMINA_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FMINA_S, fd, fj, fk));
}

void LoongArch64Emitter::FMINA_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FMINA_D, fd, fj, fk));
}

void LoongArch64Emitter::FABS_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FABS_S, fd, fj));
}

void LoongArch64Emitter::FABS_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FABS_D, fd, fj));
}

void LoongArch64Emitter::FNEG_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FNEG_S, fd, fj));
}

void LoongArch64Emitter::FNEG_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FNEG_D, fd, fj));
}

void LoongArch64Emitter::FSQRT_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FSQRT_S, fd, fj));
}

void LoongArch64Emitter::FSQRT_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FSQRT_D, fd, fj));
}

void LoongArch64Emitter::FRECIP_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FRECIP_S, fd, fj));
}

void LoongArch64Emitter::FRECIP_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FRECIP_D, fd, fj));
}

void LoongArch64Emitter::FRSQRT_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FRSQRT_S, fd, fj));
}

void LoongArch64Emitter::FRSQRT_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FRSQRT_D, fd, fj));
}

void LoongArch64Emitter::FSCALEB_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FSCALEB_S, fd, fj, fk));
}

void LoongArch64Emitter::FSCALEB_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FSCALEB_D, fd, fj, fk));
}

void LoongArch64Emitter::FLOGB_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FLOGB_S, fd, fj));
}

void LoongArch64Emitter::FLOGB_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FLOGB_D, fd, fj));
}

void LoongArch64Emitter::FCOPYSIGN_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FCOPYSIGN_S, fd, fj, fk));
}

void LoongArch64Emitter::FCOPYSIGN_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk) {
    Write32(EncodeFdFjFk(Opcode32::FCOPYSIGN_D, fd, fj, fk));
}

void LoongArch64Emitter::FCLASS_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FCLASS_S, fd, fj));
}

void LoongArch64Emitter::FCLASS_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FCLASS_D, fd, fj));
}

void LoongArch64Emitter::FRECIPE_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FRECIPE_S, fd, fj));
}

void LoongArch64Emitter::FRECIPE_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FRECIPE_D, fd, fj));
}

void LoongArch64Emitter::FRSQRTE_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FRSQRTE_S, fd, fj));
}

void LoongArch64Emitter::FRSQRTE_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FRSQRTE_D, fd, fj));
}

void LoongArch64Emitter::FCMP_COND_S(LoongArch64CFR cd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Fcond cond) {
    Write32(EncodeCdFjFkFcond(Opcode32::FCMP_COND_S, cd, fj, fk, cond));
}

void LoongArch64Emitter::FCMP_COND_D(LoongArch64CFR cd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Fcond cond) {
    Write32(EncodeCdFjFkFcond(Opcode32::FCMP_COND_D, cd, fj, fk, cond));
}

void LoongArch64Emitter::FCVT_S_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FCVT_S_D, fd, fj));
}

void LoongArch64Emitter::FCVT_D_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FCVT_D_S, fd, fj));
}

void LoongArch64Emitter::FFINT_S_W(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FFINT_S_W, fd, fj));
}

void LoongArch64Emitter::FFINT_S_L(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FFINT_S_L, fd, fj));
}

void LoongArch64Emitter::FFINT_D_W(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FFINT_D_W, fd, fj));
}

void LoongArch64Emitter::FFINT_D_L(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FFINT_D_L, fd, fj));
}

void LoongArch64Emitter::FTINT_W_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINT_W_S, fd, fj));
}

void LoongArch64Emitter::FTINT_W_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINT_W_D, fd, fj));
}

void LoongArch64Emitter::FTINT_L_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINT_L_S, fd, fj));
}

void LoongArch64Emitter::FTINT_L_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINT_L_D, fd, fj));
}

void LoongArch64Emitter::FTINTRM_W_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRM_W_S, fd, fj));
}

void LoongArch64Emitter::FTINTRM_W_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRM_W_D, fd, fj));
}

void LoongArch64Emitter::FTINTRM_L_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRM_L_S, fd, fj));
}

void LoongArch64Emitter::FTINTRM_L_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRM_L_D, fd, fj));
}

void LoongArch64Emitter::FTINTRP_W_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRP_W_S, fd, fj));
}

void LoongArch64Emitter::FTINTRP_W_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRP_W_D, fd, fj));
}

void LoongArch64Emitter::FTINTRP_L_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRP_L_S, fd, fj));
}

void LoongArch64Emitter::FTINTRP_L_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRP_L_D, fd, fj));
}

void LoongArch64Emitter::FTINTRZ_W_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRZ_W_S, fd, fj));
}

void LoongArch64Emitter::FTINTRZ_W_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRZ_W_D, fd, fj));
}

void LoongArch64Emitter::FTINTRZ_L_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRZ_L_S, fd, fj));
}

void LoongArch64Emitter::FTINTRZ_L_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRZ_L_D, fd, fj));
}

void LoongArch64Emitter::FTINTRNE_W_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRNE_W_S, fd, fj));
}

void LoongArch64Emitter::FTINTRNE_W_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRNE_W_D, fd, fj));
}

void LoongArch64Emitter::FTINTRNE_L_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRNE_L_S, fd, fj));
}

void LoongArch64Emitter::FTINTRNE_L_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FTINTRNE_L_D, fd, fj));
}

void LoongArch64Emitter::FRINT_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FRINT_S, fd, fj));
}

void LoongArch64Emitter::FRINT_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FRINT_D, fd, fj));
}

void LoongArch64Emitter::FMOV_S(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FMOV_S, fd, fj));
}

void LoongArch64Emitter::FMOV_D(LoongArch64Reg fd, LoongArch64Reg fj) {
    Write32(EncodeFdFj(Opcode32::FMOV_D, fd, fj));
}

void LoongArch64Emitter::FSEL(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64CFR ca) {
    Write32(EncodeFdFjFkCa(Opcode32::FSEL, fd, fj, fk, ca));
}

void LoongArch64Emitter::MOVGR2FR_W(LoongArch64Reg fd, LoongArch64Reg rj) {
    Write32(EncodeFdJ(Opcode32::MOVGR2FR_W, fd, rj));
}

void LoongArch64Emitter::MOVGR2FR_D(LoongArch64Reg fd, LoongArch64Reg rj) {
    Write32(EncodeFdJ(Opcode32::MOVGR2FR_D, fd, rj));
}

void LoongArch64Emitter::MOVGR2FRH_W(LoongArch64Reg fd, LoongArch64Reg rj) {
    Write32(EncodeFdJ(Opcode32::MOVGR2FRH_W, fd, rj));
}

void LoongArch64Emitter::MOVFR2GR_S(LoongArch64Reg rj, LoongArch64Reg fd) {
    Write32(EncodeDFj(Opcode32::MOVFR2GR_S, rj, fd));
}

void LoongArch64Emitter::MOVFR2GR_D(LoongArch64Reg rj, LoongArch64Reg fd) {
    Write32(EncodeDFj(Opcode32::MOVFR2GR_D, rj, fd));
}

void LoongArch64Emitter::MOVFRH2GR_S(LoongArch64Reg rj, LoongArch64Reg fd) {
    Write32(EncodeDFj(Opcode32::MOVFRH2GR_S, rj, fd));
}

void LoongArch64Emitter::MOVGR2FCSR(LoongArch64FCSR fcsr, LoongArch64Reg rj) {
    Write32(EncodeJUd5(Opcode32::MOVGR2FCSR, fcsr, rj));
}

void LoongArch64Emitter::MOVFCSR2GR(LoongArch64Reg rd, LoongArch64FCSR fcsr) {
    Write32(EncodeDUj5(Opcode32::MOVFCSR2GR, rd, fcsr));
}

void LoongArch64Emitter::MOVFR2CF(LoongArch64CFR cd, LoongArch64Reg fj) {
    Write32(EncodeCdFj(Opcode32::MOVFR2CF, cd, fj));
}

void LoongArch64Emitter::MOVCF2FR(LoongArch64Reg fd, LoongArch64CFR cj) {
    Write32(EncodeFdCj(Opcode32::MOVCF2FR, fd, cj));
}

void LoongArch64Emitter::MOVGR2CF(LoongArch64CFR cd, LoongArch64Reg rj) {
    Write32(EncodeCdJ(Opcode32::MOVGR2CF, cd, rj));
}

void LoongArch64Emitter::MOVCF2GR(LoongArch64Reg rd, LoongArch64CFR cj) {
    Write32(EncodeDCj(Opcode32::MOVGR2CF, rd, cj));
}

void LoongArch64Emitter::BCEQZ(LoongArch64CFR cj, s32 offs21) {
    Write32(EncodeCjSd5k16ps2(Opcode32::BCEQZ, cj, offs21));
}

void LoongArch64Emitter::BCNEZ(LoongArch64CFR cj, s32 offs21) {
    Write32(EncodeCjSd5k16ps2(Opcode32::BCNEZ, cj, offs21));
}

void LoongArch64Emitter::FLD_S(LoongArch64Reg fd, LoongArch64Reg rj, s16 si12) {
    Write32(EncodeFdJSk12(Opcode32::FLD_S, fd, rj, si12));
}

void LoongArch64Emitter::FLD_D(LoongArch64Reg fd, LoongArch64Reg rj, s16 si12) {
    Write32(EncodeFdJSk12(Opcode32::FLD_D, fd, rj, si12));
}

void LoongArch64Emitter::FST_S(LoongArch64Reg fd, LoongArch64Reg rj, s16 si12) {
    Write32(EncodeFdJSk12(Opcode32::FST_S, fd, rj, si12));
}

void LoongArch64Emitter::FST_D(LoongArch64Reg fd, LoongArch64Reg rj, s16 si12) {
    Write32(EncodeFdJSk12(Opcode32::FST_D, fd, rj, si12));
}

void LoongArch64Emitter::FLDX_S(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk) {
    Write32(EncodeFdJK(Opcode32::FLDX_S, fd, rj, rk));
}

void LoongArch64Emitter::FLDX_D(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk) {
    Write32(EncodeFdJK(Opcode32::FLDX_D, fd, rj, rk));
}

void LoongArch64Emitter::FSTX_S(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk) {
    Write32(EncodeFdJK(Opcode32::FSTX_S, fd, rj, rk));
}

void LoongArch64Emitter::FSTX_D(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk) {
    Write32(EncodeFdJK(Opcode32::FSTX_D, fd, rj, rk));
}

void LoongArch64Emitter::FLDGT_S(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk) {
    Write32(EncodeFdJK(Opcode32::FLDGT_S, fd, rj, rk));
}

void LoongArch64Emitter::FLDGT_D(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk) {
    Write32(EncodeFdJK(Opcode32::FLDGT_D, fd, rj, rk));
}

void LoongArch64Emitter::FLDLE_S(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk) {
    Write32(EncodeFdJK(Opcode32::FLDLE_S, fd, rj, rk));
}

void LoongArch64Emitter::FLDLE_D(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk) {
    Write32(EncodeFdJK(Opcode32::FLDLE_D, fd, rj, rk));
}

void LoongArch64Emitter::FSTGT_S(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk) {
    Write32(EncodeFdJK(Opcode32::FSTGT_S, fd, rj, rk));
}

void LoongArch64Emitter::FSTGT_D(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk) {
    Write32(EncodeFdJK(Opcode32::FSTGT_D, fd, rj, rk));
}

void LoongArch64Emitter::FSTLE_S(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk) {
    Write32(EncodeFdJK(Opcode32::FSTLE_S, fd, rj, rk));
}

void LoongArch64Emitter::FSTLE_D(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk) {
    Write32(EncodeFdJK(Opcode32::FSTLE_D, fd, rj, rk));
}

void LoongArch64CodeBlock::PoisonMemory(int offset) {
    // So we can adjust region to writable space.  Might be zero.
	ptrdiff_t writable = writable_ - code_;

	u32 *ptr = (u32 *)(region + offset + writable);
	u32 *maxptr = (u32 *)(region + region_size - offset + writable);
    // If our memory isn't a multiple of u32 then this won't write the last remaining bytes with anything
	// Less than optimal, but there would be nothing we could do but throw a runtime warning anyway.
	// LoongArch64: 0x002a0000 = BREAK 0
	while (ptr < maxptr)
		*ptr++ = 0x002a0000;
}

};
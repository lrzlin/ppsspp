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

#pragma once

#include <cstdint>
#include <cstring>
#include <type_traits>
#include "Common/CodeBlock.h"
#include "Common/Common.h"

namespace LoongArch64Gen {

enum LoongArch64Reg {
    // General-purpose Registers (64-bit)
    // https://loongson.github.io/LoongArch-Documentation/LoongArch-Vol1-EN.html#_general_purpose_registers
	R0 = 0, R1, R2, R3, R4, R5, R6, R7,
	R8, R9, R10, R11, R12, R13, R14, R15,
	R16, R17, R18, R19, R20, R21, R22, R23,
	R24, R25, R26, R27, R28, R29, R30, R31,

    R_ZERO = 0,
	R_RA = 1,
    R_SP = 3,

    // Floating-point Registers (64-bit)
    // https://loongson.github.io/LoongArch-Documentation/LoongArch-Vol1-EN.html#floating-point-registers
	F0 = 0x20, F1, F2, F3, F4, F5, F6, F7,
	F8, F9, F10, F11, F12, F13, F14, F15,
	F16, F17, F18, F19, F20, F21, F22, F23,
	F24, F25, F26, F27, F28, F29, F30, F31,

    // FP register f0 and LSX register v0 and LASX register x0 share the lowest 64 bits
    // LSX register v0 and LASX register x0 share the lowest 128 bits
    // https://jia.je/unofficial-loongarch-intrinsics-guide/

    // LSX Registers (128-bit)
    V0 = 0x40, V1, V2, V3, V4, V5, V6, V7,
	V8, V9, V10, V11, V12, V13, V14, V15,
	V16, V17, V18, V19, V20, V21, V22, V23,
	V24, V25, V26, V27, V28, V29, V30, V31,

    // LASX Registers
    X0 = 0x60, X1, X2, X3, X4, X5, X6, X7,
	X8, X9, X10, X11, X12, X13, X14, X15,
	X16, X17, X18, X19, X20, X21, X22, X23,
	X24, X25, X26, X27, X28, X29, X30, X31,

    INVALID_REG = 0xFFFFFFFF,
};

enum LoongArch64CFR {
    // Condition Flag Register
    // The length of CFR is 1 bit.
    FCC0 = 0, FCC1, FCC2, FCC3, FCC4, FCC5, FCC6, FCC7,
};

enum LoongArch64FCSR {
    // Floating-point Control and Status Register
    // The length of FCSR0 is 29 bits.
    // FCSR1-FCSR3 are aliases of some fields in fcsr0.
    FCSR0 = 0, FCSR1, FCSR2, FCSR3,
};

enum class FixupBranchType {
	B,
	J,
    BZ,
};

enum class LoongArch64Fcond {
    // Conditions used in FCMP instruction
    CAF = 0x0,
    CUN = 0x8,
    CEQ = 0x4,
    CUEQ = 0xC,
    CLT = 0x2,
    CULT = 0xA,
    CLE = 0x6,
    CULE = 0xE,
    CNE = 0x10,
    COR = 0x14,
    CUNE = 0x18,
    SAF = 0x1,
    SUN = 0x9,
    SEQ = 0x5,
    SUEQ = 0xD,
    SLT = 0x3,
    SULT = 0xB,
    SLE = 0x7,
    SULE = 0xF,
    SNE = 0x11,
    SOR = 0x15,
    SUNE = 0x19,
};

struct FixupBranch {
	FixupBranch() {}
	FixupBranch(const u8 *p, FixupBranchType t) : ptr(p), type(t) {}
	FixupBranch(FixupBranch &&other);
	FixupBranch(const FixupBranch &) = delete;
	~FixupBranch();

	FixupBranch &operator =(FixupBranch &&other);
	FixupBranch &operator =(const FixupBranch &other) = delete;

    // Pointer to executable code address.
	const u8 *ptr = nullptr;
	FixupBranchType type = FixupBranchType::B;
};

class LoongArch64Emitter {
public:
    LoongArch64Emitter() {}
    LoongArch64Emitter(const u8 *codePtr, u8 *writablePtr);
    virtual ~LoongArch64Emitter() {}

	void SetCodePointer(const u8 *ptr, u8 *writePtr);
	const u8 *GetCodePointer() const;
    u8 *GetWritableCodePtr();

    void ReserveCodeSpace(u32 bytes);
	const u8 *AlignCode16();
	const u8 *AlignCodePage();
	void FlushIcache();
	void FlushIcacheSection(const u8 *start, const u8 *end);

    void SetJumpTarget(FixupBranch &branch);
    bool BranchInRange(const void *func) const;
	bool JumpInRange(const void *func) const;
    bool BranchZeroInRange(const void *func) const;

    void QuickJump(LoongArch64Reg scratchreg, LoongArch64Reg rd, const u8 *dst);
	void QuickJ(LoongArch64Reg scratchreg, const u8 *dst) {
		QuickJump(scratchreg, R_ZERO, dst);
	}
	void QuickCallFunction(const u8 *func, LoongArch64Reg scratchreg = R_RA) {
		QuickJump(scratchreg, R_RA, func);
	}
	template <typename T>
	void QuickCallFunction(T *func, LoongArch64Reg scratchreg = R_RA) {
		static_assert(std::is_function<T>::value, "QuickCallFunction without function");
		QuickCallFunction((const u8 *)func, scratchreg);
	}

    // https://loongson.github.io/LoongArch-Documentation/LoongArch-Vol1-EN.html
    // https://github.com/loongson-community/loongarch-opcodes/

    // Basic Integer Instructions

    // Arithmetic Operation Instructions
    void ADD_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void ADD_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void SUB_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void SUB_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK

    void ADDI_W(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12
    void ADDI_D(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12
    void ADDU16I_D(LoongArch64Reg rd, LoongArch64Reg rj, s16 si16); // DJSk16

    void ALSL_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk, u8 sa2); // DJKUa2pp1
    void ALSL_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk, u8 sa2); // DJKUa2pp1
    void ALSL_WU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk, u8 sa2);// DJKUa2pp1

    void LU12I_W(LoongArch64Reg rd, s32 si20); // DSj20
    void LU32I_D(LoongArch64Reg rd, s32 si20); // DSj20
    void LU52I_D(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12

    void SLT(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void SLTU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK

    void SLTI(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12
    void SLTUI(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12

    void PCADDI(LoongArch64Reg rd, s32 si20); // DSj20
    void PCADDU12I(LoongArch64Reg rd, s32 si20); // DSj20
    void PCADDU18I(LoongArch64Reg rd, s32 si20); // DSj20
    void PCALAU12I(LoongArch64Reg rd, s32 si20); // DSj20

    void AND(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void OR(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void NOR(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void XOR(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void ANDN(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void ORN(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK

    void ANDI(LoongArch64Reg rd, LoongArch64Reg rj, u16 ui12); // DJUk12
    void ORI(LoongArch64Reg rd, LoongArch64Reg rj, u16 ui12); // DJUk12
    void XORI(LoongArch64Reg rd, LoongArch64Reg rj, u16 ui12); // DJUk12

    void NOP() {
        ANDI(R_ZERO, R_ZERO, 0);
    }
    void MOVE(LoongArch64Reg rd, LoongArch64Reg rj) {
        OR(rd, rj, R_ZERO);
    }

    template <typename T>
	void LI(LoongArch64Reg rd, const T &v) {
		_assert_msg_(rd != R_ZERO, "LI to X0");
		_assert_msg_(rd < F0, "LI to non-GPR");

		uint64_t value = AsImmediate<T, std::is_signed<T>::value>(v);
		SetRegToImmediate(rd, value);
	}

    void MUL_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void MULH_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void MULH_WU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void MUL_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void MULH_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void MULH_DU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK

    void MULW_D_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void MULW_D_WU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK

    void DIV_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void MOD_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void DIV_WU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void MOD_WU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void DIV_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void MOD_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void DIV_DU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void MOD_DU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK

    // Bit-shift Instructions
    void SLL_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void SRL_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void SRA_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void ROTR_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK

    void SLLI_W(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui5); // DJUk5
    void SRLI_W(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui5); // DJUk5
    void SRAI_W(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui5); // DJUk5
    void ROTRI_W(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui5); // DJUk5

    void SLL_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void SRL_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void SRA_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void ROTR_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK

    void SLLI_D(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui6); // DJUk6
    void SRLI_D(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui6); // DJUk6
    void SRAI_D(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui6); // DJUk6
    void ROTRI_D(LoongArch64Reg rd, LoongArch64Reg rj, u8 ui6); // DJUk6

    // Bit-manipulation Instructions
    void EXT_W_B(LoongArch64Reg rd, LoongArch64Reg rj); // DJ
    void EXT_W_H(LoongArch64Reg rd, LoongArch64Reg rj); // DJ

    void CLO_W(LoongArch64Reg rd, LoongArch64Reg rj); // DJ
    void CLO_D(LoongArch64Reg rd, LoongArch64Reg rj); // DJ
    void CLZ_W(LoongArch64Reg rd, LoongArch64Reg rj); // DJ
    void CLZ_D(LoongArch64Reg rd, LoongArch64Reg rj); // DJ
    void CTO_W(LoongArch64Reg rd, LoongArch64Reg rj); // DJ
    void CTO_D(LoongArch64Reg rd, LoongArch64Reg rj); // DJ
    void CTZ_W(LoongArch64Reg rd, LoongArch64Reg rj); // DJ
    void CTZ_D(LoongArch64Reg rd, LoongArch64Reg rj); // DJ

    void BYTEPICK_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk, u8 sa2); // DJKUa2
    void BYTEPICK_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk, u8 sa3); // DJKUa3

    void REVB_2H(LoongArch64Reg rd, LoongArch64Reg rj); // DJ
    void REVB_4H(LoongArch64Reg rd, LoongArch64Reg rj); // DJ
    void REVB_2W(LoongArch64Reg rd, LoongArch64Reg rj); // DJ
    void REVB_D(LoongArch64Reg rd, LoongArch64Reg rj); // DJ

    void BITREV_4B(LoongArch64Reg rd, LoongArch64Reg rj); // DJ
    void BITREV_8B(LoongArch64Reg rd, LoongArch64Reg rj); // DJ

    void BITREV_W(LoongArch64Reg rd, LoongArch64Reg rj); //DJ
    void BITREV_D(LoongArch64Reg rd, LoongArch64Reg rj); // DJ

    void BSTRINS_W(LoongArch64Reg rd, LoongArch64Reg rj, u8 msbw, u8 lsbw); // DJUk5Um5
    void BSTRINS_D(LoongArch64Reg RD, LoongArch64Reg RJ, u8 msbd, u8 lsbd); // DJUk6Um6

    void BSTRPICK_W(LoongArch64Reg RD, LoongArch64Reg RJ, u8 msbd, u8 lsbd); // DJUk5Um5
    void BSTRPICK_D(LoongArch64Reg RD, LoongArch64Reg RJ, u8 msbd, u8 lsbd); // DJUk6Um6

    void MASKEQZ(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void MASKNEZ(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK

    // Branch Instructions
    void BEQ(LoongArch64Reg rj, LoongArch64Reg rd, const void *dst); // JDSk16ps2
    void BNE(LoongArch64Reg rj, LoongArch64Reg rd, const void *dst); // JDSk16ps2
    void BLT(LoongArch64Reg rj, LoongArch64Reg rd, const void *dst); // JDSk16ps2
    void BGE(LoongArch64Reg rj, LoongArch64Reg rd, const void *dst); // JDSk16ps2
    void BLTU(LoongArch64Reg rj, LoongArch64Reg rd, const void *dst); // JDSk16ps2
    void BGEU(LoongArch64Reg rj, LoongArch64Reg rd, const void *dst); // JDSk16ps2
    FixupBranch BEQ(LoongArch64Reg rj, LoongArch64Reg rd);
    FixupBranch BNE(LoongArch64Reg rj, LoongArch64Reg rd);
    FixupBranch BLT(LoongArch64Reg rj, LoongArch64Reg rd);
    FixupBranch BGE(LoongArch64Reg rj, LoongArch64Reg rd);
    FixupBranch BLTU(LoongArch64Reg rj, LoongArch64Reg rd);
    FixupBranch BGEU(LoongArch64Reg rj, LoongArch64Reg rd);

    void BEQZ(LoongArch64Reg rj, const void *dst); // JSd5k16ps2
    void BNEZ(LoongArch64Reg rj, const void *dst); // JSd5k16ps2
    FixupBranch BEQZ(LoongArch64Reg rj);
    FixupBranch BNEZ(LoongArch64Reg rj);

    void B(const void *dst); // Sd10k16ps2
    void BL(const void *dst); // Sd10k16ps2
    FixupBranch B();
    FixupBranch BL();

    void JIRL(LoongArch64Reg rd, LoongArch64Reg rj, s32 offs16); // DJSk16ps2

    void JR(LoongArch64Reg rj) {
        JIRL(R_ZERO, rj, 0);
    }
    void RET() {
        JIRL(R_ZERO, R_RA, 0);
    }

    // Common Memory Access Instructions
    void LD_B(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12
    void LD_H(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12
    void LD_W(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12
    void LD_D(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12
    void LD_BU(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12
    void LD_HU(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12
    void LD_WU(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12
    void ST_B(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12
    void ST_H(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12
    void ST_W(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12
    void ST_D(LoongArch64Reg rd, LoongArch64Reg rj, s16 si12); // DJSk12

    void LDX_B(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void LDX_H(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void LDX_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void LDX_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void LDX_BU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void LDX_HU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void LDX_WU(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void STX_B(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void STX_H(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void STX_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void STX_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK

    void LDPTR_W(LoongArch64Reg rd, LoongArch64Reg rj, s16 si14); // DJSk14ps2
    void LDPTR_D(LoongArch64Reg rd, LoongArch64Reg rj, s16 si14); // DJSk14ps2
    void STPTR_W(LoongArch64Reg rd, LoongArch64Reg rj, s16 si14); // DJSk14ps2
    void STPTR_D(LoongArch64Reg rd, LoongArch64Reg rj, s16 si14); // DJSk14ps2

    void PRELD(u32 hint, LoongArch64Reg rj, s16 si12); // Ud5JSk12
    void PRELDX(u32 hint, LoongArch64Reg rj, LoongArch64Reg rk); // Ud5JK

    // Bound Check Memory Access Instructions
    void LDGT_B(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void LDGT_H(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void LDGT_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void LDGT_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void LDLE_B(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void LDLE_H(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void LDLE_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void LDLE_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void STGT_B(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void STGT_H(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void STGT_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void STGT_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void STLE_B(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void STLE_H(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void STLE_W(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK
    void STLE_D(LoongArch64Reg rd, LoongArch64Reg rj, LoongArch64Reg rk); // DJK

    // Atomic Memory Access Instructions
    void AMSWAP_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMSWAP_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMSWAP_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMSWAP_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ

    void AMADD_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMADD_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMADD_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMADD_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ

    void AMAND_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMAND_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMAND_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMAND_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ

    void AMOR_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMOR_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMOR_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMOR_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ

    void AMXOR_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMXOR_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMXOR_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMXOR_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ

    void AMMAX_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMMAX_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMMAX_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMMAX_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ

    void AMMIN_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMMIN_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMMIN_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMMIN_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ

    void AMMAX_WU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMMAX_DB_WU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMMAX_DU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMMAX_DB_DU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ

    void AMMIN_WU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMMIN_DB_WU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMMIN_DU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMMIN_DB_DU(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ

    void AMSWAP_B(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMSWAP_DB_B(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMSWAP_H(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMSWAP_DB_H(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ

    void AMADD_B(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMADD_DB_B(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMADD_H(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMADD_DB_H(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ

    void AMCAS_B(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMCAS_DB_B(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMCAS_H(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMCAS_DB_H(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMCAS_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMCAS_DB_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMCAS_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void AMCAS_DB_D(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ

    // CRC Check Instructions
    void CRC_W_B_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void CRC_W_H_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void CRC_W_W_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void CRC_W_D_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void CRCC_W_B_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void CRCC_W_H_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void CRCC_W_W_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ
    void CRCC_W_D_W(LoongArch64Reg rd, LoongArch64Reg rk, LoongArch64Reg rj); // DKJ

    // Other Miscellaneous Instructions
    void SYSCALL(u16 code); // Ud15
    void BREAK(u16 code); // Ud15

    void ASRTLE_D(LoongArch64Reg rj, LoongArch64Reg rk); // JK
    void ASRTGT_D(LoongArch64Reg rj, LoongArch64Reg rk); // JK

    void RDTIMEL_W(LoongArch64Reg rd, LoongArch64Reg rj); // DJ
    void RDTIMEH_W(LoongArch64Reg rd, LoongArch64Reg rj); // DJ
    void RDTIME_D(LoongArch64Reg rd, LoongArch64Reg rj); // DJ

    void CPUCFG(LoongArch64Reg rd, LoongArch64Reg rj); // DJ

    // Basic Floating-Point Instructions
    // Floating-Point Arithmetic Operation Instructions
    void FADD_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FADD_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FSUB_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FSUB_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FMUL_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FMUL_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FDIV_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FDIV_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk

    void FMADD_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa); // FdFjFkFa
    void FMADD_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa); // FdFjFkFa
    void FMSUB_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa); // FdFjFkFa
    void FMSUB_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa); // FdFjFkFa
    void FNMADD_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa); // FdFjFkFa
    void FNMADD_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa); // FdFjFkFa
    void FNMSUB_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa); // FdFjFkFa
    void FNMSUB_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Reg fa); // FdFjFkFa

    void FMAX_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FMAX_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FMIN_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FMIN_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk

    void FMAXA_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FMAXA_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FMINA_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FMINA_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk

    void FABS_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FABS_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FNEG_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FNEG_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj

    void FSQRT_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FSQRT_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FRECIP_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FRECIP_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FRSQRT_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FRSQRT_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj

    void FSCALEB_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FSCALEB_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FLOGB_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FLOGB_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FCOPYSIGN_S(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk
    void FCOPYSIGN_D(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk); // FdFjFk

    void FCLASS_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FCLASS_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj

    void FRECIPE_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FRECIPE_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FRSQRTE_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FRSQRTE_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj

    void FCMP_COND_S(LoongArch64CFR cd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Fcond cond); // CdFjFkFcond
    void FCMP_COND_D(LoongArch64CFR cd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64Fcond cond); // CdFjFkFcond

    // Floating-Point Conversion Instructions
    void FCVT_S_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FCVT_D_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj

    void FFINT_S_W(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FFINT_S_L(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FFINT_D_W(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FFINT_D_L(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINT_W_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINT_W_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINT_L_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINT_L_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj

    void FTINTRM_W_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRM_W_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRM_L_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRM_L_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRP_W_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRP_W_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRP_L_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRP_L_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRZ_W_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRZ_W_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRZ_L_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRZ_L_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRNE_W_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRNE_W_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRNE_L_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FTINTRNE_L_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj

    void FRINT_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FRINT_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj

    // Floating-Point Move Instructions
    void FMOV_S(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj
    void FMOV_D(LoongArch64Reg fd, LoongArch64Reg fj); // FdFj

    void FSEL(LoongArch64Reg fd, LoongArch64Reg fj, LoongArch64Reg fk, LoongArch64CFR ca); // FdFjFkCa

    void MOVGR2FR_W(LoongArch64Reg fd, LoongArch64Reg rj); // FdJ
    void MOVGR2FR_D(LoongArch64Reg fd, LoongArch64Reg rj); // FdJ
    void MOVGR2FRH_W(LoongArch64Reg fd, LoongArch64Reg rj); // FdJ

    void MOVFR2GR_S(LoongArch64Reg rj, LoongArch64Reg fd); // DFj
    void MOVFR2GR_D(LoongArch64Reg rj, LoongArch64Reg fd); // DFj
    void MOVFRH2GR_S(LoongArch64Reg rj, LoongArch64Reg fd); // DFj

    void MOVGR2FCSR(LoongArch64FCSR fcsr, LoongArch64Reg rj); // JUd5
    void MOVFCSR2GR(LoongArch64Reg rd, LoongArch64FCSR fcsr); // DUj5

    void MOVFR2CF(LoongArch64CFR cd, LoongArch64Reg fj); // CdFj
    void MOVCF2FR(LoongArch64Reg fd, LoongArch64CFR cj); // FdCj

    void MOVGR2CF(LoongArch64CFR cd, LoongArch64Reg rj); // CdJ
    void MOVCF2GR(LoongArch64Reg rd, LoongArch64CFR cj); // DCj

    // Floating-Point Branch Instructions
    void BCEQZ(LoongArch64CFR cj, s32 offs21); // CjSd5k16ps2
    void BCNEZ(LoongArch64CFR cj, s32 offs21); // CjSd5k16ps2

    // Floating-Point Common Memory Access Instructions
    void FLD_S(LoongArch64Reg fd, LoongArch64Reg rj, s16 si12); // FdJSk12
    void FLD_D(LoongArch64Reg fd, LoongArch64Reg rj, s16 si12); // FdJSk12
    void FST_S(LoongArch64Reg fd, LoongArch64Reg rj, s16 si12); // FdJSk12
    void FST_D(LoongArch64Reg fd, LoongArch64Reg rj, s16 si12); // FdJSk12

    void FLDX_S(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk); // FdJK
    void FLDX_D(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk); // FdJK
    void FSTX_S(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk); // FdJK
    void FSTX_D(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk); // FdJK

    // Floating-Point Bound Check Memory Access Instructions
    void FLDGT_S(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk); // FdJK
    void FLDGT_D(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk); // FdJK
    void FLDLE_S(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk); // FdJK
    void FLDLE_D(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk); // FdJK
    void FSTGT_S(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk); // FdJK
    void FSTGT_D(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk); // FdJK
    void FSTLE_S(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk); // FdJK
    void FSTLE_D(LoongArch64Reg fd, LoongArch64Reg rj, LoongArch64Reg rk); // FdJK

private:
    void SetJumpTarget(FixupBranch &branch, const void *dst);
	bool BranchInRange(const void *src, const void *dst) const;
	bool JumpInRange(const void *src, const void *dst) const;
    bool BranchZeroInRange(const void *src, const void *dst) const;

    void SetRegToImmediate(LoongArch64Reg rd, uint64_t value);

	template <typename T, bool extend>
	uint64_t AsImmediate(const T &v) {
		static_assert(std::is_trivial<T>::value, "Immediate argument must be a simple type");
		static_assert(sizeof(T) <= 8, "Immediate argument size should be 8, 16, 32, or 64 bits");

		// Copy the type to allow floats and avoid endian issues.
		if (sizeof(T) == 8) {
			uint64_t value;
			memcpy(&value, &v, sizeof(value));
			return value;
		} else if (sizeof(T) == 4) {
			uint32_t value;
			memcpy(&value, &v, sizeof(value));
			if (extend)
				return (int64_t)(int32_t)value;
			return value;
		} else if (sizeof(T) == 2) {
			uint16_t value;
			memcpy(&value, &v, sizeof(value));
			if (extend)
				return (int64_t)(int16_t)value;
			return value;
		} else if (sizeof(T) == 1) {
			uint8_t value;
			memcpy(&value, &v, sizeof(value));
			if (extend)
				return (int64_t)(int8_t)value;
			return value;
		}
		return (uint64_t)v;
	}

	inline void Write32(u32 value) {
        *(u32 *)writable_ = value;
		code_ += 4;
		writable_ += 4;
	}

protected:
	const u8 *code_ = nullptr;
	u8 *writable_ = nullptr;
    const u8 *lastCacheFlushEnd_ = nullptr;
};

class LoongArch64CodeBlock : public CodeBlock<LoongArch64Emitter> {
private:
    void PoisonMemory(int offset) override;
};

};
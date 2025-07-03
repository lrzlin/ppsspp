// Copyright (c) 2023- PPSSPP Project.

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

#include "Core/MemMap.h"
#include "Core/MIPS/LoongArch64/LoongArch64Jit.h"
#include "Core/MIPS/LoongArch64/LoongArch64RegCache.h"

// This file contains compilation for floating point related instructions.
//
// All functions should have CONDITIONAL_DISABLE, so we can narrow things down to a file quickly.
// Currently known non working ones should have DISABLE.  No flags because that's in IR already.

// #define CONDITIONAL_DISABLE { CompIR_Generic(inst); return; }
#define CONDITIONAL_DISABLE {}
#define DISABLE { CompIR_Generic(inst); return; }
#define INVALIDOP { _assert_msg_(false, "Invalid IR inst %d", (int)inst.op); CompIR_Generic(inst); return; }

namespace MIPSComp {

using namespace LoongArch64Gen;
using namespace LoongArch64JitConstants;

void LoongArch64JitBackend::CompIR_FArith(IRInst inst) {
	CONDITIONAL_DISABLE;

	switch (inst.op) {
	case IROp::FAdd:
		regs_.Map(inst);
		FADD_S(regs_.F(inst.dest), regs_.F(inst.src1), regs_.F(inst.src2));
		break;

	case IROp::FSub:
		regs_.Map(inst);
		FSUB_S(regs_.F(inst.dest), regs_.F(inst.src1), regs_.F(inst.src2));
		break;

	case IROp::FMul:
		regs_.Map(inst);
		// We'll assume everyone will make it such that 0 * infinity = NAN properly.
		// See blame on this comment if that proves untrue.
		FMUL_S(regs_.F(inst.dest), regs_.F(inst.src1), regs_.F(inst.src2));
		break;

	case IROp::FDiv:
		regs_.Map(inst);
		FDIV_S(regs_.F(inst.dest), regs_.F(inst.src1), regs_.F(inst.src2));
		break;

	case IROp::FSqrt:
		regs_.Map(inst);
		FSQRT_S(regs_.F(inst.dest), regs_.F(inst.src1));
		break;

	case IROp::FNeg:
		regs_.Map(inst);
		FNEG_S(regs_.F(inst.dest), regs_.F(inst.src1));
		break;

	default:
		INVALIDOP;
		break;
	}
}

void LoongArch64JitBackend::CompIR_FCondAssign(IRInst inst) {
	CONDITIONAL_DISABLE;

	regs_.Map(inst);
	FCMP_COND_S(FCC0, regs_.F(inst.src1), regs_.F(inst.src2), LoongArch64Fcond::CUN);
	MOVCF2GR(SCRATCH1, FCC0);
	FixupBranch unordered = BNEZ(SCRATCH1);
	
	switch (inst.op) {
	case IROp::FMin:
		FMIN_S(regs_.F(inst.dest), regs_.F(inst.src1), regs_.F(inst.src2));
		break;

	case IROp::FMax:
		FMAX_S(regs_.F(inst.dest), regs_.F(inst.src1), regs_.F(inst.src2));
		break;

	default:
		INVALIDOP;
		break;
	}

	FixupBranch ordererDone = B();
	SetJumpTarget(unordered);

	MOVFR2GR_S(SCRATCH1, regs_.F(inst.src1));
	MOVFR2GR_S(SCRATCH2, regs_.F(inst.src2));

	// If both are negative, we flip the comparison (not two's compliment.)
	// We cheat and use RA...
	AND(R_RA, SCRATCH1, SCRATCH2);
	SRLI_W(R_RA, R_RA, 31);

	LoongArch64Reg isSrc1LowerReg = regs_.GetAndLockTempGPR();
	SLT(isSrc1LowerReg, SCRATCH1, SCRATCH2);
	// Flip the flag (to reverse the min/max) based on if both were negative.
	XOR(isSrc1LowerReg, isSrc1LowerReg, R_RA);
	FixupBranch useSrc1;
	switch (inst.op) {
	case IROp::FMin:
		useSrc1 = BNEZ(isSrc1LowerReg);
		break;

	case IROp::FMax:
		useSrc1 = BEQZ(isSrc1LowerReg);
		break;

	default:
		INVALIDOP;
		break;
	}		
	MOVE(SCRATCH1, SCRATCH2);
	SetJumpTarget(useSrc1);

	MOVGR2FR_W(regs_.F(inst.dest), SCRATCH1);

	SetJumpTarget(ordererDone);
}

void LoongArch64JitBackend::CompIR_FAssign(IRInst inst) {
	CONDITIONAL_DISABLE;

	switch (inst.op) {
	case IROp::FMov:
		if (inst.dest != inst.src1) {
			regs_.Map(inst);
			FMOV_S(regs_.F(inst.dest), regs_.F(inst.src1));
		}
		break;

	case IROp::FAbs:
		regs_.Map(inst);
		FABS_S(regs_.F(inst.dest), regs_.F(inst.src1));
		break;

	case IROp::FSign:
	{
		regs_.Map(inst);
		// Check if it's negative zero, either 0x20/0x200 is zero.
		FCLASS_S(SCRATCHF1, regs_.F(inst.src1));
		MOVFR2GR_S(SCRATCH1, SCRATCHF1);
		ANDI(SCRATCH1, SCRATCH1, 0x220);
		SLTUI(SCRATCH1, SCRATCH1, 1);
		// Okay, it's zero if zero, 1 otherwise.  Convert 1 to a constant 1.0.
		// Probably non-zero is the common case, so we make that the straight line.
		FixupBranch skipOne = BEQZ(SCRATCH1);
		LI(SCRATCH1, 1.0f);

		// Now we just need the sign from it.
		MOVFR2GR_S(SCRATCH2, regs_.F(inst.src1));
		// Use a wall to isolate the sign, and combine.
		SRAI_W(SCRATCH2, SCRATCH2, 31);
		SLLI_W(SCRATCH2, SCRATCH2, 31);
		OR(SCRATCH1, SCRATCH1, SCRATCH2);

		SetJumpTarget(skipOne);
		MOVGR2FR_W(regs_.F(inst.dest), SCRATCH1);
		break;
	}

	default:
		INVALIDOP;
		break;
	}
}

void LoongArch64JitBackend::CompIR_FRound(IRInst inst) {
	CONDITIONAL_DISABLE;

	switch (inst.op) {
	case IROp::FRound:
	case IROp::FTrunc:
	case IROp::FCeil:
	case IROp::FFloor:
		CompIR_Generic(inst);
		break;

	default:
		INVALIDOP;
		break;
	}
}

void LoongArch64JitBackend::CompIR_FCvt(IRInst inst) {
	CONDITIONAL_DISABLE;

	switch (inst.op) {
	case IROp::FCvtWS:
	case IROp::FCvtSW:
	case IROp::FCvtScaledWS:
	case IROp::FCvtScaledSW:
		CompIR_Generic(inst);
		break;

	default:
		INVALIDOP;
		break;
	}
}

void LoongArch64JitBackend::CompIR_FSat(IRInst inst) {
	CONDITIONAL_DISABLE;

	switch (inst.op) {
	case IROp::FSat0_1:
	case IROp::FSatMinus1_1:
		CompIR_Generic(inst);
		break;

	default:
		INVALIDOP;
		break;
	}
}

void LoongArch64JitBackend::CompIR_FCompare(IRInst inst) {
	CONDITIONAL_DISABLE;

	switch (inst.op) {
	case IROp::FCmp:
	case IROp::FCmovVfpuCC:
	case IROp::FCmpVfpuBit:
	case IROp::FCmpVfpuAggregate:
		CompIR_Generic(inst);
		break;

	default:
		INVALIDOP;
		break;
	}
}

void LoongArch64JitBackend::CompIR_RoundingMode(IRInst inst) {
	CONDITIONAL_DISABLE;

	switch (inst.op) {
	case IROp::RestoreRoundingMode:
	case IROp::ApplyRoundingMode:
	case IROp::UpdateRoundingMode:
		CompIR_Generic(inst);
		break;

	default:
		INVALIDOP;
		break;
	}
}

void LoongArch64JitBackend::CompIR_FSpecial(IRInst inst) {
	CONDITIONAL_DISABLE;

	switch (inst.op) {
	case IROp::FSin:
	case IROp::FCos:
	case IROp::FRSqrt:
	case IROp::FRecip:
	case IROp::FAsin:
		CompIR_Generic(inst);
		break;

	default:
		INVALIDOP;
		break;
	}
}

} // namespace MIPSComp
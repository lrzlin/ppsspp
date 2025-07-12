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
#if PPSSPP_ARCH(LOONGARCH64)

#include "Common/CPUDetect.h"
#include "Common/Log.h"
#include "Core/Config.h"
#include "Common/LoongArch64Emitter.h"
#include "Core/MIPS/JitCommon/JitCommon.h"
#include "GPU/GPUState.h"
#include "GPU/Common/VertexDecoderCommon.h"

using namespace LoongArch64Gen;

static const LoongArch64Reg srcReg = R4;        // a0
static const LoongArch64Reg dstReg = R5;        // a1
static const LoongArch64Reg conterReg = R6;     // a2
static const LoongArch64Reg uvScaleReg = R7;    // a3
static const LoongArch64Reg tempReg1 = R8;      // a4
static const LoongArch64Reg tempReg2 = R9;      // a5
static const LoongArch64Reg tempReg3 = R10;     // a6
static const LoongArch64Reg scratchReg = R11;   // a7

static const LoongArch64Reg morphBaseReg = R12; // t0

static const LoongArch64Reg fullAlphaReg = R13; // t1
static const LoongArch64Reg boundsMinUReg = R17;
static const LoongArch64Reg boundsMinVReg = R18;
static const LoongArch64Reg boundsMaxUReg = R19;
static const LoongArch64Reg boundsMaxVReg = R20;

static const LoongArch64Reg fpScratchReg = F4;
static const LoongArch64Reg fpScratchReg2 = F5;
static const LoongArch64Reg fpScratchReg3 = F6;
static const LoongArch64Reg fpScratchReg4 = F7;

static const LoongArch64Reg lsxScratchReg = V2;
static const LoongArch64Reg lsxScratchReg2 = V3;

static const LoongArch64Reg lsxScaleOffsetReg = V0;
static const LoongArch64Reg lsxOffsetScaleReg = V0;

JittedVertexDecoder VertexDecoderJitCache::Compile(const VertexDecoder &dec, int32_t *jittedSize) {
	dec_ = &dec;

	BeginWrite(4096);
	const u8 *start = AlignCode16();

	bool log = false;
	bool prescaleStep = false;
    bool updateTexBounds = false;
	bool posThroughStep = false;

	// Look for prescaled texcoord steps
	for (int i = 0; i < dec.numSteps_; i++) {
		if (dec.steps_[i] == &VertexDecoder::Step_TcU8Prescale ||
			dec.steps_[i] == &VertexDecoder::Step_TcU16Prescale ||
			dec.steps_[i] == &VertexDecoder::Step_TcFloatPrescale) {
			prescaleStep = true;
		}
		if (dec.steps_[i] == &VertexDecoder::Step_TcU8PrescaleMorph ||
			dec.steps_[i] == &VertexDecoder::Step_TcU16PrescaleMorph ||
			dec.steps_[i] == &VertexDecoder::Step_TcFloatPrescaleMorph) {
			prescaleStep = true;
		}
        if (dec.steps_[i] == &VertexDecoder::Step_TcU16ThroughToFloat) {
			updateTexBounds = true;
		}
	}

    // We do not use any registers need to be saved now

    // Keep the scale/offset in a few fp registers if we need it.
	if (prescaleStep) {
		VLD(lsxScaleOffsetReg, R7, 0);
        VSHUF4I_D(lsxOffsetScaleReg, lsxScaleOffsetReg, 1);
	}
	

	RET();

	FlushIcache();

	if (log) {
		char temp[1024]{};
		dec.ToString(temp, true);
		INFO_LOG(Log::JIT, "=== %s (%d bytes) ===", temp, (int)(GetCodePtr() - start));
		std::vector<std::string> lines = DisassembleLA64(start, (int)(GetCodePtr() - start));
		for (auto line : lines) {
			INFO_LOG(Log::JIT, "%s", line.c_str());
		}
		INFO_LOG(Log::JIT, "==========");
	}

	*jittedSize = (int)(GetCodePtr() - start);
	EndWrite();
	return (JittedVertexDecoder)start;
}

#endif // PPSSPP_ARCH(LOONGARCH64)
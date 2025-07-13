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

alignas(16) static float bones[16 * 8];

static const float by128 = 1.0f / 128.0f;
static const float by32768 = 1.0f / 32768.0f;
static const float const65535 = 65535.0f;

using namespace LoongArch64Gen;

static const LoongArch64Reg srcReg = R4;        // a0
static const LoongArch64Reg dstReg = R5;        // a1
static const LoongArch64Reg counterReg = R6;     // a2
static const LoongArch64Reg uvScaleReg = R7;    // a3 - Unused
static const LoongArch64Reg tempReg1 = R7;      // a3 
static const LoongArch64Reg tempReg2 = R8;      // a4
static const LoongArch64Reg tempReg3 = R9;     // a5
static const LoongArch64Reg scratchReg = R10;   // a6

static const LoongArch64Reg morphBaseReg = R12; // t0

static const LoongArch64Reg fullAlphaReg = R11; // a7
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

static const LoongArch64Reg srcLSX = V8;
static const LoongArch64Reg accLSX = V9;

static const LoongArch64Reg by128LSX = V8;
static const LoongArch64Reg by32768LSX = V9;

static const LoongArch64Reg lsxWeightRegs[2] = { V12, V13 };

// We need to save these fregs when using them. (for example, skinning)
static constexpr LoongArch64Reg regs_to_save_fp[]{ F24, F25, F26, F27, F28, F29, F30, F31 };

// Similar to ARM64
// V4-V7 is the generated matrix that we multiply things by.
// V8, V9 are accumulators/scratch for matrix mul.
// V10, V11 are more scratch for matrix mul.
// V12, V13 are weight regs.
// V14, V15 are by128 and by32768 regs.
// V16+ are free-for-all for matrices. In 16 registers, we can fit 4 4x4 matrices.

static const JitLookup jitLookup[] = {
	{&VertexDecoder::Step_WeightsU8, &VertexDecoderJitCache::Jit_WeightsU8},
	{&VertexDecoder::Step_WeightsU16, &VertexDecoderJitCache::Jit_WeightsU16},
	{&VertexDecoder::Step_WeightsFloat, &VertexDecoderJitCache::Jit_WeightsFloat},
	{&VertexDecoder::Step_WeightsU8Skin, &VertexDecoderJitCache::Jit_WeightsU8Skin},
	{&VertexDecoder::Step_WeightsU16Skin, &VertexDecoderJitCache::Jit_WeightsU16Skin},
	{&VertexDecoder::Step_WeightsFloatSkin, &VertexDecoderJitCache::Jit_WeightsFloatSkin},

	{&VertexDecoder::Step_TcFloat, &VertexDecoderJitCache::Jit_TcFloat},
	{&VertexDecoder::Step_TcU8ToFloat, &VertexDecoderJitCache::Jit_TcU8ToFloat},
	{&VertexDecoder::Step_TcU16ToFloat, &VertexDecoderJitCache::Jit_TcU16ToFloat},

	{&VertexDecoder::Step_TcU8Prescale, &VertexDecoderJitCache::Jit_TcU8Prescale},
	{&VertexDecoder::Step_TcU16Prescale, &VertexDecoderJitCache::Jit_TcU16Prescale},
	{&VertexDecoder::Step_TcFloatPrescale, &VertexDecoderJitCache::Jit_TcFloatPrescale},

	{&VertexDecoder::Step_TcFloatThrough, &VertexDecoderJitCache::Jit_TcFloatThrough},
	{&VertexDecoder::Step_TcU16ThroughToFloat, &VertexDecoderJitCache::Jit_TcU16ThroughToFloat},

	{&VertexDecoder::Step_NormalS8, &VertexDecoderJitCache::Jit_NormalS8},
	{&VertexDecoder::Step_NormalS16, &VertexDecoderJitCache::Jit_NormalS16},
	{&VertexDecoder::Step_NormalFloat, &VertexDecoderJitCache::Jit_NormalFloat},

	{&VertexDecoder::Step_NormalS8Skin, &VertexDecoderJitCache::Jit_NormalS8Skin},
	{&VertexDecoder::Step_NormalS16Skin, &VertexDecoderJitCache::Jit_NormalS16Skin},
	{&VertexDecoder::Step_NormalFloatSkin, &VertexDecoderJitCache::Jit_NormalFloatSkin},

	{&VertexDecoder::Step_Color8888, &VertexDecoderJitCache::Jit_Color8888},
	{&VertexDecoder::Step_Color4444, &VertexDecoderJitCache::Jit_Color4444},
	{&VertexDecoder::Step_Color565, &VertexDecoderJitCache::Jit_Color565},
	{&VertexDecoder::Step_Color5551, &VertexDecoderJitCache::Jit_Color5551},

	{&VertexDecoder::Step_PosS8Through, &VertexDecoderJitCache::Jit_PosS8Through},
	{&VertexDecoder::Step_PosS16Through, &VertexDecoderJitCache::Jit_PosS16Through},
	{&VertexDecoder::Step_PosFloatThrough, &VertexDecoderJitCache::Jit_PosFloatThrough},

	{&VertexDecoder::Step_PosS8, &VertexDecoderJitCache::Jit_PosS8},
	{&VertexDecoder::Step_PosS16, &VertexDecoderJitCache::Jit_PosS16},
	{&VertexDecoder::Step_PosFloat, &VertexDecoderJitCache::Jit_PosFloat},

	{&VertexDecoder::Step_PosS8Skin, &VertexDecoderJitCache::Jit_PosS8Skin},
	{&VertexDecoder::Step_PosS16Skin, &VertexDecoderJitCache::Jit_PosS16Skin},
	{&VertexDecoder::Step_PosFloatSkin, &VertexDecoderJitCache::Jit_PosFloatSkin},

	/*
	{&VertexDecoder::Step_NormalS8Morph, &VertexDecoderJitCache::Jit_NormalS8Morph},
	{&VertexDecoder::Step_NormalS16Morph, &VertexDecoderJitCache::Jit_NormalS16Morph},
	{&VertexDecoder::Step_NormalFloatMorph, &VertexDecoderJitCache::Jit_NormalFloatMorph},

	{&VertexDecoder::Step_PosS8Morph, &VertexDecoderJitCache::Jit_PosS8Morph},
	{&VertexDecoder::Step_PosS16Morph, &VertexDecoderJitCache::Jit_PosS16Morph},
	{&VertexDecoder::Step_PosFloatMorph, &VertexDecoderJitCache::Jit_PosFloatMorph},

	{&VertexDecoder::Step_Color8888Morph, &VertexDecoderJitCache::Jit_Color8888Morph},
	{&VertexDecoder::Step_Color4444Morph, &VertexDecoderJitCache::Jit_Color4444Morph},
	{&VertexDecoder::Step_Color565Morph, &VertexDecoderJitCache::Jit_Color565Morph},
	{&VertexDecoder::Step_Color5551Morph, &VertexDecoderJitCache::Jit_Color5551Morph},
	*/
};

JittedVertexDecoder VertexDecoderJitCache::Compile(const VertexDecoder &dec, int32_t *jittedSize) {
	dec_ = &dec;

	BeginWrite(4096);
	const u8 *start = AlignCode16();

    int saveSize = (64 / 8) * (int)ARRAY_SIZE(regs_to_save_fp);
    int saveOffset = 0;

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

    QuickFLI(32, by128LSX, by128, scratchReg);
	QuickFLI(32, by32768LSX, by32768, scratchReg);
    VREPLVEI_W(by128LSX, by128LSX, 0);
    VREPLVEI_W(by32768LSX, by32768LSX, 0);

    // We need to save callee saved fregs when skinning
    if (dec.skinInDecode) {
        if (saveSize & 0xF)
		    saveSize += 8;
        _assert_msg_((saveSize & 0xF) == 0, "Stack must be kept aligned");
        ADDI_D(R_SP, R_SP, -saveSize);
        for (LoongArch64Reg r : regs_to_save_fp) {
            FST_D(r, R_SP, saveOffset);
            saveOffset += 64 / 8;
        }
        _assert_(saveOffset <= saveSize);
    }
    
    // Keep the scale/offset in a few fp registers if we need it.
	if (prescaleStep) {
		VLD(lsxScaleOffsetReg, R7, 0);
        VSHUF4I_D(lsxOffsetScaleReg, lsxScaleOffsetReg, 1);
        // TODO: deal with by128 and more
        
        
        
	}
	
    // Add code to convert matrices to 4x4.
    // Later we might want to do this when the matrices are loaded instead.
    if (dec.skinInDecode) {
		// Copying from R7 to R8
		LI(R7, gstate.boneMatrix);
		// This is only used with more than 4 weights, and points to the first of them.
		if (dec.nweights > 4)
			LI(R8, &bones[16 * 4]);

		// Construct a mask to zero out the top lane with.
		VOR_V(V3 ,V3, V3);
		VORN_V(V4, V3, V3);
        VINSGR2VR_W(V3, LoongArch64Gen::R_ZERO, 3);
		
		for (int i = 0; i < dec.nweights; i++) {
			// This loads V4, V5, V6, V7 with 12 floats.
            // And sort those floats into 4 regs: ABCD EFGH IJKL -> ABC0 DEF0 GHI0 JKL0.
            // TODO: Is unaligned load worth it?
            VLD(V4, R7, 0);
            VLD(V5, R7, 12);
            VLD(V6, R7, 24);
            VLD(V7, R7,36);
            ADDI_D(R7, R7, 48);
            VINSGR2VR_W(V4, LoongArch64Gen::R_ZERO, 3);
            VINSGR2VR_W(V5, LoongArch64Gen::R_ZERO, 3);
            VINSGR2VR_W(V6, LoongArch64Gen::R_ZERO, 3);
            VINSGR2VR_W(V7, LoongArch64Gen::R_ZERO, 3);

			LoongArch64Reg matrixRow[4]{ V4, V5, V6, V7 };
			// First four matrices are in registers Q16+.
			if (i < 4) {
				for (int w = 0; w < 4; ++w)
					matrixRow[w] = (LoongArch64Reg)(V16 + i * 4 + w);
			}
			// Zero out the top lane of each one with the mask created above.
			VAND_V(matrixRow[0], V4, V3);
			VAND_V(matrixRow[1], V5, V3);
			VAND_V(matrixRow[2], V6, V3);
			VAND_V(matrixRow[3], V7, V3);

			if (i >= 4) {
                VST(matrixRow[0], R8, 0);
                VST(matrixRow[0], R8, 16);
                VST(matrixRow[0], R8, 32);
                VST(matrixRow[0], R8, 48);
            }
		}
	}

    if (dec.col) {
        // Or LB and skip the conditional?  This is probably cheaper.
        LI(fullAlphaReg, 0xFF);
    }

    if (updateTexBounds) {
        LI(tempReg1, &gstate_c.vertBounds.minU);
        LD_H(boundsMinUReg, tempReg1, offsetof(KnownVertexBounds, minU));
        LD_H(boundsMaxUReg, tempReg1, offsetof(KnownVertexBounds, maxU));
        LD_H(boundsMinVReg, tempReg1, offsetof(KnownVertexBounds, minV));
        LD_H(boundsMaxVReg, tempReg1, offsetof(KnownVertexBounds, maxV));
    }

    const u8 *loopStart = GetCodePtr();
    for (int i = 0; i < dec.numSteps_; i++) {
        if (!CompileStep(dec, i)) {
            EndWrite();
            // Reset the code ptr (effectively undoing what we generated) and return zero to indicate that we failed.
            ResetCodePtr(GetOffset(start));
            char temp[1024]{};
            dec.ToString(temp, true);
            ERROR_LOG(Log::G3D, "Could not compile vertex decoder, failed at step %d: %s", i, temp);
            return nullptr;
        }
    }

    ADDI_D(srcReg, srcReg, dec.VertexSize());
    ADDI_D(dstReg, dstReg, dec.decFmt.stride);
    ADDI_D(counterReg, counterReg, -1);
    BLT(R_ZERO, counterReg, loopStart);

    if (dec.col) {
        LI(tempReg1, &gstate_c.vertexFullAlpha);
        FixupBranch skip = BNEZ(fullAlphaReg);
        ST_B(fullAlphaReg, tempReg1, 0);
        SetJumpTarget(skip);
    }

    if (updateTexBounds) {
        LI(tempReg1, &gstate_c.vertBounds.minU);
        ST_H(boundsMinUReg, tempReg1, offsetof(KnownVertexBounds, minU));
        ST_H(boundsMaxUReg, tempReg1, offsetof(KnownVertexBounds, maxU));
        ST_H(boundsMinVReg, tempReg1, offsetof(KnownVertexBounds, minV));
        ST_H(boundsMaxVReg, tempReg1, offsetof(KnownVertexBounds, maxV));
    }

    if (dec.skinInDecode) {
        saveOffset = 0;
        for (LoongArch64Reg r : regs_to_save_fp) {
            FLD_D(r, R_SP, saveOffset);
            saveOffset += 64 / 8;
        }
        ADDI_D(R_SP, R_SP, saveSize);
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

bool VertexDecoderJitCache::CompileStep(const VertexDecoder &dec, int step) {
	// See if we find a matching JIT function.
	for (size_t i = 0; i < ARRAY_SIZE(jitLookup); i++) {
		if (dec.steps_[step] == jitLookup[i].func) {
			((*this).*jitLookup[i].jitFunc)();
			return true;
		}
	}
	return false;
}

void VertexDecoderJitCache::Jit_ApplyWeights() {
	// We construct a matrix in V4-V7
	if (dec_->nweights > 4) {
		LI(scratchReg, bones + 16 * 4);
	}
	for (int i = 0; i < dec_->nweights; i++) {
		switch (i) {
		case 0:
            VREPLVEI_W(lsxScratchReg, lsxWeightRegs[0], 0);
			VFMUL_S(V4, V16, lsxScratchReg);
			VFMUL_S(V5, V17, lsxScratchReg);
			VFMUL_S(V6, V18, lsxScratchReg);
			VFMUL_S(V7, V19, lsxScratchReg);
			break;
		case 1:
            VREPLVEI_W(lsxScratchReg, lsxWeightRegs[0], 1);
			VFMADD_S(V4, V20, lsxScratchReg, lsxScratchReg);
			VFMADD_S(V5, V21, lsxScratchReg, lsxScratchReg);
			VFMADD_S(V6, V22, lsxScratchReg, lsxScratchReg);
			VFMADD_S(V7, V23, lsxScratchReg, lsxScratchReg);
			break;
		case 2:
			VREPLVEI_W(lsxScratchReg, lsxWeightRegs[0], 2);
			VFMADD_S(V4, V24, lsxScratchReg, lsxScratchReg);
			VFMADD_S(V5, V25, lsxScratchReg, lsxScratchReg);
			VFMADD_S(V6, V26, lsxScratchReg, lsxScratchReg);
			VFMADD_S(V7, V27, lsxScratchReg, lsxScratchReg);
			break;
		case 3:
			VREPLVEI_W(lsxScratchReg, lsxWeightRegs[0], 3);
			VFMADD_S(V4, V28, lsxScratchReg, lsxScratchReg);
			VFMADD_S(V5, V29, lsxScratchReg, lsxScratchReg);
			VFMADD_S(V6, V30, lsxScratchReg, lsxScratchReg);
			VFMADD_S(V7, V31, lsxScratchReg, lsxScratchReg);
			break;
		default:
			// Matrices 4+ need to be loaded from memory.
			VLD(V8, scratchReg, 0);
            VLD(V9, scratchReg, 16);
            VLD(V10, scratchReg, 32);
            VLD(V11, scratchReg, 48);
            VREPLVEI_W(lsxScratchReg, lsxWeightRegs[i >> 2], i & 3);
			VFMADD_S(V4, V8, lsxScratchReg, lsxScratchReg);
			VFMADD_S(V5, V9, lsxScratchReg, lsxScratchReg);
			VFMADD_S(V6, V10, lsxScratchReg, lsxScratchReg);
			VFMADD_S(V7, V11, lsxScratchReg, lsxScratchReg);
			break;
		}
	}
}

void VertexDecoderJitCache::Jit_WeightsU8() {
	// Basic implementation - a byte at a time.
    // TODO: Could optimize with unaligned load/store
	int j;
	for (j = 0; j < dec_->nweights; j++) {
		LD_B(tempReg1, srcReg, dec_->weightoff + j);
		ST_B(tempReg1, dstReg, dec_->decFmt.w0off + j);
	}
	while (j & 3) {
		ST_B(R_ZERO, dstReg, dec_->decFmt.w0off + j);
		j++;
	}
}

void VertexDecoderJitCache::Jit_WeightsU16() {
	// Basic implementation - a short at a time.
    // TODO: Could optimize with unaligned load/store
	int j;
	for (j = 0; j < dec_->nweights; j++) {
		LD_H(tempReg1, srcReg, dec_->weightoff + j * 2);
		ST_H(tempReg1, dstReg, dec_->decFmt.w0off + j * 2);
	}
	while (j & 3) {
		ST_H(R_ZERO, dstReg, dec_->decFmt.w0off + j * 2);
		j++;
	}
}

void VertexDecoderJitCache::Jit_WeightsFloat() {
	int j;
	for (j = 0; j < dec_->nweights; j++) {
		LD_W(tempReg1, srcReg, dec_->weightoff + j * 4);
		ST_W(tempReg1, dstReg, dec_->decFmt.w0off + j * 4);
	}
	while (j & 3) {  // Zero additional weights rounding up to 4.
		ST_W(R_ZERO, dstReg, dec_->decFmt.w0off + j * 4);
		j++;
	}
}

void VertexDecoderJitCache::Jit_WeightsU8Skin() {
	// Weight is first so srcReg is correct.
	switch (dec_->nweights) {
	case 1: LD_BU(scratchReg, srcReg, 0); break;
	case 2: LD_HU(scratchReg, srcReg, 0); break;
	default:
		// For 3, we over read, for over 4, we read more later.
		LD_WU(scratchReg, srcReg, 0);
		break;
	}

    VINSGR2VR_D(lsxScratchReg, scratchReg, 0);
	VSLLWIL_HU_BU(lsxScratchReg, lsxScratchReg, 0);
	VSLLWIL_WU_HU(lsxScratchReg, lsxScratchReg, 0);
	VFFINT_S_WU(lsxWeightRegs[0], lsxScratchReg);
    VFMUL_S(lsxWeightRegs[0], lsxWeightRegs[0], by128LSX);

	if (dec_->nweights > 4) {
		switch (dec_->nweights) {
		case 5: LD_BU(scratchReg, srcReg, 4); break;
	    case 6: LD_HU(scratchReg, srcReg, 4); break;
		case 7:
		case 8:
			LD_WU(scratchReg, srcReg, 4);
			break;
		}
        VINSGR2VR_D(lsxScratchReg, scratchReg, 0);
		VSLLWIL_HU_BU(lsxScratchReg, lsxScratchReg, 0);
	    VSLLWIL_WU_HU(lsxScratchReg, lsxScratchReg, 0);
		VFFINT_S_WU(lsxWeightRegs[1], lsxScratchReg);
        VFMUL_S(lsxWeightRegs[1], lsxWeightRegs[1], by128LSX);
	}
	Jit_ApplyWeights();
}

void VertexDecoderJitCache::Jit_WeightsU16Skin() {
	switch (dec_->nweights) {
	case 1: LD_HU(scratchReg, srcReg, 0); break;
	case 2: LD_WU(scratchReg, srcReg, 0); break;
	default:
		// For 3, we over read, for over 4, we read more later.
		LD_D(scratchReg, srcReg, 0);
		break;
	}
    VINSGR2VR_D(lsxScratchReg, scratchReg, 0);
	VSLLWIL_WU_HU(lsxScratchReg, lsxScratchReg, 0);
	VFFINT_S_WU(lsxWeightRegs[0], lsxScratchReg);
    VFMUL_S(lsxWeightRegs[0], lsxWeightRegs[0], by32768LSX);

	if (dec_->nweights > 4) {
		switch (dec_->nweights) {
		case 5: LD_HU(scratchReg, srcReg, 0); break;
	    case 6: LD_WU(scratchReg, srcReg, 0); break;
		case 7:
		case 8:
			LD_D(scratchReg, srcReg, 0);
			break;
		}
		VINSGR2VR_D(lsxScratchReg, scratchReg, 0);
        VSLLWIL_WU_HU(lsxScratchReg, lsxScratchReg, 0);
        VFFINT_S_WU(lsxWeightRegs[1], lsxScratchReg);
        VFMUL_S(lsxWeightRegs[1], lsxWeightRegs[1], by32768LSX);
	}
	Jit_ApplyWeights();
}

#endif // PPSSPP_ARCH(LOONGARCH64)
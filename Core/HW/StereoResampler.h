// Copyright (c) 2015- PPSSPP Project and Dolphin Project.

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

// Adapted from Dolphin.

#pragma once

#include <cstdint>
#include <atomic>

#include "Common/CommonTypes.h"

struct AudioDebugStats;

class StereoResampler {
public:
	StereoResampler() noexcept;
	~StereoResampler();

	// Called from audio threads
	void Mix(s16 *samples, unsigned int numSamples, bool consider_framelimit, int sampleRate);

	// Called from main thread
	// This clamps the samples to 16-bit before starting to work on them.
	// Volume is a multiplier from 0.0f to 1.0f.
	void PushSamples(const s32* samples, unsigned int num_samples, float volume);

	void Clear();

	void GetAudioDebugStats(char *buf, size_t bufSize);
	void ResetStatCounters();

private:
	void UpdateBufferSize();

	int maxBufsize_;
	int targetBufsize_;

	// This can be adjusted, for the case of non-60hz output (a few hz off).
	int inputSampleRateHz_ = 44100;

	int16_t *buffer_;
	std::atomic<u32> indexW_;
	std::atomic<u32> indexR_;
	float numLeftI_ = 0.0f;

	u32 frac_ = 0;
	float outputSampleRateHz_ = 0.0;
	int lastBufSize_ = 0;
	int lastPushSize_ = 0;
	u32 ratio_ = 0;

	int underrunCount_ = 0;
	int overrunCount_ = 0;
	int underrunCountTotal_ = 0;
	int overrunCountTotal_ = 0;

	int droppedSamples_ = 0;

	int64_t inputSampleCount_ = 0;
	int64_t outputSampleCount_ = 0;

	double startTime_ = 0.0;
};

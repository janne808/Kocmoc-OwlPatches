/*
 *  (C) 2021 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Ladder Filter OWL Patch.
 *
 *  Ladder Filter OWL Patch is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Ladder Filter OWL Patch is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Ladder Filter OWL Patch.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __dspladderh__
#define __dspladderh__

#include "iir.h"

// filter modes
enum LadderFilterMode {
   LADDER_LOWPASS_MODE,
   LADDER_BANDPASS_MODE,
   LADDER_HIGHPASS_MODE
};

// integration methods
enum LadderIntegrationMethod {
   LADDER_EULER_FULL_TANH,
   LADDER_PREDICTOR_CORRECTOR_FULL_TANH,
   LADDER_PREDICTOR_CORRECTOR_FEEDBACK_TANH,
   LADDER_TRAPEZOIDAL_FEEDBACK_TANH
};

class Ladder{
public:
  // constructor/destructor
  Ladder(float newCutoff, float newResonance, int newOversamplingFactor,
      LadderFilterMode newFilterMode, float newSampleRate, LadderIntegrationMethod newIntegrationMethod);
  Ladder();
  ~Ladder();

  // set filter parameters
  void SetFilterCutoff(float newCutoff);
  void SetFilterResonance(float newResonance);
  void SetFilterOversamplingFactor(int newOversamplingFactor);
  void SetFilterMode(LadderFilterMode newFilterMode);
  void SetFilterSampleRate(float newSampleRate);
  void SetFilterIntegrationMethod(LadderIntegrationMethod method);
  
  // get filter parameters
  float GetFilterCutoff();
  float GetFilterResonance();
  int GetFilterOversamplingFactor();  
  LadderFilterMode GetFilterMode();  
  float GetFilterSampleRate();
  LadderIntegrationMethod GetFilterIntegrationMethod();
  
  // tick filter state
  void LadderFilter(float input);

  // get filter responses
  float GetFilterLowpass();
  float GetFilterBandpass();
  float GetFilterHighpass();

  // get filter output
  float GetFilterOutput();

  // reset state
  void ResetFilterState();

private:
  // set integration rate
  void SetFilterIntegrationRate();

  // filter parameters
  float cutoffFrequency;
  float Resonance;
  int oversamplingFactor;
  LadderFilterMode filterMode;
  float sampleRate;
  float dt;
  LadderIntegrationMethod integrationMethod;
  
  // filter state
  float p0, p1, p2, p3;
  float ut_1;
  
  // filter output
  float out;

  // IIR downsampling filter
  IIRLowpass *iir;
};

#endif

/*
 *  (C) 2021 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of State Variable Filter OWL Patch.
 *
 *  State Variable Filter OWL Patch is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  State Variable Filter OWL Patch is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with State Variable Filter OWL Patch.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __dspsvfh__
#define __dspsvfh__

#include "iir.h"

// filter modes
enum SVFFilterMode {
   SVF_LOWPASS_MODE,
   SVF_BANDPASS_MODE,
   SVF_HIGHPASS_MODE
};

// integration methods
enum SVFIntegrationMethod {
   SVF_SEMI_IMPLICIT_EULER,
   SVF_PREDICTOR_CORRECTOR,
   SVF_TRAPEZOIDAL,
   SVF_INV_TRAPEZOIDAL
};

class SVFilter{
public:
  // constructor/destructor
  SVFilter(float newCutoff, float newResonance, int newOversamplingFactor,
	   SVFFilterMode newFilterMode, float newSampleRate, SVFIntegrationMethod newIntegrationMethod);
  SVFilter();
  ~SVFilter();

  // set filter parameters
  void SetFilterCutoff(float newCutoff);
  void SetFilterResonance(float newResonance);
  void SetFilterOversamplingFactor(int newOversamplingFactor);
  void SetFilterMode(SVFFilterMode newFilterMode);
  void SetFilterSampleRate(float newSampleRate);
  void SetFilterIntegrationMethod(SVFIntegrationMethod method);
  
  // get filter parameters
  float GetFilterCutoff();
  float GetFilterResonance();
  int GetFilterOversamplingFactor();  
  SVFFilterMode GetFilterMode();  
  float GetFilterSampleRate();
  SVFIntegrationMethod GetFilterIntegrationMethod();
  
  // tick filter state
  void filter(float input);

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

  // pade approximant functions for hyperbolic functions
  // filter parameters
  float cutoffFrequency;
  float Resonance;
  int oversamplingFactor;
  SVFFilterMode filterMode;
  float sampleRate;
  float dt;
  SVFIntegrationMethod integrationMethod;
  
  // filter state
  float lp;
  float bp;
  float hp;
  float u_t1;
  
  // filter output
  float out;

  // IIR downsampling filter
  IIRLowpass *iir;
};

#endif

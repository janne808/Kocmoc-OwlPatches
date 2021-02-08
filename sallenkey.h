/*
 *  (C) 2021 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Sallen-Key Filter OWL Patch.
 *
 *  Sallen-Key Filter OWL Patch is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Sallen-Key Filter OWL Patch is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Sallen-Key Filter OWL Patch.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __dspskfh__
#define __dspskfh__

#include "iir.h"

// filter modes
enum SKFilterMode {
   SK_LOWPASS_MODE,
   SK_BANDPASS_MODE,
   SK_HIGHPASS_MODE
};

// integration methods
enum SKIntegrationMethod {
   SK_SEMI_IMPLICIT_EULER,
   SK_PREDICTOR_CORRECTOR,
   SK_TRAPEZOIDAL
};

class SKFilter{
public:
  // constructor/destructor
  SKFilter(float newCutoff, float newResonance, int newOversamplingFactor,
	   SKFilterMode newFilterMode, float newSampleRate, SKIntegrationMethod newIntegrationMethod);
  SKFilter();
  ~SKFilter();

  // set filter parameters
  void SetFilterCutoff(float newCutoff);
  void SetFilterResonance(float newResonance);
  void SetFilterOversamplingFactor(int newOversamplingFactor);
  void SetFilterMode(SKFilterMode newFilterMode);
  void SetFilterSampleRate(float newSampleRate);
  void SetFilterIntegrationMethod(SKIntegrationMethod method);
  
  // get filter parameters
  float GetFilterCutoff();
  float GetFilterResonance();
  int GetFilterOversamplingFactor();  
  SKFilterMode GetFilterMode();  
  float GetFilterSampleRate();
  SKIntegrationMethod GetFilterIntegrationMethod();
  
  // tick filter state
  void filter(float input);

  // set filter inputs
  void SetFilterLowpassInput(float input);
  void SetFilterBandpassInput(float input);
  void SetFilterHighpassInput(float input);

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
  SKFilterMode filterMode;
  float sampleRate;
  float dt;
  SKIntegrationMethod integrationMethod;
  
  // filter state
  float p0;
  float p1;

  // filter input
  float input_lp;
  float input_bp;
  float input_hp;
  float input_lp_t1;
  float input_bp_t1;
  float input_hp_t1;
  
  // filter output
  float out;

  // IIR downsampling filter
  IIRLowpass *iir;
};

#endif

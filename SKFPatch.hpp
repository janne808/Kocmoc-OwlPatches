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

#ifndef __SKFPatch_h__
#define __SKFPatch_h__

#include "StompBox.h"

#include "sallenkey.h"
#include "iir.h"

class SKFPatch : public Patch {
public:
  SKFilter skf;
  
  SKFPatch(){
    registerParameter(PARAMETER_A, "Cutoff");    
    registerParameter(PARAMETER_B, "Resonance");    
    registerParameter(PARAMETER_C, "Gain");    
    registerParameter(PARAMETER_D, "Mode");

    skf.SetFilterSampleRate(getSampleRate());
    skf.SetFilterOversamplingFactor(4);
    skf.SetFilterIntegrationMethod(SK_TRAPEZOIDAL);
    skf.SetFilterMode(SK_LOWPASS_MODE);
  }

  void processAudio(AudioBuffer &buffer){
    float cutoff = getParameterValue(PARAMETER_A);
    float reso = getParameterValue(PARAMETER_B);
    float gain = 1.f + 7.f*getParameterValue(PARAMETER_C);
    float mode = getParameterValue(PARAMETER_D);
    
    // shape parameters
    cutoff *= 2.5f*cutoff*cutoff;

    skf.SetFilterCutoff(cutoff);
    skf.SetFilterResonance(reso);

    if(mode >= 0.f && mode < 0.5f){
      skf.SetFilterMode(SK_LOWPASS_MODE);
    }
    else if(mode >= 0.5f && mode < 1.f){
      skf.SetFilterMode(SK_BANDPASS_MODE);
    }
    
    int size = buffer.getSize();
    
    float* buf = buffer.getSamples(0);
    for(int i=0; i<size; ++i){
      skf.filter(gain*buf[i]);
      buf[i] = 0.4f*skf.GetFilterOutput()/gain;
    }
  }
};

#endif // __SKFPatch_h__


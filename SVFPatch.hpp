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

#ifndef __SVFPatch_h__
#define __SVFPatch_h__

#include "StompBox.h"

#include "svfilter.h"
#include "iir.h"

class SVFPatch : public Patch {
public:
  SVFilter svf;
  
  SVFPatch(){
    registerParameter(PARAMETER_A, "Cutoff");    
    registerParameter(PARAMETER_B, "Resonance");    
    registerParameter(PARAMETER_C, "Gain");    
    registerParameter(PARAMETER_D, "Mode");

    svf.SetFilterSampleRate(getSampleRate());
    svf.SetFilterOversamplingFactor(4);
    svf.SetFilterIntegrationMethod(SVF_TRAPEZOIDAL);
    svf.SetFilterMode(SVF_LOWPASS_MODE);
  }

  void processAudio(AudioBuffer &buffer){
    float cutoff = getParameterValue(PARAMETER_A);
    float reso = getParameterValue(PARAMETER_B);
    float gain = 1.f + 7.f*getParameterValue(PARAMETER_C);
    float mode = getParameterValue(PARAMETER_D);
    
    // shape parameters
    cutoff *= 2.5f*cutoff*cutoff;

    svf.SetFilterCutoff(cutoff);
    svf.SetFilterResonance(reso);

    if(mode >= 0.f && mode < 0.33f){
      svf.SetFilterMode(SVF_LOWPASS_MODE);
    }
    else if(mode >= 0.33f && mode < 0.66f){
      svf.SetFilterMode(SVF_BANDPASS_MODE);
    }
    else if(mode >= 0.66f && mode < 1.f){
      svf.SetFilterMode(SVF_HIGHPASS_MODE);
    }
    
    int size = buffer.getSize();
    
    float* buf = buffer.getSamples(0);
    for(int i=0; i<size; ++i){
      svf.filter(gain*buf[i]);
      buf[i] = 0.4f*svf.GetFilterOutput()/gain;
    }
  }
};

#endif // __SVFPatch_h__


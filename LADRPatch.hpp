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

#ifndef __LADRPatch_h__
#define __LADRPatch_h__

#include "StompBox.h"

#include "ladder.h"
#include "iir.h"

class LADRPatch : public Patch {
public:
  Ladder ladder;
  
  LADRPatch(){
    registerParameter(PARAMETER_A, "Cutoff");    
    registerParameter(PARAMETER_B, "Resonance");    
    registerParameter(PARAMETER_C, "Gain");    
    registerParameter(PARAMETER_D, "Mode");

    ladder.SetFilterSampleRate(getSampleRate());
    ladder.SetFilterOversamplingFactor(4);
    ladder.SetFilterIntegrationMethod(LADDER_TRAPEZOIDAL_FEEDBACK_TANH);
    ladder.SetFilterMode(LADDER_LOWPASS_MODE);
  }

  void processAudio(AudioBuffer &buffer){
    float cutoff = getParameterValue(PARAMETER_A);
    float reso = getParameterValue(PARAMETER_B);
    float gain = 1.f + 7.f*getParameterValue(PARAMETER_C);
    float mode = getParameterValue(PARAMETER_D);
    
    // shape parameters
    cutoff *= 2.5f*cutoff*cutoff;

    ladder.SetFilterCutoff(cutoff);
    ladder.SetFilterResonance(reso);

    if(mode >= 0.f && mode < 0.33f){
      ladder.SetFilterMode(LADDER_LOWPASS_MODE);
    }
    else if(mode >= 0.33f && mode < 0.66f){
      ladder.SetFilterMode(LADDER_BANDPASS_MODE);
    }
    else if(mode >= 0.66f && mode < 1.f){
      ladder.SetFilterMode(LADDER_HIGHPASS_MODE);
    }
    
    int size = buffer.getSize();
    
    float* buf = buffer.getSamples(0);
    for(int i=0; i<size; ++i){
      ladder.LadderFilter(gain*buf[i]);
      buf[i] = 0.4f*ladder.GetFilterOutput()/gain;
    }
  }
};

#endif // __LADRPatch_h__


/*
 *  (C) 2022 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of DigiDelay OWL Patch.
 *
 *  DigiDelay OWL Patch is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  DigiDelay OWL Patch is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with DigiDelay OWL Patch.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __DigiDelayPatch_h__
#define __DigiDelayPatch_h__

#include "Patch.h"

#include "fastmath.h"

#define TIME_THRESHOLD 0.006
#define FADE_RATE 0.04

class DigiDelayPatch : public Patch {
public:
  float *ringBuffer;
  int bufferLength;
  int writePointer;
    
  int sampleRate;

  float time2;

  int fade_state;
  float fade_value;
  float fade0_time, fade1_time;

  float hp;
  
  DigiDelayPatch(){
    registerParameter(PARAMETER_A, "Time");    
    registerParameter(PARAMETER_B, "Feedback");    
    registerParameter(PARAMETER_C, "Gain");    
    registerParameter(PARAMETER_D, "Dry/Wet");

    sampleRate = getSampleRate();
    writePointer = 0;

    // 2 seconds of delay time
    bufferLength = 2*sampleRate;
    ringBuffer = new float[bufferLength];
    
    for(int i=0; i<2*bufferLength; i++){
      ringBuffer[i] = 0.f;
    }

    time2 = getParameterValue(PARAMETER_A);

    fade_state = 0;
    fade0_time = fade1_time = 0.f;

    hp = 0.f;
  }

  float readDelay(float time){
    int readPointer, readPointer2;
    float frac;
    
    readPointer = writePointer - (int)(time*bufferLength);
    if(readPointer < 0){
      readPointer += bufferLength;
    }

    readPointer2 = readPointer - 1;
    if(readPointer2 < 0){
      readPointer2 += bufferLength;
    }

    // simple fractional linear interpolation
    frac = (float)(time*bufferLength) - (float)((int)(time*bufferLength));
    return (1.f - frac)*ringBuffer[readPointer] + (frac)*ringBuffer[readPointer2];
  }

  void writeDelay(float input){
      writePointer += 1;
      if(writePointer > bufferLength - 1){
	writePointer -= bufferLength;
      }

      // dc blocking filter for write head
      float hp_input = input;
      hp += 0.00005f*(hp_input - hp);
      input = hp - hp_input;
      
      ringBuffer[writePointer] = input;
  }
  
  void processAudio(AudioBuffer &buffer){
    float time = getParameterValue(PARAMETER_A);
    float feedback = getParameterValue(PARAMETER_B);
    float gain = getParameterValue(PARAMETER_C);
    float drywet = getParameterValue(PARAMETER_D);

    int fade_trigger = 0;

    // lich cv has noisy inputs
    // add hysteresis threshold to time parameter value
    if(abs(time-time2) > TIME_THRESHOLD){
      time2 = time;

      // trigger crossfade
      if(fade_state){
	fade_state = 0;
	fade0_time = time2*time2*time2*time2;
      }
      else{
	fade_state = 1;
	fade1_time = time2*time2*time2*time2;	
      }
    }
    
    int size = buffer.getSize();
    
    float* io_buf = buffer.getSamples(0);
    float delay;
      
    for(int i=0; i<size; ++i){
      // update crossfade
      if(fade_state){
	fade_value += FADE_RATE;
	if(fade_value > 1.f){
	  fade_value = 1.f;
	}
      }
      else{
	fade_value -= FADE_RATE;
	if(fade_value < 0.f){
	  fade_value = 0.f;
	}
      }
      
      // read delayed signal
      delay = (1.f - fade_value)*readDelay(fade0_time) + fade_value*readDelay(fade1_time);

      // update buffer
      writeDelay(gain*io_buf[i] + feedback*delay);

      // output
      io_buf[i] = (1.f - drywet)*gain*io_buf[i] + drywet*delay;
    }
  }
};

#endif // __DigiDelayPatch_h__


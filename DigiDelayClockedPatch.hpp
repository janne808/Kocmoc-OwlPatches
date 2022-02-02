/*
 *  (C) 2022 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Clocked DigiDelay OWL Patch.
 *
 *  Clocked DigiDelay OWL Patch is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Clocked DigiDelay OWL Patch is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Clocked DigiDelay OWL Patch.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __DigiDelayClockedPatch_h__
#define __DigiDelayClockedPatch_h__

#include "Patch.h"

#include "fastmath.h"

#define TIME_THRESHOLD 0.006
#define FADE_RATE 0.04

class DigiDelayClockedPatch : public Patch {
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

  int last_clk;
  int clk_event_offset;
  int clk_event;
  int clk_counter;
  int clk_period;
  
  DigiDelayClockedPatch(){
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

    last_clk = 0;
    clk_event_offset = 0;
    clk_event = 0;
    clk_counter = 0;
    clk_period = 0;
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
  
  void buttonChanged(PatchButtonId bid, uint16_t value, uint16_t samples){
    bool set = value != 0;
    switch(bid){
    case BUTTON_A:
      // detect rising edge
      if(set && !last_clk){
	last_clk = 1;
	clk_event_offset = samples;
	clk_event = 1;
      }
      else if(!set && last_clk){
	last_clk = 0;
	clk_event_offset = samples;
	clk_event = 2;
      }
      else{
	clk_event = 0;
      }
      
      break;
    }
  }
  
  void processAudio(AudioBuffer &buffer){
    float time = getParameterValue(PARAMETER_A);
    float feedback = getParameterValue(PARAMETER_B);
    float gain = getParameterValue(PARAMETER_C);
    float drywet = getParameterValue(PARAMETER_D);

    int fade_trigger = 0;

    int size = buffer.getSize();    

    // clock rate division and multiplier table
    const float div_table[16] = {
      0.125f, 0.25f, 0.375f, 0.5f,
      0.625f, 0.75f, 0.875f, 1.f,
      1.125f, 1.25f, 1.375f, 1.5f,
      1.625f, 1.75f, 1.875f, 2.f
    };

    float clk_time;
    float ratio;

    ratio = 4.f*div_table[static_cast <int> (15.f*time)];
    
    clk_time = (static_cast <float> (clk_period))/(static_cast <float> (sampleRate));
    time = ratio*clk_time/(static_cast <float> (2));

    if(time > 0.995f){
      time = 0.995f;
    }

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
    
    float* io_buf = buffer.getSamples(0);
    float delay;

    for(int i=0; i<size; ++i){
      // clock period counter
      clk_counter++;
      
      // handle clock events
      // rising edge
      if(clk_event == 1 && i == clk_event_offset){
	clk_period = clk_counter;
	clk_counter = 0;
	clk_event = 0;
      }
      // falling edge
      else if(clk_event == 2 && i == clk_event_offset){
	clk_event = 0;
      }

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

#endif // __DigiDelayClockedPatch_h__


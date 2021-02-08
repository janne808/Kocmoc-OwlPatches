/*
 *  (C) 2021 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Infinite Impulse Response Filter OWL Patch.
 *
 *  Infinite Impulse Response Filter OWL Patch is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Infinite Impulse Response Filter OWL Patch is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Infinite Impulse Response Filter OWL Patch.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __dspiirh__
#define __dspiirh__

class IIRLowpass{
public:
  // constructor/destructor
  IIRLowpass(float newSamplerate, float newCutoff, int newOrder);
  IIRLowpass();
  ~IIRLowpass();

  // set filter parameters
  void SetFilterOrder(int newOrder);
  void SetFilterSamplerate(float newSamplerate);
  void SetFilterCutoff(float newCutoff);

  // initialize biquad cascade delayline
  void InitializeBiquadCascade();
  
  // IIR filter signal 
  float IIRfilter(float input);

  // get filter coefficients
  float* GetFilterCoeffA1();
  float* GetFilterCoeffA2();
  float* GetFilterCoeffK();
  
private:
  // compute biquad cascade coefficients
  void ComputeCoefficients();

  // filter design variables
  float samplerate;
  float cutoff;
  int order;
  
  // dsp variables
  float *a1;
  float *a2;
  float *K;
  float *pa_real;
  float *pa_imag;
  float *p_real;
  float *p_imag;
  
  // cascaded biquad buffers
  float *z;
};

#endif

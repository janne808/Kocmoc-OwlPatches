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

#include <cmath>
#include "iir.h"

// constructor
IIRLowpass::IIRLowpass(float newSamplerate, float newCutoff, int newOrder)
{
  // initialize filter design parameters
  samplerate = newSamplerate;
  cutoff = newCutoff;
  order = newOrder;

  // allocate dsp vectors
  a1 = new float[order/2];
  a2 = new float[order/2];
  K = new float[order/2];
  pa_real = new float[order/2];
  pa_imag = new float[order/2];
  p_real = new float[order/2];
  p_imag = new float[order/2];

  // allocate cascaded biquad buffer
  z = new float[order];
  
  // initialize cascade delayline
  InitializeBiquadCascade();
  
  // compute impulse response
  ComputeCoefficients();
}

// default constructor
IIRLowpass::IIRLowpass()
{
  // set default design parameters
  samplerate=(float)(44100.0);
  cutoff=(float)(440.0);
  order=32;
  
  // allocate dsp vectors
  a1 = new float[order/2];
  a2 = new float[order/2];
  K = new float[order/2];
  pa_real = new float[order/2];
  pa_imag = new float[order/2];
  p_real = new float[order/2];
  p_imag = new float[order/2];

  // allocate cascaded biquad buffer
  z = new float[order];
  
  // initialize cascade delayline
  InitializeBiquadCascade();
  
  // compute impulse response
  ComputeCoefficients();
}

// destructor
IIRLowpass::~IIRLowpass(){
  // free dsp vectors
  delete[] a1;
  delete[] a2;
  delete[] K;
  delete[] pa_real;
  delete[] pa_imag;
  delete[] p_real;
  delete[] p_imag;
  
  // free cascaded biquad buffer
  delete[] z;
}

void IIRLowpass::SetFilterOrder(int newOrder){
  order = newOrder;

  // free dsp vectors
  delete[] a1;
  delete[] a2;
  delete[] K;
  delete[] pa_real;
  delete[] pa_imag;
  delete[] p_real;
  delete[] p_imag;
    
  // free cascaded biquad buffer
  delete[] z;

  // allocate dsp vectors
  a1 = new float[order/2];
  a2 = new float[order/2];
  K = new float[order/2];
  pa_real = new float[order/2];
  pa_imag = new float[order/2];
  p_real = new float[order/2];
  p_imag = new float[order/2];

  // allocate cascaded biquad buffer
  z = new float[order];

  // initialize cascade delayline
  InitializeBiquadCascade();
  
  // compute new impulse response
  ComputeCoefficients();
}

void IIRLowpass::SetFilterSamplerate(float newSamplerate){
  samplerate = newSamplerate;

  // initialize cascade delayline
  InitializeBiquadCascade();
  
  // compute new cascade coefficients
  ComputeCoefficients();
}

void IIRLowpass::SetFilterCutoff(float newCutoff){
  cutoff = newCutoff;

  // initialize cascade delayline
  InitializeBiquadCascade();
  
  // compute new cascade coefficients
  ComputeCoefficients();
}

void IIRLowpass::InitializeBiquadCascade(){
  for(int ii=0; ii<order/2; ii++){
    z[ii*2+1] = 0.0;
    z[ii*2] = 0.0;
  }
}

float IIRLowpass::IIRfilter(float input){
  float out=input;
  float in;

  // process biquad cascade
  for(int ii=0; ii<order/2; ii++) {
    // compute biquad input
    in = K[ii]*out - a1[ii]*z[ii*2] - a2[ii]*z[ii*2+1];
      
    // compute biquad output
    out = in + 2.0*z[ii*2] + z[ii*2+1];
    
    // update delays
    z[ii*2+1] = z[ii*2];
    z[ii*2] = in;
  }
  
  return out;
}

float* IIRLowpass::GetFilterCoeffA1(){
  return a1;
}

float* IIRLowpass::GetFilterCoeffA2(){
  return a2;
}

float* IIRLowpass::GetFilterCoeffK(){
  return K;
}

void IIRLowpass::ComputeCoefficients(){
  // place butterworth style analog filter poles
  float theta;

  for(int ii = 0; ii<order/2; ii++) {
    int k = order/2 - ii;
    theta = (2.0*(float)(k) - 1.0)*M_PI/(2.0*(float)(order));
    
    pa_real[ii] = -1.0*sin(theta);
    pa_imag[ii] = cos(theta);
  }

  // prewarp and scale poles
  float Fc = samplerate/M_PI*tan(M_PI*cutoff/samplerate);  
  for(int ii = 0; ii<order/2; ii++) {
    pa_real[ii] *= 2.0*M_PI*Fc; 
    pa_imag[ii] *= 2.0*M_PI*Fc; 
  }

  // bilinear transform to z-plane
  for(int ii = 0; ii<order/2; ii++) {
    // complex division
    float u = (2.0*samplerate+pa_real[ii])/(2.0*samplerate); 
    float v = pa_imag[ii]/(2.0*samplerate); 
    float x = (2.0*samplerate-pa_real[ii])/(2.0*samplerate); 
    float y = -1.0*pa_imag[ii]/(2.0*samplerate);
    
    float c = 1.0/(x*x + y*y);
    
    p_real[ii] = c*(u*x + v*y);
    p_imag[ii] = c*(v*x - u*y);
  }
  
  // compute cascade coefficients
  for(int ii = 0; ii<order/2; ii++) {
    a1[ii] = -2.0*p_real[ii];
    a2[ii] = p_real[ii]*p_real[ii] + p_imag[ii]*p_imag[ii];
    K[ii] = (1.0 + a1[ii] + a2[ii])/4.0;
  }
}


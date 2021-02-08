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

#include <cstdlib>
#include <cmath>
#include "sallenkey.h"
#include "iir.h"
#include "fastmath.h"

// steepness of downsample filter response
#define IIR_DOWNSAMPLE_ORDER 8

// downsampling passthrough bandwidth
#define IIR_DOWNSAMPLING_BANDWIDTH 0.9

// constructor
SKFilter::SKFilter(float newCutoff, float newResonance, int newOversamplingFactor,
		   SKFilterMode newFilterMode, float newSampleRate, SKIntegrationMethod newIntegrationMethod){
  // initialize filter parameters
  cutoffFrequency = newCutoff;
  Resonance = newResonance;
  oversamplingFactor = newOversamplingFactor;
  filterMode = newFilterMode;
  sampleRate = newSampleRate;

  SetFilterIntegrationRate();

  // initialize filter state
  p0 = p1 = out = 0.0;

  // initialize filter inputs
  input_lp = input_bp = input_hp = 0.0;
  input_lp_t1 = input_bp_t1 = input_hp_t1 = 0.0;
  
  integrationMethod = newIntegrationMethod;
  
  // instantiate downsampling filter
  iir = new IIRLowpass(sampleRate * oversamplingFactor, IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0, IIR_DOWNSAMPLE_ORDER);
}

// default constructor
SKFilter::SKFilter(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;
  oversamplingFactor = 2;
  filterMode = SK_LOWPASS_MODE;
  sampleRate = 44100.0;

  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = p1 = out = 0.0;

  // initialize filter inputs
  input_lp = input_bp = input_hp = 0.0;
  input_lp_t1 = input_bp_t1 = input_hp_t1 = 0.0;
  
  integrationMethod = SK_TRAPEZOIDAL;
  
  // instantiate downsampling filter
  iir = new IIRLowpass(sampleRate * oversamplingFactor, IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0, IIR_DOWNSAMPLE_ORDER);
}

// default destructor
SKFilter::~SKFilter(){
  delete iir;
}

void SKFilter::ResetFilterState(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;

  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = p1 = out = 0.0;

  // initialize filter inputs
  input_lp = input_bp = input_hp = 0.0;
  input_lp_t1 = input_bp_t1 = input_hp_t1 = 0.0;
  
  // set oversampling
  iir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0);
}

void SKFilter::SetFilterCutoff(float newCutoff){
  cutoffFrequency = newCutoff;

  SetFilterIntegrationRate();
}

void SKFilter::SetFilterResonance(float newResonance){
  Resonance = newResonance;
}

void SKFilter::SetFilterOversamplingFactor(int newOversamplingFactor){
  oversamplingFactor = newOversamplingFactor;
  iir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0);

  SetFilterIntegrationRate();
}

void SKFilter::SetFilterMode(SKFilterMode newFilterMode){
  filterMode = newFilterMode;
}

void SKFilter::SetFilterSampleRate(float newSampleRate){
  sampleRate = newSampleRate;
  iir->SetFilterSamplerate(sampleRate * (float)(oversamplingFactor));
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0);

  SetFilterIntegrationRate();
}

void SKFilter::SetFilterIntegrationMethod(SKIntegrationMethod method){
  integrationMethod = method;
}

void SKFilter::SetFilterIntegrationRate(){
  // normalize cutoff freq to samplerate
  dt = 44100.0 / (sampleRate * oversamplingFactor) * cutoffFrequency;

  // clamp integration rate
  if(dt < 0.0){
    dt = 0.0;
  }
  else if(dt > 0.35){
    dt = 0.35;
  }
}

float SKFilter::GetFilterCutoff(){
  return cutoffFrequency;
}

float SKFilter::GetFilterResonance(){
  return Resonance;
}

int SKFilter::GetFilterOversamplingFactor(){
  return oversamplingFactor;
}

float SKFilter::GetFilterOutput(){
  return out;
}

SKFilterMode SKFilter::GetFilterMode(){
  return filterMode;
}

float SKFilter::GetFilterSampleRate(){
  return sampleRate;
}

SKIntegrationMethod SKFilter::GetFilterIntegrationMethod(){
  return integrationMethod;
}

void SKFilter::filter(float input){
  // noise term
  float noise;

  // feedback amount variables
  float res=4.0*Resonance;
  float fb=0.0;

  // update noise terms
  noise = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  noise = 1.0e-6 * 2.0 * (noise - 0.5);

  input += noise;

  // set filter mode
  switch(filterMode){
  case SK_LOWPASS_MODE:
    input_lp = input;
    input_bp = 0.0;
    input_hp = 0.0;
    break;
  case SK_BANDPASS_MODE:
    input_lp = 0.0;
    input_bp = input;
    input_hp = 0.0;
    break;
  case SK_HIGHPASS_MODE:
    input_lp = 0.0;
    input_bp = 0.0;
    input_hp = input;
    break;
  default:
    input_lp = 0.0;
    input_bp = 0.0;
    input_hp = 0.0;
  }
    
  // integrate filter state
  // with oversampling
  for(int nn = 0; nn < oversamplingFactor; nn++){
    // switch integration method
    switch(integrationMethod){
    case SK_SEMI_IMPLICIT_EULER:
      // semi-implicit euler integration
      {
	fb = input_bp + res*p1;
	p0 += dt*(input_lp - p0 - fb);
       	p1 += dt*(p0 + fb - p1 - 1.0/4.0*SinhPade34(p0*4.0));
      	out = p1;
      }
      break;
    case SK_PREDICTOR_CORRECTOR:
      // predictor-corrector integration
      {
	float p0_prime, p1_prime, fb_prime;
	  
	fb = input_bp_t1 + res*p1;
	p0_prime = p0 + dt*(input_lp_t1 - p0 - fb);
       	p1_prime = p1 + dt*(p0 + fb - p1 - 1.0/4.0*SinhPade34(p1*4.0));	
	fb_prime = input_bp + res*p1_prime;
	
       	p1 += 0.5*dt*((p0 + fb - p1 - 1.0/4.0*SinhPade34(p1*4.0)) +
		      (p0_prime + fb_prime - p1_prime - 1.0/4.0*SinhPade34(p1*4.0)));
	p0 += 0.5*dt*((input_lp_t1 - p0 - fb) +
		      (input_lp - p0_prime - fb_prime));

	out = p1;
      }
      break;
    case SK_TRAPEZOIDAL:
      // trapezoidal integration
      {
	float x_k, x_k2;
	float fb_t = input_bp_t1 + res*p1;
	float alpha = dt/2.0;
	float A = p0 + fb_t - p1 - 1.0/4.0*SinhPade54(4.0*p1) +
	           p0/(1.0 + alpha) + alpha/(1 + alpha)*(input_lp_t1 - p0 - fb_t + input_lp);
	float c = 1.0 - (alpha - alpha*alpha/(1.0 + alpha))*res + alpha;
	float D_n = p1 + alpha*A + (alpha - alpha*alpha/(1.0 + alpha))*input_bp;

	x_k = p1;
	
	// newton-raphson
	for(int ii=0; ii < 8; ii++) {
	  x_k2 = x_k - (c*x_k + alpha*1.0/4.0*SinhPade54(4.0*x_k) - D_n)/(c + alpha*CoshPade54(4.0*x_k));
	  
	  // breaking limit
	  if(abs(x_k2 - x_k) < 1.0e-9) {
	    x_k = x_k2;
	    break;
	  }
	  
	  x_k = x_k2;
	}
	
	p1 = x_k;
	fb = input_bp + res*p1;
	p0 = p0/(1.0 + alpha) + alpha/(1.0 + alpha)*(input_lp_t1 - p0 - fb_t + input_lp - fb);
	out = p1;
      }
      break;
    default:
      break;
    }

    // downsampling filter
    if(oversamplingFactor > 1){
      out = iir->IIRfilter(out);
    }
  }
  
  // set input at t-1
  input_lp_t1 = input_lp;    
  input_bp_t1 = input_bp;    
  input_hp_t1 = input_hp;    
}

void SKFilter::SetFilterLowpassInput(float input){
  input_lp = input;
}

void SKFilter::SetFilterBandpassInput(float input){
  input_bp = input;
}

void SKFilter::SetFilterHighpassInput(float input){
  input_hp = input;
}

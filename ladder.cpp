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

#include <cstdlib>
#include <cmath>
#include "ladder.h"
#include "iir.h"
#include "fastmath.h"

// steepness of downsample filter response
#define IIR_DOWNSAMPLE_ORDER 8

// downsampling passthrough bandwidth
#define IIR_DOWNSAMPLING_BANDWIDTH 0.9

// constructor
Ladder::Ladder(float newCutoff, float newResonance, int newOversamplingFactor,
	       LadderFilterMode newFilterMode, float newSampleRate, LadderIntegrationMethod newIntegrationMethod){
  // initialize filter parameters
  cutoffFrequency = newCutoff;
  Resonance = newResonance;
  oversamplingFactor = newOversamplingFactor;
  filterMode = newFilterMode;
  sampleRate = newSampleRate;

  SetFilterIntegrationRate();

  // initialize filter state
  p0 = p1 = p2 = p3 = out = ut_1 = 0.0;
  
  integrationMethod = newIntegrationMethod;
  
  // instantiate downsampling filter
  iir = new IIRLowpass(sampleRate * oversamplingFactor, IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0, IIR_DOWNSAMPLE_ORDER);
}

// default constructor
Ladder::Ladder(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;
  oversamplingFactor = 2;
  filterMode = LADDER_LOWPASS_MODE;
  sampleRate = 44100.0;

  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = p1 = p2 = p3 = out = ut_1 = 0.0;
  
  integrationMethod = LADDER_PREDICTOR_CORRECTOR_FULL_TANH;
  
  // instantiate downsampling filter
  iir = new IIRLowpass(sampleRate * oversamplingFactor, IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0, IIR_DOWNSAMPLE_ORDER);
}

// default destructor
Ladder::~Ladder(){
  delete iir;
}

void Ladder::ResetFilterState(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.0;

  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = p1 = p2 = p3 = out = ut_1 = 0.0;
  
  // set oversampling
  iir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0);
}

void Ladder::SetFilterCutoff(float newCutoff){
  cutoffFrequency = newCutoff;

  SetFilterIntegrationRate();
}

void Ladder::SetFilterResonance(float newResonance){
  Resonance = newResonance;
}

void Ladder::SetFilterOversamplingFactor(int newOversamplingFactor){
  oversamplingFactor = newOversamplingFactor;
  iir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0);

  SetFilterIntegrationRate();
}

void Ladder::SetFilterMode(LadderFilterMode newFilterMode){
  filterMode = newFilterMode;
}

void Ladder::SetFilterSampleRate(float newSampleRate){
  sampleRate = newSampleRate;
  iir->SetFilterSamplerate(sampleRate * (float)(oversamplingFactor));
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0);

  SetFilterIntegrationRate();
}

void Ladder::SetFilterIntegrationMethod(LadderIntegrationMethod method){
  integrationMethod = method;
}

void Ladder::SetFilterIntegrationRate(){
  // normalize cutoff freq to samplerate
  dt = 44100.0 / (sampleRate * oversamplingFactor) * cutoffFrequency;

  // clamp integration rate
  if(dt < 0.0){
    dt = 0.0;
  }
  else if(dt > 0.85){
    dt = 0.85;
  }
}

float Ladder::GetFilterCutoff(){
  return cutoffFrequency;
}

float Ladder::GetFilterResonance(){
  return Resonance;
}

int Ladder::GetFilterOversamplingFactor(){
  return oversamplingFactor;
}

float Ladder::GetFilterOutput(){
  return out;
}

LadderFilterMode Ladder::GetFilterMode(){
  return filterMode;
}

float Ladder::GetFilterSampleRate(){
  return sampleRate;
}

LadderIntegrationMethod Ladder::GetFilterIntegrationMethod(){
  return integrationMethod;
}

void Ladder::LadderFilter(float input){
  // noise term
  float noise;

  // feedback amount
  float fb = 8.0*Resonance;

  // update noise terms
  noise = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  noise = 1.0e-6 * 2.0 * (noise - 0.5);

  input += noise;
  
  // integrate filter state
  // with oversampling
  for(int nn = 0; nn < oversamplingFactor; nn++){
    // switch integration method
    switch(integrationMethod){
    case LADDER_EULER_FULL_TANH:
      // semi-implicit euler integration
      // with full tanh stages
      {
	p0 = p0 + dt*(TanhPade32(input - fb*p3) - TanhPade32(p0));
	p1 = p1 + dt*(TanhPade32(p0) - TanhPade32(p1));
	p2 = p2 + dt*(TanhPade32(p1) - TanhPade32(p2));
	p3 = p3 + dt*(TanhPade32(p2) - TanhPade32(p3));
      }
      break;
      
    case LADDER_PREDICTOR_CORRECTOR_FULL_TANH:
      // predictor-corrector integration
      // with full tanh stages
      {
	float p0_prime, p1_prime, p2_prime, p3_prime, p3t_1;

	// predictor
	p0_prime = p0 + dt*(TanhPade32(ut_1 - fb*p3) - TanhPade32(p0));
	p1_prime = p1 + dt*(TanhPade32(p0) - TanhPade32(p1));
	p2_prime = p2 + dt*(TanhPade32(p1) - TanhPade32(p2));
	p3_prime = p3 + dt*(TanhPade32(p2) - TanhPade32(p3));

	// corrector
	p3t_1 = p3;
	p3 = p3 + 0.5*dt*((TanhPade32(p2) - TanhPade32(p3)) + (TanhPade32(p2_prime) - TanhPade32(p3_prime)));
	p2 = p2 + 0.5*dt*((TanhPade32(p1) - TanhPade32(p2)) + (TanhPade32(p1_prime) - TanhPade32(p2_prime)));
	p1 = p1 + 0.5*dt*((TanhPade32(p0) - TanhPade32(p1)) + (TanhPade32(p0_prime) - TanhPade32(p1_prime)));
	p0 = p0 + 0.5*dt*((TanhPade32(ut_1 - fb*p3t_1) - TanhPade32(p0)) + (TanhPade32(input - fb*p3) - TanhPade32(p0_prime)));
      }
      break;
      
    case LADDER_PREDICTOR_CORRECTOR_FEEDBACK_TANH:
      // predictor-corrector integration
      // with feedback tanh stage only
      {
	float p0_prime, p1_prime, p2_prime, p3_prime, p3t_1;

	// predictor
	p0_prime = p0 + dt*(TanhPade32(ut_1 - fb*p3) - p0);
	p1_prime = p1 + dt*(p0 - p1);
	p2_prime = p2 + dt*(p1 - p2);
	p3_prime = p3 + dt*(p2 - p3);

	// corrector
	p3t_1 = p3;
	p3 = p3 + 0.5*dt*((p2 - p3) + (p2_prime - p3_prime));
	p2 = p2 + 0.5*dt*((p1 - p2) + (p1_prime - p2_prime));
	p1 = p1 + 0.5*dt*((p0 - p1) + (p0_prime - p1_prime));
	p0 = p0 + 0.5*dt*((TanhPade32(ut_1 - fb*p3t_1) - p0) +
			  (TanhPade32(input - fb*p3) - p0_prime));
      }
      break;
      
    case LADDER_TRAPEZOIDAL_FEEDBACK_TANH:
      // implicit trapezoidal integration
      // with feedback tanh stage only
      {
	float x_k, x_k2, g, b, c, C_t, D_t, ut, ut_2;
	float p0_prime, p1_prime, p2_prime, p3_prime;

	ut = TanhPade32(ut_1 - fb*p3);
    	b = (0.5*dt)/(1.0 + 0.5*dt);
	c = (1.0 - 0.5*dt)/(1.0 + 0.5*dt);
	g = -fb*b*b*b*b;
	x_k = ut;
	D_t = c*p3 + (b + c*b)*p2 + (b*b+b*b*c)*p1 +
	               (b*b*b+b*b*b*c)*p0 + b*b*b*b*ut;
	C_t = TanhPade32(input - fb*D_t);

	// newton-raphson 
	for(int ii=0; ii < 8; ii++) {
	  float tanh_g_xk, tanh_g_xk2;
	  
	  tanh_g_xk = TanhPade32(g*x_k);
	  tanh_g_xk2 = g*(1.0 - TanhPade32(g*x_k)*TanhPade32(g*x_k));
	  
	  x_k2 = x_k - (x_k + x_k*tanh_g_xk*C_t - tanh_g_xk - C_t) /
	                 (1.0 + C_t*(tanh_g_xk + x_k*tanh_g_xk2) - tanh_g_xk2);
	  
	  // breaking limit
	  if(abs(x_k2 - x_k) < 1.0e-9) {
	    x_k = x_k2;
	    break;
	  }
	  
	  x_k = x_k2;
	}
	
	ut_2 = x_k;

	p0_prime = p0;
	p1_prime = p1;
	p2_prime = p2;
	p3_prime = p3;

	p0 = c*p0_prime + b*(ut + ut_2);
	p1 = c*p1_prime + b*(p0_prime + p0);
	p2 = c*p2_prime + b*(p1_prime + p1);
	p3 = c*p3_prime + b*(p2_prime + p2);
      }
      break;
      
    default:
      break;
    }

    // input at t-1
    ut_1 = input;

    //switch filter mode
    switch(filterMode){
    case LADDER_LOWPASS_MODE:
      out = p3;
      break;
    case LADDER_BANDPASS_MODE:
      out = p1 - p3;
      break;
    case LADDER_HIGHPASS_MODE:
      out = TanhPade32(input - p0 - fb*p3);
      break;
    default:
      out = 0.0;
    }

    // downsampling filter
    if(oversamplingFactor > 1){
      out = iir->IIRfilter(out);
    }
  }
}

float Ladder::GetFilterLowpass(){
  return p3;
}

float Ladder::GetFilterBandpass(){
  return 0.0;
}

float Ladder::GetFilterHighpass(){
  return 0.0;
}



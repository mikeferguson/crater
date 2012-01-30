/*******************************************************************************
 * AVRRA: The AVR Robotics API
 * smooth.h - some functions to clean up data
 * 
 * Copyright (c) 2008, Michael E. Ferguson
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice, 
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *   this list of conditions and the following disclaimer in the documentation 
 *   and/or other materials provided with the distribution.
 * - Neither the name of AVRRA nor the names of its contributors 
 *   may be used to endorse or promote products derived from this software 
 *   without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef AVRRA_SENSOR_SMOOTH
#define AVRRA_SENSOR_SMOOTH

/** models used by bayesThresh */
//#define MODEL_GP2D12        1
/** pobability that Zi < val, given Xi true */
//#define MODEL_GP2D12_ZLT    85L
/** pobability that Zi > val, given Xi true */
//#define MODEL_GP2D12_ZGT    15L
/** pobability that Zi < val, given Xi false */
//#define MODEL_GP2D12_ZLF    60L
/** pobability that Zi > val, given Xi false */
//#define MODEL_GP2D12_ZGF    40L


/** = a smoothed out reading of a port - useful for IR sensors... */
int smooth(int (*func)(char ch), char channel){
    int reading = func(channel);
    reading = reading + func(channel);
    reading = reading + func(channel);
    reading = reading + func(channel);
    reading = reading/4;
    return reading; 
}

/** = posterior probablity that actual distance < val. Uses a bayesian
      thresholding function. 
int bayesThresh(int (*func)(char ch), char channel, int val, int model){
    // all probabilities are scaled to integers (0 = 0.0, 100 = 1.0)

    // A sensor is one of two states: closer than <val> to an object, or not.
    // Each sensor state, Xi, has an associated probability, p(Xi).
    
    // We denote X0 as the prior probability that we are closer than <val>
    // We denote Xi as the probability after i measurements. 
    // We denote Zi as the ith measurment.

    // We are only concerned with Xi true. Thus, after our ith reading:
    // p(Xi = T) = p(Zi|X[i-1]=T)*p(X[i-1]=T)/Sum
    
    // Zi
    int Zi;
    // p(Xi)    
    long pXi;
    // p(X[i-1])
    int prior;
    // number of iterations to perform
    int i;
    
    switch(model){
        case MODEL_GP2D12:
            prior = 50; // E0
            for(i = 0; i < 5; i++){
                // take a measurement
                Zi = gp2d12GetData(channel);
                // update the posterior based on measurement
                if(Zi > (val+2)){
                    // p(Xi = T) = p(Zi|X[i-1]=T)*p(X[i-1]=T)/Sum
                    pXi = (MODEL_GP2D12_ZGT * prior * 100)/(MODEL_GP2D12_ZGT * prior + MODEL_GP2D12_ZGF * (100-prior));
                }else{
                    pXi = (MODEL_GP2D12_ZLT * prior * 100)/(MODEL_GP2D12_ZLT * prior + MODEL_GP2D12_ZLF * (100-prior));
                }
                // next prior = current posterior
                prior = pXi + 1;
            }
            break;
        default:
            // no other models at this time!
            pXi = 0;
    }
    return pXi;
    if(pXi < 60){
        return 0;
    }else{
        return 1;
    }
}*/

#endif

/*********************************End of smooth.h*******************************
 * REVISIONS
 * 11-08-08 - smooth() now works with int-return type functions.
 * 11-15-08 - added bayesThresh()
 */


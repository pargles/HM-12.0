/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.  
 *
 * Copyright (c) 2010-2013, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     decmain.cpp
    \brief    Decoder application main
*/

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "TAppDecTop.h"

//gcorrea 17/10/2013
ofstream CU64x64data;
ofstream CU32x32data;
ofstream CU16x16data;
ofstream CU8x8data;

char filename_64x64[100];
char filename_32x32[100];
char filename_16x16[100];
char filename_8x8[100];

Pel **saveLumaPel;
Pel **saveHorGrad;
Pel **saveVerGrad;

int count_frame;

int saveResData2Nx2N;
double sumRes2Nx2N, medRes2Nx2N, sqdRes2Nx2N, varRes2Nx2N;
double res_sum_VP1, res_sum_VP2, res_med_VP1, res_med_VP2, res_sqd_VP1, res_sqd_VP2, res_var_VP1, res_var_VP2;
double res_sum_HP1, res_sum_HP2, res_med_HP1, res_med_HP2, res_sqd_HP1, res_sqd_HP2, res_var_HP1, res_var_HP2;
double ResHorGrad, ResVerGrad, ResGrad, res_RHV_grad;
double res_rhi_V, res_rhi_H, res_rhi_Q;
double res_RHV_sum, res_RHV_med, res_RHV_var, res_RHV_HI;
double res_HP2Sobel, res_HP1Sobel, res_VP2Sobel, res_VP1Sobel;
double res_TotalSobelVer_CB, res_TotalSobelHor_CB, res_TotalSobel_CB, res_RHV_Sobel, res_RHV_TotalSobel;
double res_HP2grad, res_HP1grad, res_VP2grad, res_VP1grad;
double res_TotalGradVer_CB, res_TotalGradHor_CB, res_TotalGrad_CB, res_RHV_TotalGrad_CB ;
double res_RHV_sumCoef;
double res_RHV_sumVar3x3, res_RVH_sumVar3x3, res_sumVar3x3_HP2, res_sumVar3x3_HP1, res_sumVar3x3_VP2, res_sumVar3x3_VP1;
double ResHorAccGrad, ResVerAccGrad;

int curr_uiDepth;

int frameWidth, frameHeight;
int nCU_hor, nCU_ver, nCU32x32_hor, nCU32x32_ver, nCU16x16_hor, nCU16x16_ver, nCU8x8_hor, nCU8x8_ver, nCU;
//gcorrea 17/10/2013 END

//! \ingroup TAppDecoder
//! \{

bool g_md5_mismatch = false; ///< top level flag that indicates if there has been a decoding mismatch

// ====================================================================================================================
// Main function
// ====================================================================================================================

int main(int argc, char* argv[])
{
  TAppDecTop  cTAppDecTop;

  // print information
  fprintf( stdout, "\n" );
  fprintf( stdout, "HM software: Decoder Version [%s]", NV_VERSION );
  fprintf( stdout, NVM_ONOS );
  fprintf( stdout, NVM_COMPILEDBY );
  fprintf( stdout, NVM_BITS );
  fprintf( stdout, "\n" );

  // create application decoder class
  cTAppDecTop.create();

  // parse configuration
  if(!cTAppDecTop.parseCfg( argc, argv ))
  {
    cTAppDecTop.destroy();
    return 1;
  }

  // starting time
  double dResult;
  long lBefore = clock();

  // call decoding function
  cTAppDecTop.decode();

  if (g_md5_mismatch)
  {
    printf("\n\n***ERROR*** A decoding mismatch occured: signalled md5sum does not match\n");
  }

  // ending time
  dResult = (double)(clock()-lBefore) / CLOCKS_PER_SEC;
  printf("\n Total Time: %12.3f sec.\n", dResult);

  // destroy application decoder class
  cTAppDecTop.destroy();

  return g_md5_mismatch ? EXIT_FAILURE : EXIT_SUCCESS;
}

//! \}

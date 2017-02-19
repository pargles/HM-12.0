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

/** \file     encmain.cpp
    \brief    Encoder application main
*/

#include <time.h>
#include <iostream>
#include "TAppEncTop.h"
#include "TAppCommon/program_options_lite.h"
#include "libc5/C5.h"
#include "libc5/C5Decoder.h"
#define BILLION  1000000000L
using namespace std;
namespace po = df::program_options_lite;

//gcorrea 17/10/2013
ofstream CU64x64data;
ofstream CU32x32data;
ofstream CU16x16data;
ofstream CU8x8data;
ofstream C5headerFileCU64x64;
ofstream C5headerFileCU32x32;
ofstream C5headerFileCU16x16;
ofstream C5dataFileCU64x64;
ofstream C5dataFileCU32x32;
ofstream C5dataFileCU16x16;
ofstream C5costsFileCU64x64;
ofstream C5costsFileCU32x32;
ofstream C5costsFileCU16x16;

C5 c5_64;
C5 c5_32;
C5 c5_16;

C5Decoder c5_64_decoder;
C5Decoder c5_32_decoder;
C5Decoder c5_16_decoder;

bool onlineTrainingIsDone;
char filename_64x64[100];
char filename_32x32[100];
char filename_16x16[100];
char filename_8x8[100];

char filename_C5Costs_64x64[100];
char filename_C5Costs_32x32[100];
char filename_C5Costs_16x16[100];

char filename_C5header_64x64[100];
char filename_C5header_32x32[100];
char filename_C5header_16x16[100];
char filename_C5data_64x64[100];
char filename_C5data_32x32[100];
char filename_C5data_16x16[100];
Char*  relation;
string cu64x64forC5;
string cu32x32forC5;
string cu16x16forC5;

string last64x64SplitLineVector;
string last64x64NonSplitLineVector;
int split64x64CuOrNotCounter;

string last32x32SplitLineVector;
string last32x32NonSplitLineVector;
int split32x32CuOrNotCounter;

string last16x16SplitLineVector;
string last16x16NonSplitLineVector;
int split16x16CuOrNotCounter;

int disparityLimitOfLineBeforeBalance;


Pel **saveLumaPel;
Pel **saveHorGrad;
Pel **saveVerGrad;

int count_frame;
int GOPforC5;

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
double RDcost_MSM, RDcost_2Nx2N, RDcost_2NxN, RDcost_Nx2N, RDcost_NxN, RDcost_2NxnU, RDcost_2NxnD, RDcost_nLx2N, RDcost_nRx2N;

int curr_uiDepth;

int frameWidth, frameHeight;
int nCU_hor, nCU_ver, nCU32x32_hor, nCU32x32_ver, nCU16x16_hor, nCU16x16_ver, nCU8x8_hor, nCU8x8_ver, nCU;

long int count_64x64_MSM, count_64x64_MERGE, count_64x64_2Nx2N_MERGE, count_64x64_SKIP, count_64x64_2Nx2N_SKIP, count_64x64_2Nx2N_nonMSM, count_64x64_2Nx2N, count_64x64_2NxN, count_64x64_Nx2N, count_64x64_NxN, count_64x64_2NxnU, count_64x64_2NxnD, count_64x64_nLx2N, count_64x64_nRx2N;
long int count_32x32_MSM, count_32x32_MERGE, count_32x32_2Nx2N_MERGE, count_32x32_SKIP, count_32x32_2Nx2N_SKIP, count_32x32_2Nx2N_nonMSM, count_32x32_2Nx2N, count_32x32_2NxN, count_32x32_Nx2N, count_32x32_NxN, count_32x32_2NxnU, count_32x32_2NxnD, count_32x32_nLx2N, count_32x32_nRx2N;
long int count_16x16_MSM, count_16x16_MERGE, count_16x16_2Nx2N_MERGE, count_16x16_SKIP, count_16x16_2Nx2N_SKIP, count_16x16_2Nx2N_nonMSM, count_16x16_2Nx2N, count_16x16_2NxN, count_16x16_Nx2N, count_16x16_NxN, count_16x16_2NxnU, count_16x16_2NxnD, count_16x16_nLx2N, count_16x16_nRx2N;
long int count_8x8_MSM, count_8x8_MERGE, count_8x8_2Nx2N_MERGE, count_8x8_SKIP, count_8x8_2Nx2N_SKIP, count_8x8_2Nx2N_nonMSM, count_8x8_2Nx2N, count_8x8_2NxN, count_8x8_Nx2N, count_8x8_NxN, count_8x8_2NxnU, count_8x8_2NxnD, count_8x8_nLx2N, count_8x8_nRx2N;
//gcorrea 17/10/2013 END


//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Main function
// ====================================================================================================================

int main(int argc, char* argv[])
{
    onlineTrainingIsDone= false;//set to true on TEncCu after N frames
    
    GOPforC5 = 30;//TODO - get this number as a parameter
    string C5header;
    //predicted class, true class : cost
    // the costs file penalizes if the predicted class
    // was to not split the CU, but it was supposed to be
    //split
    string C5costs = "0, 1: 10\n";
    
    disparityLimitOfLineBeforeBalance = 10;
    
    split64x64CuOrNotCounter = 0;
    last64x64SplitLineVector ="";
    last64x64NonSplitLineVector="";
    
    split32x32CuOrNotCounter = 0;
    last32x32SplitLineVector ="";
    last32x32NonSplitLineVector="";
    
    split16x16CuOrNotCounter = 0;
    last16x16SplitLineVector ="";
    last16x16NonSplitLineVector="";
            
  TAppEncTop  cTAppEncTop;
  relation = "defaultRelation";
  // print information
  fprintf( stdout, "\n" );
 // fprintf( stdout, cfg.getInputFile() );
  //fprintf( stdout, cfg.getQP());
  fprintf( stdout, "HM software: Encoder Version [%s]", NV_VERSION );
  fprintf( stdout, NVM_ONOS );
  fprintf( stdout, NVM_COMPILEDBY );
  fprintf( stdout, NVM_BITS );
  fprintf( stdout, "\n" );
  
  C5header+= "SplitQuadtree.\n";
    C5header+="RDcost_MSM: continuous.\n";
    C5header+= "RDcost_2Nx2N: continuous.\n";
    C5header+= "RDcost_2NxN: continuous.\n";
    C5header+= "RDcost_Nx2N: continuous.\n";
    C5header+= "part: 0,1,2,3,4,5,6,7.\n";
    C5header+= "MergeFlag: 0,1.\n";
    C5header+= "SkipMergeFlag: 0,1.\n";
    C5header+= "neighDepth_diff: continuous.\n";
    C5header+= "abs(a2-a1)/a1: continuous.\n";
    C5header+= "a2/a1: continuous.\n";
    C5header+= "SplitQuadtree: 0,1.\n";
    
    
    strcpy(filename_C5header_64x64,"/home/pargles/Documents/codificador/HM-12.0/C5hevc_64x64.names");
    strcpy(filename_C5data_64x64,"/home/pargles/Documents/codificador/HM-12.0/C5hevc_64x64.data");
    C5headerFileCU64x64.open(filename_C5header_64x64, ios::out);
    if(C5headerFileCU64x64.is_open()){
        C5headerFileCU64x64 << C5header; 
        C5headerFileCU64x64.close();
    }
    strcpy(filename_C5header_32x32,"/home/pargles/Documents/codificador/HM-12.0/C5hevc_32x32.names");
    strcpy(filename_C5data_32x32,"/home/pargles/Documents/codificador/HM-12.0/C5hevc_32x32.data");
    C5headerFileCU32x32.open(filename_C5header_32x32, ios::out);
    if(C5headerFileCU32x32.is_open()){
        C5headerFileCU32x32 << C5header; 
        C5headerFileCU32x32.close();
    }
    strcpy(filename_C5header_16x16,"/home/pargles/Documents/codificador/HM-12.0/C5hevc_16x16.names");
    strcpy(filename_C5data_16x16,"/home/pargles/Documents/codificador/HM-12.0/C5hevc_16x16.data");
    C5headerFileCU16x16.open(filename_C5header_16x16, ios::out);
    if(C5headerFileCU16x16.is_open()){
        C5headerFileCU16x16 << C5header; 
        C5headerFileCU16x16.close();
    }
    
    //-------------------- COSTS FILE ---------------------------------
    strcpy(filename_C5Costs_64x64,"/home/pargles/Documents/codificador/HM-12.0/C5hevc_64x64.costs");
    C5costsFileCU64x64.open(filename_C5Costs_64x64, ios::out);
    if(C5costsFileCU64x64.is_open()){
        C5costsFileCU64x64 << C5costs; 
        C5costsFileCU64x64.close();
    }
    
    strcpy(filename_C5Costs_32x32,"/home/pargles/Documents/codificador/HM-12.0/C5hevc_32x32.costs");
    C5costsFileCU32x32.open(filename_C5Costs_32x32, ios::out);
    if(C5costsFileCU32x32.is_open()){
        C5costsFileCU32x32 << C5costs; 
        C5costsFileCU32x32.close();
    }
    
    strcpy(filename_C5Costs_16x16,"/home/pargles/Documents/codificador/HM-12.0/C5hevc_16x16.costs");
    C5costsFileCU16x16.open(filename_C5Costs_16x16, ios::out);
    if(C5costsFileCU16x16.is_open()){
        C5costsFileCU16x16 << C5costs; 
        C5costsFileCU16x16.close();
    }
  // create application encoder class
  cTAppEncTop.create();

  // parse configuration
  try
  {
    if(!cTAppEncTop.parseCfg( argc, argv ))
    {
      cTAppEncTop.destroy();
      return 1;
    }
  }
  catch (po::ParseFailure& e)
  {
    cerr << "Error parsing option \""<< e.arg <<"\" with argument \""<< e.val <<"\"." << endl;
    return 1;
  }

  // starting time
  
  //@see http://users.pja.edu.pl/~jms/qnx/help/watcom/clibref/qnx/clock_gettime.html
  
  struct timespec start, stop;
  double totalTime;

  if( clock_gettime( CLOCK_REALTIME, &start) == -1 ) {
      perror( "clock gettime" );
      exit( EXIT_FAILURE );
    }

  // call encoding function
  cTAppEncTop.encode();

  if( clock_gettime( CLOCK_REALTIME, &stop) == -1 ) {
      perror( "clock gettime" );
      exit( EXIT_FAILURE );
    }
  
  totalTime = ( stop.tv_sec - start.tv_sec )
          + ( stop.tv_nsec - start.tv_nsec )
            / BILLION;
    printf("\n Total Time: %.2f sec.\n", totalTime );
  

  // destroy application encoder class
  cTAppEncTop.destroy();

  return 0;
}

//! \}

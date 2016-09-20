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

/** \file     TEncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include <stdio.h>
#include "TEncTop.h"
#include "TEncCu.h"
#include "TEncAnalyze.h"
#include "libc5/C5Decoder.h"

#include <cmath>
#include <algorithm>
using namespace std;

//gcorrea 17/10/2013
extern ofstream CU64x64data;
extern ofstream CU32x32data;
extern ofstream CU16x16data;
extern ofstream CU8x8data;
extern ofstream C5dataFileCU64x64;
extern ofstream C5dataFileCU32x32;
extern ofstream C5dataFileCU16x16;
extern bool onlineTrainingIsDone;
extern char filename_64x64[100];
extern char filename_32x32[100];
extern char filename_16x16[100];
extern char filename_8x8[100];
extern char filename_C5data_64x64[100];
extern char filename_C5data_32x32[100];
extern char filename_C5data_16x16[100];
extern Char*  relation;

extern C5Decoder c5_64_decoder;
extern C5Decoder c5_32_decoder;
extern C5Decoder c5_16_decoder;

extern string cu64x64forC5;
extern string cu32x32forC5;
extern string cu16x16forC5;

extern string last64x64SplitLineVector;
extern string last64x64NonSplitLineVector;
extern int split64x64CuOrNotCounter;

extern string last32x32SplitLineVector;
extern string last32x32NonSplitLineVector;
extern int split32x32CuOrNotCounter;

extern string last16x16SplitLineVector;
extern string last16x16NonSplitLineVector;
extern int split16x16CuOrNotCounter;

extern int disparityLimitOfLineBeforeBalance;

extern Pel **saveLumaPel;
extern Pel **saveHorGrad;
extern Pel **saveVerGrad;

extern int count_frame;
extern int GOPforC5;
extern int saveResData2Nx2N;
extern double sumRes2Nx2N, medRes2Nx2N, sqdRes2Nx2N, varRes2Nx2N;
extern double res_sum_VP1, res_sum_VP2, res_med_VP1, res_med_VP2, res_sqd_VP1, res_sqd_VP2, res_var_VP1, res_var_VP2;
extern double res_sum_HP1, res_sum_HP2, res_med_HP1, res_med_HP2, res_sqd_HP1, res_sqd_HP2, res_var_HP1, res_var_HP2;
extern double ResHorGrad, ResVerGrad, ResGrad, res_RHV_grad;
extern double res_rhi_V, res_rhi_H, res_rhi_Q;
extern double res_RHV_sum, res_RHV_med, res_RHV_var, res_RHV_HI;
extern double res_HP2Sobel, res_HP1Sobel, res_VP2Sobel, res_VP1Sobel;
extern double res_TotalSobelVer_CB, res_TotalSobelHor_CB, res_TotalSobel_CB, res_RHV_Sobel, res_RHV_TotalSobel;
extern double res_HP2grad, res_HP1grad, res_VP2grad, res_VP1grad;
extern double res_TotalGradVer_CB, res_TotalGradHor_CB, res_TotalGrad_CB, res_RHV_TotalGrad_CB ;
extern double res_RHV_sumCoef;
extern double res_RHV_sumVar3x3, res_RVH_sumVar3x3, res_sumVar3x3_HP2, res_sumVar3x3_HP1, res_sumVar3x3_VP2, res_sumVar3x3_VP1;
extern double ResHorAccGrad, ResVerAccGrad;
extern double RDcost_MSM, RDcost_2Nx2N, RDcost_2NxN, RDcost_Nx2N, RDcost_NxN, RDcost_2NxnU, RDcost_2NxnD, RDcost_nLx2N, RDcost_nRx2N;

extern int curr_uiDepth;

extern long int count_64x64_MSM, count_64x64_MERGE, count_64x64_2Nx2N_MERGE, count_64x64_SKIP, count_64x64_2Nx2N_SKIP, count_64x64_2Nx2N_nonMSM, count_64x64_2Nx2N, count_64x64_2NxN, count_64x64_Nx2N, count_64x64_NxN, count_64x64_2NxnU, count_64x64_2NxnD, count_64x64_nLx2N, count_64x64_nRx2N;
extern long int count_32x32_MSM, count_32x32_MERGE, count_32x32_2Nx2N_MERGE, count_32x32_SKIP, count_32x32_2Nx2N_SKIP, count_32x32_2Nx2N_nonMSM, count_32x32_2Nx2N, count_32x32_2NxN, count_32x32_Nx2N, count_32x32_NxN, count_32x32_2NxnU, count_32x32_2NxnD, count_32x32_nLx2N, count_32x32_nRx2N;
extern long int count_16x16_MSM, count_16x16_MERGE, count_16x16_2Nx2N_MERGE, count_16x16_SKIP, count_16x16_2Nx2N_SKIP, count_16x16_2Nx2N_nonMSM, count_16x16_2Nx2N, count_16x16_2NxN, count_16x16_Nx2N, count_16x16_NxN, count_16x16_2NxnU, count_16x16_2NxnD, count_16x16_nLx2N, count_16x16_nRx2N;
extern long int count_8x8_MSM, count_8x8_MERGE, count_8x8_2Nx2N_MERGE, count_8x8_SKIP, count_8x8_2Nx2N_SKIP, count_8x8_2Nx2N_nonMSM, count_8x8_2Nx2N, count_8x8_2NxN, count_8x8_Nx2N, count_8x8_NxN, count_8x8_2NxnU, count_8x8_2NxnD, count_8x8_nLx2N, count_8x8_nRx2N;

//gcorrea 17/10/2013 END

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

/**
 \param    uiTotalDepth  total number of allowable depth
 \param    uiMaxWidth    largest CU width
 \param    uiMaxHeight   largest CU height
 */
Void TEncCu::create(UChar uhTotalDepth, UInt uiMaxWidth, UInt uiMaxHeight)
{
  Int i;
  
  m_uhTotalDepth   = uhTotalDepth + 1;
  m_ppcBestCU      = new TComDataCU*[m_uhTotalDepth-1];
  m_ppcTempCU      = new TComDataCU*[m_uhTotalDepth-1];
    
  m_ppcPredYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcResiYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcRecoYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcPredYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcResiYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcRecoYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcOrigYuv     = new TComYuv*[m_uhTotalDepth-1];
  
  UInt uiNumPartitions;
  for( i=0 ; i<m_uhTotalDepth-1 ; i++)
  {
    uiNumPartitions = 1<<( ( m_uhTotalDepth - i - 1 )<<1 );
    UInt uiWidth  = uiMaxWidth  >> i;
    UInt uiHeight = uiMaxHeight >> i;
    
    m_ppcBestCU[i] = new TComDataCU; m_ppcBestCU[i]->create( uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) );
    m_ppcTempCU[i] = new TComDataCU; m_ppcTempCU[i]->create( uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) );
    
    m_ppcPredYuvBest[i] = new TComYuv; m_ppcPredYuvBest[i]->create(uiWidth, uiHeight);
    m_ppcResiYuvBest[i] = new TComYuv; m_ppcResiYuvBest[i]->create(uiWidth, uiHeight);
    m_ppcRecoYuvBest[i] = new TComYuv; m_ppcRecoYuvBest[i]->create(uiWidth, uiHeight);
    
    m_ppcPredYuvTemp[i] = new TComYuv; m_ppcPredYuvTemp[i]->create(uiWidth, uiHeight);
    m_ppcResiYuvTemp[i] = new TComYuv; m_ppcResiYuvTemp[i]->create(uiWidth, uiHeight);
    m_ppcRecoYuvTemp[i] = new TComYuv; m_ppcRecoYuvTemp[i]->create(uiWidth, uiHeight);
    
    m_ppcOrigYuv    [i] = new TComYuv; m_ppcOrigYuv    [i]->create(uiWidth, uiHeight);
  }
  
  m_bEncodeDQP = false;
#if RATE_CONTROL_LAMBDA_DOMAIN && !M0036_RC_IMPROVEMENT
  m_LCUPredictionSAD = 0;
  m_addSADDepth      = 0;
  m_temporalSAD      = 0;
#endif

  // initialize partition order.
  UInt* piTmp = &g_auiZscanToRaster[0];
  initZscanToRaster( m_uhTotalDepth, 1, 0, piTmp);
  initRasterToZscan( uiMaxWidth, uiMaxHeight, m_uhTotalDepth );
  
  // initialize conversion matrix from partition index to pel
  initRasterToPelXY( uiMaxWidth, uiMaxHeight, m_uhTotalDepth );
}

Void TEncCu::destroy()
{
  Int i;
  
  for( i=0 ; i<m_uhTotalDepth-1 ; i++)
  {
    if(m_ppcBestCU[i])
    {
      m_ppcBestCU[i]->destroy();      delete m_ppcBestCU[i];      m_ppcBestCU[i] = NULL;
    }
    if(m_ppcTempCU[i])
    {
      m_ppcTempCU[i]->destroy();      delete m_ppcTempCU[i];      m_ppcTempCU[i] = NULL;
    }
    if(m_ppcPredYuvBest[i])
    {
      m_ppcPredYuvBest[i]->destroy(); delete m_ppcPredYuvBest[i]; m_ppcPredYuvBest[i] = NULL;
    }
    if(m_ppcResiYuvBest[i])
    {
      m_ppcResiYuvBest[i]->destroy(); delete m_ppcResiYuvBest[i]; m_ppcResiYuvBest[i] = NULL;
    }
    if(m_ppcRecoYuvBest[i])
    {
      m_ppcRecoYuvBest[i]->destroy(); delete m_ppcRecoYuvBest[i]; m_ppcRecoYuvBest[i] = NULL;
    }
    if(m_ppcPredYuvTemp[i])
    {
      m_ppcPredYuvTemp[i]->destroy(); delete m_ppcPredYuvTemp[i]; m_ppcPredYuvTemp[i] = NULL;
    }
    if(m_ppcResiYuvTemp[i])
    {
      m_ppcResiYuvTemp[i]->destroy(); delete m_ppcResiYuvTemp[i]; m_ppcResiYuvTemp[i] = NULL;
    }
    if(m_ppcRecoYuvTemp[i])
    {
      m_ppcRecoYuvTemp[i]->destroy(); delete m_ppcRecoYuvTemp[i]; m_ppcRecoYuvTemp[i] = NULL;
    }
    if(m_ppcOrigYuv[i])
    {
      m_ppcOrigYuv[i]->destroy();     delete m_ppcOrigYuv[i];     m_ppcOrigYuv[i] = NULL;
    }
  }
  if(m_ppcBestCU)
  {
    delete [] m_ppcBestCU;
    m_ppcBestCU = NULL;
  }
  if(m_ppcTempCU)
  {
    delete [] m_ppcTempCU;
    m_ppcTempCU = NULL;
  }
  
  if(m_ppcPredYuvBest)
  {
    delete [] m_ppcPredYuvBest;
    m_ppcPredYuvBest = NULL;
  }
  if(m_ppcResiYuvBest)
  {
    delete [] m_ppcResiYuvBest;
    m_ppcResiYuvBest = NULL;
  }
  if(m_ppcRecoYuvBest)
  {
    delete [] m_ppcRecoYuvBest;
    m_ppcRecoYuvBest = NULL;
  }
  if(m_ppcPredYuvTemp)
  {
    delete [] m_ppcPredYuvTemp;
    m_ppcPredYuvTemp = NULL;
  }
  if(m_ppcResiYuvTemp)
  {
    delete [] m_ppcResiYuvTemp;
    m_ppcResiYuvTemp = NULL;
  }
  if(m_ppcRecoYuvTemp)
  {
    delete [] m_ppcRecoYuvTemp;
    m_ppcRecoYuvTemp = NULL;
  }
  if(m_ppcOrigYuv)
  {
    delete [] m_ppcOrigYuv;
    m_ppcOrigYuv = NULL;
  }
}

/** \param    pcEncTop      pointer of encoder class
 */
Void TEncCu::init( TEncTop* pcEncTop )
{
  m_pcEncCfg           = pcEncTop;
  m_pcPredSearch       = pcEncTop->getPredSearch();
  m_pcTrQuant          = pcEncTop->getTrQuant();
  m_pcBitCounter       = pcEncTop->getBitCounter();
  m_pcRdCost           = pcEncTop->getRdCost();
  
  m_pcEntropyCoder     = pcEncTop->getEntropyCoder();
  m_pcCavlcCoder       = pcEncTop->getCavlcCoder();
  m_pcSbacCoder       = pcEncTop->getSbacCoder();
  m_pcBinCABAC         = pcEncTop->getBinCABAC();
  
  m_pppcRDSbacCoder   = pcEncTop->getRDSbacCoder();
  m_pcRDGoOnSbacCoder = pcEncTop->getRDGoOnSbacCoder();
  
  m_bUseSBACRD        = pcEncTop->getUseSBACRD();
  m_pcRateCtrl        = pcEncTop->getRateCtrl();
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param  rpcCU pointer of CU data class
 */
Void TEncCu::compressCU( TComDataCU*& rpcCU )
{
  // initialize CU data
  m_ppcBestCU[0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
  m_ppcTempCU[0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );

#if RATE_CONTROL_LAMBDA_DOMAIN && !M0036_RC_IMPROVEMENT
  m_addSADDepth      = 0;
  m_LCUPredictionSAD = 0;
  m_temporalSAD      = 0;
#endif

  // analysis of CU
  xCompressCU( m_ppcBestCU[0], m_ppcTempCU[0], 0 );

#if ADAPTIVE_QP_SELECTION
  if( m_pcEncCfg->getUseAdaptQpSelect() )
  {
    if(rpcCU->getSlice()->getSliceType()!=I_SLICE) //IIII
    {
      xLcuCollectARLStats( rpcCU);
    }
  }
#endif
}
/** \param  pcCU  pointer of CU data class
 */
Void TEncCu::encodeCU ( TComDataCU* pcCU )
{
  if ( pcCU->getSlice()->getPPS()->getUseDQP() )
  {
    setdQPFlag(true);
  }

  // Encode CU data
  xEncodeCU( pcCU, 0, 0 );
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
/** Derive small set of test modes for AMP encoder speed-up
 *\param   rpcBestCU
 *\param   eParentPartSize
 *\param   bTestAMP_Hor
 *\param   bTestAMP_Ver
 *\param   bTestMergeAMP_Hor
 *\param   bTestMergeAMP_Ver
 *\returns Void 
*/
#if AMP_ENC_SPEEDUP
#if AMP_MRG
Void TEncCu::deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver, Bool &bTestMergeAMP_Hor, Bool &bTestMergeAMP_Ver)
#else
Void TEncCu::deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver)
#endif
{
  if ( rpcBestCU->getPartitionSize(0) == SIZE_2NxN )
  {
    bTestAMP_Hor = true;
  }
  else if ( rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
  {
    bTestAMP_Ver = true;
  }
  else if ( rpcBestCU->getPartitionSize(0) == SIZE_2Nx2N && rpcBestCU->getMergeFlag(0) == false && rpcBestCU->isSkipped(0) == false )
  {
    bTestAMP_Hor = true;          
    bTestAMP_Ver = true;          
  }

#if AMP_MRG
  //! Utilizing the partition size of parent PU    
  if ( eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N )
  { 
    bTestMergeAMP_Hor = true;
    bTestMergeAMP_Ver = true;
  }

  if ( eParentPartSize == SIZE_NONE ) //! if parent is intra
  {
    if ( rpcBestCU->getPartitionSize(0) == SIZE_2NxN )
    {
      bTestMergeAMP_Hor = true;
    }
    else if ( rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
    {
      bTestMergeAMP_Ver = true;
    }
  }

  if ( rpcBestCU->getPartitionSize(0) == SIZE_2Nx2N && rpcBestCU->isSkipped(0) == false )
  {
    bTestMergeAMP_Hor = true;          
    bTestMergeAMP_Ver = true;          
  }

  if ( rpcBestCU->getWidth(0) == 64 )
  { 
    bTestAMP_Hor = false;
    bTestAMP_Ver = false;
  }    
#else
  //! Utilizing the partition size of parent PU        
  if ( eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N )
  { 
    bTestAMP_Hor = true;
    bTestAMP_Ver = true;
  }

  if ( eParentPartSize == SIZE_2Nx2N )
  { 
    bTestAMP_Hor = false;
    bTestAMP_Ver = false;
  }      
#endif
}
#endif

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
/** Compress a CU block recursively with enabling sub-LCU-level delta QP
 *\param   rpcBestCU
 *\param   rpcTempCU
 *\param   uiDepth
 *\returns Void
 *
 *- for loop of QP value to compress the current CU with all possible QP
*/
#if AMP_ENC_SPEEDUP
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, PartSize eParentPartSize )
#else
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth )
#endif
{
  TComPic* pcPic = rpcBestCU->getPic();

  // get Original YUV data from picture
  m_ppcOrigYuv[uiDepth]->copyFromPicYuv( pcPic->getPicYuvOrg(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU() );

  // variable for Early CU determination
  Bool    bSubBranch = true;

  // variable for Cbf fast mode PU decision
  Bool    doNotBlockPu = true;
  Bool earlyDetectionSkipMode = false;

  Bool bBoundary = false;
  UInt uiLPelX   = rpcBestCU->getCUPelX();
  UInt uiRPelX   = uiLPelX + rpcBestCU->getWidth(0)  - 1;
  UInt uiTPelY   = rpcBestCU->getCUPelY();
  UInt uiBPelY   = uiTPelY + rpcBestCU->getHeight(0) - 1;

  Int iBaseQP = xComputeQP( rpcBestCU, uiDepth );
  Int iMinQP;
  Int iMaxQP;
  Bool isAddLowestQP = false;
  Int lowestQP = -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY();
  
  int div;
  
  if(onlineTrainingIsDone){
      
    //gcorrea 03/03/2014
    RDcost_MSM = RDcost_2Nx2N = RDcost_2NxN = RDcost_Nx2N = RDcost_NxN = RDcost_2NxnU = RDcost_2NxnD = RDcost_nLx2N = RDcost_nRx2N = 0;
    //gcorrea 03/03/2014 END
  }

  if( (g_uiMaxCUWidth>>uiDepth) >= rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    Int idQP = m_pcEncCfg->getMaxDeltaQP();
    iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP-idQP );
    iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP+idQP );
    if ( (rpcTempCU->getSlice()->getSPS()->getUseLossless()) && (lowestQP < iMinQP) && rpcTempCU->getSlice()->getPPS()->getUseDQP() )
    {
      isAddLowestQP = true; 
      iMinQP = iMinQP - 1;
    }
  }
  else
  {
    iMinQP = rpcTempCU->getQP(0);
    iMaxQP = rpcTempCU->getQP(0);
  }

#if RATE_CONTROL_LAMBDA_DOMAIN
  if ( m_pcEncCfg->getUseRateCtrl() )
  {
    iMinQP = m_pcRateCtrl->getRCQP();
    iMaxQP = m_pcRateCtrl->getRCQP();
  }
#else
  if(m_pcEncCfg->getUseRateCtrl())
  {
    Int qp = m_pcRateCtrl->getUnitQP();
    iMinQP  = Clip3( MIN_QP, MAX_QP, qp);
    iMaxQP  = Clip3( MIN_QP, MAX_QP, qp);
  }
#endif

  // If slice start or slice end is within this cu...
  TComSlice * pcSlice = rpcTempCU->getPic()->getSlice(rpcTempCU->getPic()->getCurrSliceIdx());
  Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurStartCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart();
  Bool bSliceEnd = (pcSlice->getSliceSegmentCurEndCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurEndCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart());
  Bool bInsidePicture = ( uiRPelX < rpcBestCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < rpcBestCU->getSlice()->getSPS()->getPicHeightInLumaSamples() );
  // We need to split, so don't try these modes.
  if(!bSliceEnd && !bSliceStart && bInsidePicture )
  {
    for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
    {
      if (isAddLowestQP && (iQP == iMinQP))
      {
        iQP = lowestQP;
      }

      rpcTempCU->initEstData( uiDepth, iQP );

      // do inter modes, SKIP and 2Nx2N
      if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
      {
        // 2Nx2N
        if(m_pcEncCfg->getUseEarlySkipDetection())
        {
		  //gcorrea: 06/09/2013
			saveResData2Nx2N=1;
			xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N );  rpcTempCU->initEstData( uiDepth, iQP );//by Competition for inter_2Nx2N
			saveResData2Nx2N=0;
		  //gcorrea: 06/09/2013 END
        }
        // SKIP
        xCheckRDCostMerge2Nx2N( rpcBestCU, rpcTempCU, &earlyDetectionSkipMode );//by Merge for inter_2Nx2N
		RDcost_MSM = rpcBestCU->getTotalCost();

		//cout << "\tgetMergeFlag: " << rpcBestCU->getMergeFlag( 0 ) << "\tisSkipped: " << rpcBestCU->isSkipped( 0 ) << "\tgetTotalCost: " << rpcBestCU->getTotalCost() << endl;
        rpcTempCU->initEstData( uiDepth, iQP );

        if(!m_pcEncCfg->getUseEarlySkipDetection())
        {
          // 2Nx2N, NxN
			//gcorrea: 06/09/2013
			saveResData2Nx2N=1;
			xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N );  rpcTempCU->initEstData( uiDepth, iQP );
			saveResData2Nx2N=0;
			//gcorrea: 06/09/2013 END
          if(m_pcEncCfg->getUseCbfFastMode())
          {
            doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
          }
        }
      }

      if (isAddLowestQP && (iQP == lowestQP))
      {
        iQP = iMinQP;
      }
    }

#if RATE_CONTROL_LAMBDA_DOMAIN && !M0036_RC_IMPROVEMENT
    if ( uiDepth <= m_addSADDepth )
    {
      m_LCUPredictionSAD += m_temporalSAD;
      m_addSADDepth = uiDepth;
    }
#endif

    if(!earlyDetectionSkipMode)
    {
      for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
      {
        if (isAddLowestQP && (iQP == iMinQP))
        {
          iQP = lowestQP;
        }
        rpcTempCU->initEstData( uiDepth, iQP );

        // do inter modes, NxN, 2NxN, and Nx2N
        if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
        {
          // 2Nx2N, NxN
          if(!( (rpcBestCU->getWidth(0)==8) && (rpcBestCU->getHeight(0)==8) ))
          {
            if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth && doNotBlockPu)
            {
              xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_NxN   );
              rpcTempCU->initEstData( uiDepth, iQP );
            }
          }

          // 2NxN, Nx2N
          if(doNotBlockPu)
          {
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_Nx2N  );
            rpcTempCU->initEstData( uiDepth, iQP );
            if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
            {
              doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
            }
          }
          if(doNotBlockPu)
          {
            xCheckRDCostInter      ( rpcBestCU, rpcTempCU, SIZE_2NxN  );
            rpcTempCU->initEstData( uiDepth, iQP );
            if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxN)
            {
              doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
            }
          }

#if 1
          //! Try AMP (SIZE_2NxnU, SIZE_2NxnD, SIZE_nLx2N, SIZE_nRx2N)
          if( pcPic->getSlice(0)->getSPS()->getAMPAcc(uiDepth) )
          {
#if AMP_ENC_SPEEDUP        
			  //gcorrea 01/11/2013
            //Bool bTestAMP_Hor = false, bTestAMP_Ver = false;
			  Bool bTestAMP_Hor = true, bTestAMP_Ver = true;    
#if AMP_MRG
            Bool bTestMergeAMP_Hor = false, bTestMergeAMP_Ver = false;

			//gcorrea 01/11/2013
            //deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver, bTestMergeAMP_Hor, bTestMergeAMP_Ver);
#else
            deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver);
#endif

            //! Do horizontal AMP
            if ( bTestAMP_Hor )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU );
                rpcTempCU->initEstData( uiDepth, iQP );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD );
                rpcTempCU->initEstData( uiDepth, iQP );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
            }
#if AMP_MRG
            else if ( bTestMergeAMP_Hor ) 
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU, true );
                rpcTempCU->initEstData( uiDepth, iQP );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD, true );
                rpcTempCU->initEstData( uiDepth, iQP );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
            }
#endif

            //! Do horizontal AMP
            if ( bTestAMP_Ver )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N );
                rpcTempCU->initEstData( uiDepth, iQP );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N );
                rpcTempCU->initEstData( uiDepth, iQP );
              }
            }
#if AMP_MRG
            else if ( bTestMergeAMP_Ver )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N, true );
                rpcTempCU->initEstData( uiDepth, iQP );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N, true );
                rpcTempCU->initEstData( uiDepth, iQP );
              }
            }
#endif

#else
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU );
            rpcTempCU->initEstData( uiDepth, iQP );
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD );
            rpcTempCU->initEstData( uiDepth, iQP );
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N );
            rpcTempCU->initEstData( uiDepth, iQP );

            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N );
            rpcTempCU->initEstData( uiDepth, iQP );

#endif
          }    
#endif
        }

        // do normal intra modes
        // speedup for inter frames
        if( rpcBestCU->getSlice()->getSliceType() == I_SLICE || 
          rpcBestCU->getCbf( 0, TEXT_LUMA     ) != 0   ||
          rpcBestCU->getCbf( 0, TEXT_CHROMA_U ) != 0   ||
          rpcBestCU->getCbf( 0, TEXT_CHROMA_V ) != 0     ) // avoid very complex intra if it is unlikely
        {
          xCheckRDCostIntra( rpcBestCU, rpcTempCU, SIZE_2Nx2N );
          rpcTempCU->initEstData( uiDepth, iQP );
          if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
          {
            if( rpcTempCU->getWidth(0) > ( 1 << rpcTempCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() ) )
            {
              xCheckRDCostIntra( rpcBestCU, rpcTempCU, SIZE_NxN   );
              rpcTempCU->initEstData( uiDepth, iQP );
            }
          }
        }

        // test PCM
        if(pcPic->getSlice(0)->getSPS()->getUsePCM()
          && rpcTempCU->getWidth(0) <= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MaxSize())
          && rpcTempCU->getWidth(0) >= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MinSize()) )
        {
          UInt uiRawBits = (2 * g_bitDepthY + g_bitDepthC) * rpcBestCU->getWidth(0) * rpcBestCU->getHeight(0) / 2;
          UInt uiBestBits = rpcBestCU->getTotalBits();
          if((uiBestBits > uiRawBits) || (rpcBestCU->getTotalCost() > m_pcRdCost->calcRdCost(uiRawBits, 0)))
          {
            xCheckIntraPCM (rpcBestCU, rpcTempCU);
            rpcTempCU->initEstData( uiDepth, iQP );
          }
        }
        if (isAddLowestQP && (iQP == lowestQP))
        {
          iQP = iMinQP;
        }
      }
    }

    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeSplitFlag( rpcBestCU, 0, uiDepth, true );
    rpcBestCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
    if(m_pcEncCfg->getUseSBACRD())
    {
      rpcBestCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
    }
    rpcBestCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcBestCU->getTotalBits(), rpcBestCU->getTotalDistortion() );

    // Early CU determination
    if( m_pcEncCfg->getUseEarlyCU() && rpcBestCU->isSkipped(0) )
    {
      bSubBranch = false;
    }
    else
    {
      bSubBranch = true;
    }
  }
  else if(!(bSliceEnd && bInsidePicture))
  {
    bBoundary = true;
#if RATE_CONTROL_LAMBDA_DOMAIN && !M0036_RC_IMPROVEMENT
    m_addSADDepth++;
#endif
  }

  // copy orginal YUV samples to PCM buffer
  if( rpcBestCU->isLosslessCoded(0) && (rpcBestCU->getIPCMFlag(0) == false))
  {
    xFillPCMBuffer(rpcBestCU, m_ppcOrigYuv[uiDepth]);
  }
  if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    Int idQP = m_pcEncCfg->getMaxDeltaQP();
    iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP-idQP );
    iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP+idQP );
    if ( (rpcTempCU->getSlice()->getSPS()->getUseLossless()) && (lowestQP < iMinQP) && rpcTempCU->getSlice()->getPPS()->getUseDQP() )
    {
      isAddLowestQP = true;
      iMinQP = iMinQP - 1;      
    }
  }
  else if( (g_uiMaxCUWidth>>uiDepth) > rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    iMinQP = iBaseQP;
    iMaxQP = iBaseQP;
  }
  else
  {
    Int iStartQP;
    if( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr())
    {
      iStartQP = rpcTempCU->getQP(0);
    }
    else
    {
      UInt uiCurSliceStartPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
      iStartQP = rpcTempCU->getQP(uiCurSliceStartPartIdx);
    }
    iMinQP = iStartQP;
    iMaxQP = iStartQP;
  }
#if RATE_CONTROL_LAMBDA_DOMAIN
  if ( m_pcEncCfg->getUseRateCtrl() )
  {
    iMinQP = m_pcRateCtrl->getRCQP();
    iMaxQP = m_pcRateCtrl->getRCQP();
  }
#else
  if(m_pcEncCfg->getUseRateCtrl())
  {
    Int qp = m_pcRateCtrl->getUnitQP();
    iMinQP  = Clip3( MIN_QP, MAX_QP, qp);
    iMaxQP  = Clip3( MIN_QP, MAX_QP, qp);
  }
#endif
  //gcorrea: 17/10/2013
//double bestCost = rpcBestCU->getTotalCost();
int mode = rpcBestCU->getPredictionMode( 0 );
int part = rpcBestCU->getPartitionSize( 0 );
int divide;
int split = 1;

//mode 0 means inter prediction
//mode 1 means intra prediction
//mode 15 means no prediction
//part 0 means 2Nx2N
//part 1 means 2NxN and so on

if(mode==0 && part==0)
        divide = 0;
else
        divide = 1;
//gcorrea: 17/10/2013 END
    
if(onlineTrainingIsDone){
    
  double diff_NeiDepth = 0;
  if(mode == 0) {
	double med_Above = -1;
	double med_AboveLeft = -1;
	double med_AboveRight = -1;
	double med_Left = -1;
	double med_Colocated1 = -1;
	double med_Colocated2 = -1;
	double sum_med = 0;
	double med_med = -1;
	int i;
	int j;
	double sum;
	double a2DIVa1, ABSa2DIVa1;
	int SKIPMergeFlag, MergeFlag;
	
	a2DIVa1 = (double)RDcost_2Nx2N / (double)RDcost_MSM;
	ABSa2DIVa1 = abs((double)RDcost_2Nx2N - (double)RDcost_MSM) / (double)RDcost_MSM;
	
	if(rpcBestCU->getMergeFlag( 0 ))
		MergeFlag = 1;
	else
		MergeFlag = 0;
	if((rpcBestCU->isSkipped( 0 ) && rpcBestCU->getMergeFlag( 0 )))
		SKIPMergeFlag = 1;
	else
		SKIPMergeFlag = 0;
	i = j = sum = 0;
	for(i=0; i<256; i+=4) {
		if(rpcBestCU->getCUAbove() != NULL) {
			sum += (double) ((rpcBestCU->getCUAbove())->getDepth(i));
			j++;
		}
	}
	if(j>0)
		med_Above = sum/j;
	
	i = j = sum = 0;
	for(i=0; i<256; i+=4) {
		if(rpcBestCU->getCUAboveLeft() != NULL) {
			sum += (double) ((rpcBestCU->getCUAboveLeft())->getDepth(i));
			j++;
		}
	}
	if(j>0)
		med_AboveLeft = sum/j;
	
	i = j = sum = 0;
	for(i=0; i<256; i+=4) {
		if(rpcBestCU->getCUAboveRight() != NULL) {
			sum += (double) ((rpcBestCU->getCUAboveRight())->getDepth(i));
			j++;
		}
	}
	if(j>0)
		med_AboveRight = sum/j;
	i = j = sum = 0;
	for(i=0; i<256; i+=4) {
		if(rpcBestCU->getCULeft() != NULL) {
			sum += (double) ((rpcBestCU->getCULeft())->getDepth(i));
			j++;
		}
	}
	if(j>0)
		med_Left = sum/j;
	i = j = sum = 0;
	for(i=0; i<256; i+=4) {
		if(rpcBestCU->getCUColocated(REF_PIC_LIST_0) != NULL) {
			sum += (double) ((rpcBestCU->getCUColocated(REF_PIC_LIST_0))->getDepth(i));
			j++;
		}
	}
	if(j>0)
		med_Colocated1 = sum/j;
	
	i = j = sum = 0;
	for(i=0; i<256; i+=4) {
		if(rpcBestCU->getCUColocated(REF_PIC_LIST_1) != NULL) {
			sum += (double) ((rpcBestCU->getCUColocated(REF_PIC_LIST_1))->getDepth(i));
			j++;
		}
	}
	if(j>0)
		med_Colocated2 = sum/j;
	
	j = 0;
	if(med_Above != -1) {
		sum_med += med_Above;
		j++;
	}
	if(med_AboveLeft != -1) {
		sum_med += med_AboveLeft;
		j++;
	}
	if(med_AboveRight != -1) {
		sum_med += med_AboveRight;
		j++;
	}
	if(med_Left != -1) {
		sum_med += med_Left;
		j++;
	}
	if(med_Colocated1 != -1) {
		sum_med += med_Colocated1;
		j++;
	}
	if(med_Colocated2 != -1) {
		sum_med += med_Colocated2;
		j++;
	}
	if(j>0) {
		med_med = sum_med/j;
		diff_NeiDepth = med_med - uiDepth;
	}
	else {
		med_med = -1;
		diff_NeiDepth = 0;
	}
        
        //////// DECISION TREES FOR CU-SPLITTING EARLY-TERMINATION
        string currentLine;
        stringstream convert;
            convert << RDcost_MSM;
            currentLine += convert.str() + ',';
            convert.str("");
            convert << RDcost_2Nx2N;
            currentLine += convert.str() + ',';
            convert.str("");
            convert << RDcost_2NxN;
            currentLine += convert.str() + ',';
            convert.str("");
            convert << RDcost_Nx2N;
            currentLine += convert.str() + ',';
            convert.str("");
            convert << part;
            currentLine += convert.str() + ',';
            convert.str("");
            convert << rpcBestCU->getMergeFlag(0);
            currentLine += convert.str() + ',';
            convert.str("");
            convert << (rpcBestCU->isSkipped(0) && rpcBestCU->getMergeFlag(0));
            currentLine += convert.str() + ',';
            convert.str("");
            convert << diff_NeiDepth;
            currentLine += convert.str() + ',';
            convert.str("");
            convert << fabs(RDcost_2Nx2N-RDcost_MSM)/RDcost_MSM;
            currentLine += convert.str() + ',';
            convert.str("");
            convert << RDcost_2Nx2N/RDcost_MSM;
            currentLine += convert.str() + ',';
            currentLine +='?';
            //question mark works as a wild card
            // to find if the CU will be split or not
            
            /*
           * RDcost_MSM: continuous. //RDcost_MSM (double)
            RDcost_2Nx2N: continuous. // RDcost_2Nx2N(double)
            RDcost_2NxN: continuous. //RDcost_2NxN
            RDcost_Nx2N: continuous. //RDcost_Nx2N
            part: 0,1,2,3,4,5,6,7.
            MergeFlag: 0,1.
            SkipMergeFlag: 0,1.
            neighDepth_diff: continuous.
            abs(a2-a1)/a1: continuous.
            a2/a1: continuous.
            SplitQuadtree: 0,1.
            */
            //@see http://www.cplusplus.com/reference/string/string/c_str/
          cout << currentLine;
          char * cstr = new char [currentLine.length()+1];
            strcpy (cstr, currentLine.c_str());
            
            
      if(uiDepth == 0) {	// 64x64 Cus
          
            split = c5_64_decoder.classifyCurrentLine(cstr);
            //example: 3866,3445,3815,3199,2,0,0,1.625,0.108898,0.891102,?

        }
        else if(uiDepth == 1) { // in 32x32 CUs
            
            split = c5_32_decoder.classifyCurrentLine(cstr);
            
        }
        else if(uiDepth == 2) { // in 16x16 CUs

           split = c5_16_decoder.classifyCurrentLine(cstr);
            
        }
        //uiDepth == 3, setting split == 1 wont make any difference    
    }
    //gcorrea: 03/03/2014 END 
}else{
//collect data
  if(mode==0) {

          if(uiDepth==0) {
                  if(part==0) {
                          count_64x64_2Nx2N++;
                          if(rpcBestCU->getMergeFlag( 0 )) {
                                  count_64x64_MSM++;  
                                  if(rpcBestCU->isSkipped( 0 )) {
                                          count_64x64_SKIP++;
                                          count_64x64_2Nx2N_SKIP++;
                                  }
                                  else {
                                          count_64x64_MERGE++;
                                          count_64x64_2Nx2N_MERGE++;
                                  }
                          }
                          else
                                  count_64x64_2Nx2N_nonMSM++;
                  }
                  else if(part==1) {
                          count_64x64_2NxN++;
                  }
                  else if(part==2) {
                          count_64x64_Nx2N++;
                  }
                  else if(part==3) {
                          count_64x64_NxN++;
                  }
                  else if(part==4) {
                          count_64x64_2NxnU++;
                  }
                  else if(part==5) {
                          count_64x64_2NxnD++;
                  }
                  else if(part==6) {
                          count_64x64_nLx2N++;
                  }
                  else if(part==7) {
                          count_64x64_nRx2N++;
                  }

                  if(part > 0) {
                          if(rpcBestCU->getMergeFlag( 0 )) {
                                  count_64x64_MSM++;
                                  if(rpcBestCU->isSkipped( 0 )) {
                                          count_64x64_SKIP++;
                                  }
                                  else {
                                          count_64x64_MERGE++;
                                  }
                          }
                  }
          }

          else if(uiDepth==1) {
                  if(part==0) {
                          count_32x32_2Nx2N++;
                          if(rpcBestCU->getMergeFlag( 0 )) {
                                  count_32x32_MSM++;  
                                  if(rpcBestCU->isSkipped( 0 )) {
                                          count_32x32_SKIP++;
                                          count_32x32_2Nx2N_SKIP++;
                                  }
                                  else {
                                          count_32x32_MERGE++;
                                          count_32x32_2Nx2N_MERGE++;
                                  }
                          }
                          else
                                  count_32x32_2Nx2N_nonMSM++;
                  }
                  else if(part==1) {
                          count_32x32_2NxN++;
                  }
                  else if(part==2) {
                          count_32x32_Nx2N++;
                  }
                  else if(part==3) {
                          count_32x32_NxN++;
                  }
                  else if(part==4) {
                          count_32x32_2NxnU++;
                  }
                  else if(part==5) {
                          count_32x32_2NxnD++;
                  }
                  else if(part==6) {
                          count_32x32_nLx2N++;
                  }
                  else if(part==7) {
                          count_32x32_nRx2N++;
                  }

                  if(part > 0) {
                          if(rpcBestCU->getMergeFlag( 0 )) {
                                  count_32x32_MSM++;
                                  if(rpcBestCU->isSkipped( 0 )) {
                                          count_32x32_SKIP++;
                                  }
                                  else {
                                          count_32x32_MERGE++;
                                  }
                          }
                  }
          }

          else if(uiDepth==2) {
                  if(part==0) {
                          count_16x16_2Nx2N++;
                          if(rpcBestCU->getMergeFlag( 0 )) {
                                  count_16x16_MSM++;  
                                  if(rpcBestCU->isSkipped( 0 )) {
                                          count_16x16_SKIP++;
                                          count_16x16_2Nx2N_SKIP++;
                                  }
                                  else {
                                          count_16x16_MERGE++;
                                          count_16x16_2Nx2N_MERGE++;
                                  }
                          }
                          else
                                  count_16x16_2Nx2N_nonMSM++;
                  }
                  else if(part==1) {
                          count_16x16_2NxN++;
                  }
                  else if(part==2) {
                          count_16x16_Nx2N++;
                  }
                  else if(part==3) {
                          count_16x16_NxN++;
                  }
                  else if(part==4) {
                          count_16x16_2NxnU++;
                  }
                  else if(part==5) {
                          count_16x16_2NxnD++;
                  }
                  else if(part==6) {
                          count_16x16_nLx2N++;
                  }
                  else if(part==7) {
                          count_16x16_nRx2N++;
                  }

                  if(part > 0) {
                          if(rpcBestCU->getMergeFlag( 0 )) {
                                  count_16x16_MSM++;
                                  if(rpcBestCU->isSkipped( 0 )) {
                                          count_16x16_SKIP++;
                                  }
                                  else {
                                          count_16x16_MERGE++;
                                  }
                          }
                  }
          }

          else if(uiDepth==3) {
                  if(part==0) {
                          count_8x8_2Nx2N++;
                          if(rpcBestCU->getMergeFlag( 0 )) {
                                  count_8x8_MSM++;  
                                  if(rpcBestCU->isSkipped( 0 )) {
                                          count_8x8_SKIP++;
                                          count_8x8_2Nx2N_SKIP++;
                                  }
                                  else {
                                          count_8x8_MERGE++;
                                          count_8x8_2Nx2N_MERGE++;
                                  }
                          }
                          else
                                  count_8x8_2Nx2N_nonMSM++;
                  }
                  else if(part==1) {
                          count_8x8_2NxN++;
                  }
                  else if(part==2) {
                          count_8x8_Nx2N++;
                  }
                  else if(part==3) {
                          count_8x8_NxN++;
                  }
                  else if(part==4) {
                          count_8x8_2NxnU++;
                  }
                  else if(part==5) {
                          count_8x8_2NxnD++;
                  }
                  else if(part==6) {
                          count_8x8_nLx2N++;
                  }
                  else if(part==7) {
                          count_8x8_nRx2N++;
                  }

                  if(part > 0) {
                          if(rpcBestCU->getMergeFlag( 0 )) {
                                  count_8x8_MSM++;
                                  if(rpcBestCU->isSkipped( 0 )) {
                                          count_8x8_SKIP++;
                                  }
                                  else {
                                          count_8x8_MERGE++;
                                  }
                          }
                  }
          }

  } 
}
 
  for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
  {
    if (isAddLowestQP && (iQP == iMinQP))
    {
      iQP = lowestQP;
    }
    rpcTempCU->initEstData( uiDepth, iQP );

    // further split
    // gcorrea: 04/03/2014		
   if( split == 1) {		
   // gcorrea: 04/03/2014 END
    if( bSubBranch && uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth )
    {
      UChar       uhNextDepth         = uiDepth+1;
      TComDataCU* pcSubBestPartCU     = m_ppcBestCU[uhNextDepth];
      TComDataCU* pcSubTempPartCU     = m_ppcTempCU[uhNextDepth];

      for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )
      {
        pcSubBestPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.
        pcSubTempPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.

        Bool bInSlice = pcSubBestPartCU->getSCUAddr()+pcSubBestPartCU->getTotalNumPart()>pcSlice->getSliceSegmentCurStartCUAddr()&&pcSubBestPartCU->getSCUAddr()<pcSlice->getSliceSegmentCurEndCUAddr();
        if(bInSlice && ( pcSubBestPartCU->getCUPelX() < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( pcSubBestPartCU->getCUPelY() < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
        {
          if( m_bUseSBACRD )
          {
            if ( 0 == uiPartUnitIdx) //initialize RD with previous depth buffer
            {
              m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
            }
            else
            {
              m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]);
            }
          }

#if AMP_ENC_SPEEDUP
          if ( rpcBestCU->isIntra(0) )
          {
            xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth, SIZE_NONE );
          }
          else
          {
            xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth, rpcBestCU->getPartitionSize(0) );
          }
#else
          xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
#endif

          rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );         // Keep best part data to current temporary data.
          xCopyYuv2Tmp( pcSubBestPartCU->getTotalNumPart()*uiPartUnitIdx, uhNextDepth );
        }
        else if (bInSlice)
        {
          pcSubBestPartCU->copyToPic( uhNextDepth );
          rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );
        }
      }

      if( !bBoundary )
      {
        m_pcEntropyCoder->resetBits();
        m_pcEntropyCoder->encodeSplitFlag( rpcTempCU, 0, uiDepth, true );

        rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
        if(m_pcEncCfg->getUseSBACRD())
        {
          rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
        }
      }
      rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

      if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() && rpcTempCU->getSlice()->getPPS()->getUseDQP())
      {
        Bool hasResidual = false;
        for( UInt uiBlkIdx = 0; uiBlkIdx < rpcTempCU->getTotalNumPart(); uiBlkIdx ++)
        {
          if( ( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(uiBlkIdx+rpcTempCU->getZorderIdxInCU()) == rpcTempCU->getSlice()->getSliceSegmentCurStartCUAddr() ) && 
              ( rpcTempCU->getCbf( uiBlkIdx, TEXT_LUMA ) || rpcTempCU->getCbf( uiBlkIdx, TEXT_CHROMA_U ) || rpcTempCU->getCbf( uiBlkIdx, TEXT_CHROMA_V ) ) )
          {
            hasResidual = true;
            break;
          }
        }

        UInt uiTargetPartIdx;
        if ( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) != pcSlice->getSliceSegmentCurStartCUAddr() )
        {
          uiTargetPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
        }
        else
        {
          uiTargetPartIdx = 0;
        }
        if ( hasResidual )
        {
#if !RDO_WITHOUT_DQP_BITS
          m_pcEntropyCoder->resetBits();
          m_pcEntropyCoder->encodeQP( rpcTempCU, uiTargetPartIdx, false );
          rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits
          if(m_pcEncCfg->getUseSBACRD())
          {
            rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
          }
          rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
#endif

          Bool foundNonZeroCbf = false;
          rpcTempCU->setQPSubCUs( rpcTempCU->getRefQP( uiTargetPartIdx ), rpcTempCU, 0, uiDepth, foundNonZeroCbf );
          assert( foundNonZeroCbf );
        }
        else
        {
          rpcTempCU->setQPSubParts( rpcTempCU->getRefQP( uiTargetPartIdx ), 0, uiDepth ); // set QP to default QP
        }
      }

      if( m_bUseSBACRD )
      {
        m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
      }
      Bool isEndOfSlice        = rpcBestCU->getSlice()->getSliceMode()==FIXED_NUMBER_OF_BYTES
                                 && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceArgument()<<3);
      Bool isEndOfSliceSegment = rpcBestCU->getSlice()->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES
                                 && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceSegmentArgument()<<3);
      if(isEndOfSlice||isEndOfSliceSegment)
      {
        rpcBestCU->getTotalCost()=rpcTempCU->getTotalCost()+1;
      }
	  //gcorrea: 19/02/2014
      //Temp: CU dividida; Best: CU nao dividida
	  if( rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost() ){
              div = 1;
          }else{
               div = 0;
          }
      
	  //(int) (rpcTempCU->getCUAbove())->getDepth();
	  //gcorrea: 19/02/2014 END

      xCheckBestMode( rpcBestCU, rpcTempCU, uiDepth);                                  // RD compare current larger prediction
    }                                                                                  // with sub partitioned prediction.
  }// gcorrea: 04/03/2014 -- END if(split == 1)
    if (isAddLowestQP && (iQP == lowestQP))
    {
      iQP = iMinQP;
    }
  }

  rpcBestCU->copyToPic(uiDepth);                                                     // Copy Best data to Picture for next partition prediction.

  xCopyYuv2Pic( rpcBestCU->getPic(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU(), uiDepth, uiDepth, rpcBestCU, uiLPelX, uiTPelY );   // Copy Yuv data to picture Yuv

    //gcorrea: 17/02/2014

  //mode 0 means inter prediction
  if(mode==0) {
      int partDir = -1;
      if ((part == 1) || (part == 4) || (part == 5))		// horizontal partitioning
              partDir = 0;
      else if ((part == 2) || (part == 6) || (part == 7))	// vertical partitioning
              partDir = 1;
      else if  (part == 3)									//hor+ver partitioning 
              partDir = 2;

      double med_Above = -1;
      double med_AboveLeft = -1;
      double med_AboveRight = -1;
      double med_Left = -1;
      double med_Colocated1 = -1;
      double med_Colocated2 = -1;
      double sum_med = 0;
      double med_med = -1;
      double diff_NeiDepth = 0;

      int i;
      int j;
      double sum;

      i = j = sum = 0;
      for(i=0; i<256; i+=4) {
              if(rpcBestCU->getCUAbove() != NULL) {
                      sum += (double) ((rpcBestCU->getCUAbove())->getDepth(i));
                      j++;
              }
      }
      if(j>0)
              med_Above = sum/j;

      i = j = sum = 0;
      for(i=0; i<256; i+=4) {
              if(rpcBestCU->getCUAboveLeft() != NULL) {
                      sum += (double) ((rpcBestCU->getCUAboveLeft())->getDepth(i));
                      j++;
              }
      }
      if(j>0)
              med_AboveLeft = sum/j;

      i = j = sum = 0;
      for(i=0; i<256; i+=4) {
              if(rpcBestCU->getCUAboveRight() != NULL) {
                      sum += (double) ((rpcBestCU->getCUAboveRight())->getDepth(i));
                      j++;
              }
      }
      if(j>0)
              med_AboveRight = sum/j;

      i = j = sum = 0;
      for(i=0; i<256; i+=4) {
              if(rpcBestCU->getCULeft() != NULL) {
                      sum += (double) ((rpcBestCU->getCULeft())->getDepth(i));
                      j++;
              }
      }
      if(j>0)
              med_Left = sum/j;

      i = j = sum = 0;
      for(i=0; i<256; i+=4) {
              if(rpcBestCU->getCUColocated(REF_PIC_LIST_0) != NULL) {
                      sum += (double) ((rpcBestCU->getCUColocated(REF_PIC_LIST_0))->getDepth(i));
                      j++;
              }
      }
      if(j>0)
              med_Colocated1 = sum/j;

      i = j = sum = 0;
      for(i=0; i<256; i+=4) {
              if(rpcBestCU->getCUColocated(REF_PIC_LIST_1) != NULL) {
                      sum += (double) ((rpcBestCU->getCUColocated(REF_PIC_LIST_1))->getDepth(i));
                      j++;
              }
      }
      if(j>0)
              med_Colocated2 = sum/j;


      j = 0;
      if(med_Above != -1) {
              sum_med += med_Above;
              j++;
      }
      if(med_AboveLeft != -1) {
              sum_med += med_AboveLeft;
              j++;
      }
      if(med_AboveRight != -1) {
              sum_med += med_AboveRight;
              j++;
      }
      if(med_Left != -1) {
              sum_med += med_Left;
              j++;
      }
      if(med_Colocated1 != -1) {
              sum_med += med_Colocated1;
              j++;
      }
      if(med_Colocated2 != -1) {
              sum_med += med_Colocated2;
              j++;
      }

      if(j>0) {
              med_med = sum_med/j;
              diff_NeiDepth = med_med - uiDepth;
      }
      else {
              med_med = -1;
              diff_NeiDepth = 0;
      }

      //cout << med_Above << '\t' << med_AboveLeft << '\t' << med_AboveRight << '\t' << med_Left << '\t' << med_Colocated1 << '\t' << med_Colocated2 << '\t' << sum_med << '\t' << med_med << '\t' << diff_NeiDepth << endl;
      // pargles April 28th, 2015
      
      if (count_frame < GOPforC5) {
        
          string currentLineVector;
        
        if(uiDepth == 0 || uiDepth == 1 || uiDepth == 2 ){
              
          stringstream convert;
            convert << RDcost_MSM;
            currentLineVector += convert.str() + ',';
            convert.str("");
            convert << RDcost_2Nx2N;
            currentLineVector += convert.str() + ',';
            convert.str("");
            convert << RDcost_2NxN;
            currentLineVector += convert.str() + ',';
            convert.str("");
            convert << RDcost_Nx2N;
            currentLineVector += convert.str() + ',';
            convert.str("");
            convert << part;
            currentLineVector += convert.str() + ',';
            convert.str("");
            convert << rpcBestCU->getMergeFlag(0);
            currentLineVector += convert.str() + ',';
            convert.str("");
            convert << (rpcBestCU->isSkipped(0) && rpcBestCU->getMergeFlag(0));
            currentLineVector += convert.str() + ',';
            convert.str("");
            convert << diff_NeiDepth;
            currentLineVector += convert.str() + ',';
            convert.str("");
            convert << fabs(RDcost_2Nx2N-RDcost_MSM)/RDcost_MSM;
            currentLineVector += convert.str() + ',';
            convert.str("");
            convert << RDcost_2Nx2N/RDcost_MSM;
            currentLineVector += convert.str() + ',';
            convert.str("");
            convert << div;
            currentLineVector += convert.str() + '\n';
            
          }
          
          //todo, what if lasSplitLine or lasNonSplit is empty? What if you get a non split just after 50 split?
            
        if (uiDepth == 0) {
            
            if(div == 1){
                
                split64x64CuOrNotCounter ++;
                last64x64SplitLineVector = currentLineVector;
            }else{
                
                split64x64CuOrNotCounter --;
                last64x64NonSplitLineVector = currentLineVector;
            }
            
            // example 11 splits in the last 11 vectors
            //positive numbers count splits and negative numbers counts non split
            if(split64x64CuOrNotCounter > disparityLimitOfLineBeforeBalance){
                
                //copy last vector with non split
                if(!last64x64NonSplitLineVector.empty()){
                    currentLineVector =  last64x64NonSplitLineVector;
                    split64x64CuOrNotCounter -= 2;
                }
                 
            }else if(split64x64CuOrNotCounter < -disparityLimitOfLineBeforeBalance){
                
                //copy last vector with split
                if(!last64x64SplitLineVector.empty()){
                    currentLineVector = last64x64SplitLineVector;
                    split64x64CuOrNotCounter += 2;
                }
            }
            
            cu64x64forC5 += currentLineVector;
            
        } else if (uiDepth == 1) {
            
            if(div == 1){
                
                split32x32CuOrNotCounter ++;
                last32x32SplitLineVector = currentLineVector;
            }else{
                
                split32x32CuOrNotCounter --;
                last32x32NonSplitLineVector = currentLineVector;
            }
            
            // example 11 splits in the last 11 vectors
            //positive numbers count splits and negative numbers counts non split
            if(split32x32CuOrNotCounter > disparityLimitOfLineBeforeBalance){
                
                //copy last vector with non split
                if(!last32x32NonSplitLineVector.empty()){
                    currentLineVector =  last32x32NonSplitLineVector;
                    split32x32CuOrNotCounter -= 2;
                }
                
            }else if(split32x32CuOrNotCounter < -disparityLimitOfLineBeforeBalance){
                
                //copy last vector with split
                if(!last32x32SplitLineVector.empty()){
                    currentLineVector = last32x32SplitLineVector;
                    split32x32CuOrNotCounter += 2;
                } 
            }
            
            cu32x32forC5 += currentLineVector;
            
        } else if (uiDepth == 2) {
            
            if(div == 1){
                
                split16x16CuOrNotCounter ++;
                last16x16SplitLineVector = currentLineVector;
            }else{
                
                split16x16CuOrNotCounter --;
                last16x16NonSplitLineVector = currentLineVector;
            }
            
            // example 11 splits in the last 11 vectors
            //positive numbers count splits and negative numbers counts non split
            if(split16x16CuOrNotCounter > disparityLimitOfLineBeforeBalance){
                
                //copy last vector with non split
                if(!last16x16NonSplitLineVector.empty()){
                    currentLineVector =  last16x16NonSplitLineVector;
                    split16x16CuOrNotCounter -= 2;
                } 
                
            }else if(split16x16CuOrNotCounter < -disparityLimitOfLineBeforeBalance){
                
                //copy last vector with split
                if(!last16x16SplitLineVector.empty()){
                    currentLineVector = last16x16SplitLineVector;
                    split16x16CuOrNotCounter += 2;
                }
            }
            
            cu16x16forC5 += currentLineVector;
        }
      }
      // END pargles April 28th, 2015
  }
 
  if( bBoundary ||(bSliceEnd && bInsidePicture))
  {
    return;
  }

  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  assert( rpcBestCU->getPartitionSize ( 0 ) != SIZE_NONE  );
  assert( rpcBestCU->getPredictionMode( 0 ) != MODE_NONE  );
  assert( rpcBestCU->getTotalCost     (   ) != MAX_DOUBLE );
}

/** finish encoding a cu and handle end-of-slice conditions
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth 
 * \returns Void
 */
Void TEncCu::finishCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  TComPic* pcPic = pcCU->getPic();
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());

  //Calculate end address
  UInt uiCUAddr = pcCU->getSCUAddr()+uiAbsPartIdx;

  UInt uiInternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) % pcPic->getNumPartInCU();
  UInt uiExternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) / pcPic->getNumPartInCU();
  UInt uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
  UInt uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
  UInt uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
  UInt uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
  while(uiPosX>=uiWidth||uiPosY>=uiHeight)
  {
    uiInternalAddress--;
    uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
    uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
  }
  uiInternalAddress++;
  if(uiInternalAddress==pcCU->getPic()->getNumPartInCU())
  {
    uiInternalAddress = 0;
    uiExternalAddress = pcPic->getPicSym()->getCUOrderMap(pcPic->getPicSym()->getInverseCUOrderMap(uiExternalAddress)+1);
  }
  UInt uiRealEndAddress = pcPic->getPicSym()->getPicSCUEncOrder(uiExternalAddress*pcPic->getNumPartInCU()+uiInternalAddress);

  // Encode slice finish
  Bool bTerminateSlice = false;
  if (uiCUAddr+(pcCU->getPic()->getNumPartInCU()>>(uiDepth<<1)) == uiRealEndAddress)
  {
    bTerminateSlice = true;
  }
  UInt uiGranularityWidth = g_uiMaxCUWidth;
  uiPosX = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  uiPosY = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  Bool granularityBoundary=((uiPosX+pcCU->getWidth(uiAbsPartIdx))%uiGranularityWidth==0||(uiPosX+pcCU->getWidth(uiAbsPartIdx)==uiWidth))
    &&((uiPosY+pcCU->getHeight(uiAbsPartIdx))%uiGranularityWidth==0||(uiPosY+pcCU->getHeight(uiAbsPartIdx)==uiHeight));
  
  if(granularityBoundary)
  {
    // The 1-terminating bit is added to all streams, so don't add it here when it's 1.
    if (!bTerminateSlice)
      m_pcEntropyCoder->encodeTerminatingBit( bTerminateSlice ? 1 : 0 );
  }
  
  Int numberOfWrittenBits = 0;
  if (m_pcBitCounter)
  {
    numberOfWrittenBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  }
  
  // Calculate slice end IF this CU puts us over slice bit size.
  UInt iGranularitySize = pcCU->getPic()->getNumPartInCU();
  Int iGranularityEnd = ((pcCU->getSCUAddr()+uiAbsPartIdx)/iGranularitySize)*iGranularitySize;
  if(iGranularityEnd<=pcSlice->getSliceSegmentCurStartCUAddr()) 
  {
    iGranularityEnd+=max(iGranularitySize,(pcCU->getPic()->getNumPartInCU()>>(uiDepth<<1)));
  }
  // Set slice end parameter
  if(pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES&&!pcSlice->getFinalized()&&pcSlice->getSliceBits()+numberOfWrittenBits>pcSlice->getSliceArgument()<<3) 
  {
    pcSlice->setSliceSegmentCurEndCUAddr(iGranularityEnd);
    pcSlice->setSliceCurEndCUAddr(iGranularityEnd);
    return;
  }
  // Set dependent slice end parameter
  if(pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES&&!pcSlice->getFinalized()&&pcSlice->getSliceSegmentBits()+numberOfWrittenBits > pcSlice->getSliceSegmentArgument()<<3) 
  {
    pcSlice->setSliceSegmentCurEndCUAddr(iGranularityEnd);
    return;
  }
  if(granularityBoundary)
  {
    pcSlice->setSliceBits( (UInt)(pcSlice->getSliceBits() + numberOfWrittenBits) );
    pcSlice->setSliceSegmentBits(pcSlice->getSliceSegmentBits()+numberOfWrittenBits);
    if (m_pcBitCounter)
    {
      m_pcEntropyCoder->resetBits();      
    }
  }
}

/** Compute QP for each CU
 * \param pcCU Target CU
 * \param uiDepth CU depth
 * \returns quantization parameter
 */
Int TEncCu::xComputeQP( TComDataCU* pcCU, UInt uiDepth )
{
  Int iBaseQp = pcCU->getSlice()->getSliceQp();
  Int iQpOffset = 0;
  if ( m_pcEncCfg->getUseAdaptiveQP() )
  {
    TEncPic* pcEPic = dynamic_cast<TEncPic*>( pcCU->getPic() );
    UInt uiAQDepth = min( uiDepth, pcEPic->getMaxAQDepth()-1 );
    TEncPicQPAdaptationLayer* pcAQLayer = pcEPic->getAQLayer( uiAQDepth );
    UInt uiAQUPosX = pcCU->getCUPelX() / pcAQLayer->getAQPartWidth();
    UInt uiAQUPosY = pcCU->getCUPelY() / pcAQLayer->getAQPartHeight();
    UInt uiAQUStride = pcAQLayer->getAQPartStride();
    TEncQPAdaptationUnit* acAQU = pcAQLayer->getQPAdaptationUnit();

    Double dMaxQScale = pow(2.0, m_pcEncCfg->getQPAdaptationRange()/6.0);
    Double dAvgAct = pcAQLayer->getAvgActivity();
    Double dCUAct = acAQU[uiAQUPosY * uiAQUStride + uiAQUPosX].getActivity();
    Double dNormAct = (dMaxQScale*dCUAct + dAvgAct) / (dCUAct + dMaxQScale*dAvgAct);
    Double dQpOffset = log(dNormAct) / log(2.0) * 6.0;
    iQpOffset = Int(floor( dQpOffset + 0.49999 ));
  }
  return Clip3(-pcCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQp+iQpOffset );
}

/** encode a CU block recursively
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth 
 * \returns Void
 */
Void TEncCu::xEncodeCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  TComPic* pcPic = pcCU->getPic();
  
  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;
  
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
  // If slice start is within this cu...
  Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > pcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx && 
    pcSlice->getSliceSegmentCurStartCUAddr() < pcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcPic->getNumPartInCU() >> (uiDepth<<1) );
  // We need to split, so don't try these modes.
  if(!bSliceStart&&( uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
  {
    m_pcEntropyCoder->encodeSplitFlag( pcCU, uiAbsPartIdx, uiDepth );
  }
  else
  {
    bBoundary = true;
  }
  
  if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < (g_uiMaxCUDepth-g_uiAddCUDepth) ) ) || bBoundary )
  {
    UInt uiQNumParts = ( pcPic->getNumPartInCU() >> (uiDepth<<1) )>>2;
    if( (g_uiMaxCUWidth>>uiDepth) == pcCU->getSlice()->getPPS()->getMinCuDQPSize() && pcCU->getSlice()->getPPS()->getUseDQP())
    {
      setdQPFlag(true);
    }
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
      Bool bInSlice = pcCU->getSCUAddr()+uiAbsPartIdx+uiQNumParts>pcSlice->getSliceSegmentCurStartCUAddr()&&pcCU->getSCUAddr()+uiAbsPartIdx<pcSlice->getSliceSegmentCurEndCUAddr();
      if(bInSlice&&( uiLPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
      {
        xEncodeCU( pcCU, uiAbsPartIdx, uiDepth+1 );
      }
    }
    return;
  }
  
  if( (g_uiMaxCUWidth>>uiDepth) >= pcCU->getSlice()->getPPS()->getMinCuDQPSize() && pcCU->getSlice()->getPPS()->getUseDQP())
  {
    setdQPFlag(true);
  }
  if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( pcCU, uiAbsPartIdx );
  }
  if( !pcCU->getSlice()->isIntra() )
  {
    m_pcEntropyCoder->encodeSkipFlag( pcCU, uiAbsPartIdx );
  }
  
  if( pcCU->isSkipped( uiAbsPartIdx ) )
  {
    m_pcEntropyCoder->encodeMergeIndex( pcCU, uiAbsPartIdx );
    finishCU(pcCU,uiAbsPartIdx,uiDepth);
    return;
  }
  m_pcEntropyCoder->encodePredMode( pcCU, uiAbsPartIdx );
  
  m_pcEntropyCoder->encodePartSize( pcCU, uiAbsPartIdx, uiDepth );
  
  if (pcCU->isIntra( uiAbsPartIdx ) && pcCU->getPartitionSize( uiAbsPartIdx ) == SIZE_2Nx2N )
  {
    m_pcEntropyCoder->encodeIPCMInfo( pcCU, uiAbsPartIdx );

    if(pcCU->getIPCMFlag(uiAbsPartIdx))
    {
      // Encode slice finish
      finishCU(pcCU,uiAbsPartIdx,uiDepth);
      return;
    }
  }

  // prediction Info ( Intra : direction mode, Inter : Mv, reference idx )
  m_pcEntropyCoder->encodePredInfo( pcCU, uiAbsPartIdx );
  
  // Encode Coefficients
  Bool bCodeDQP = getdQPFlag();
  m_pcEntropyCoder->encodeCoeff( pcCU, uiAbsPartIdx, uiDepth, pcCU->getWidth (uiAbsPartIdx), pcCU->getHeight(uiAbsPartIdx), bCodeDQP );
  setdQPFlag( bCodeDQP );

  // --- write terminating bit ---
  finishCU(pcCU,uiAbsPartIdx,uiDepth);
}

#if RATE_CONTROL_INTRA
Int xCalcHADs8x8_ISlice(Pel *piOrg, Int iStrideOrg) 
{
  Int k, i, j, jj;
  Int diff[64], m1[8][8], m2[8][8], m3[8][8], iSumHad = 0;

  for( k = 0; k < 64; k += 8 )
  {
    diff[k+0] = piOrg[0] ;
    diff[k+1] = piOrg[1] ;
    diff[k+2] = piOrg[2] ;
    diff[k+3] = piOrg[3] ;
    diff[k+4] = piOrg[4] ;
    diff[k+5] = piOrg[5] ;
    diff[k+6] = piOrg[6] ;
    diff[k+7] = piOrg[7] ;
 
    piOrg += iStrideOrg;
  }
  
  //horizontal
  for (j=0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj  ] + diff[jj+4];
    m2[j][1] = diff[jj+1] + diff[jj+5];
    m2[j][2] = diff[jj+2] + diff[jj+6];
    m2[j][3] = diff[jj+3] + diff[jj+7];
    m2[j][4] = diff[jj  ] - diff[jj+4];
    m2[j][5] = diff[jj+1] - diff[jj+5];
    m2[j][6] = diff[jj+2] - diff[jj+6];
    m2[j][7] = diff[jj+3] - diff[jj+7];
    
    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];
    
    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }
  
  //vertical
  for (i=0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];
    
    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];
    
    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }
  
  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      iSumHad += abs(m2[i][j]);
    }
  }
  iSumHad -= abs(m2[0][0]);
  iSumHad =(iSumHad+2)>>2;
  return(iSumHad);
}

Int  TEncCu::updateLCUDataISlice(TComDataCU* pcCU, Int LCUIdx, Int width, Int height)
{
  Int  xBl, yBl; 
  const Int iBlkSize = 8;

  Pel* pOrgInit   = pcCU->getPic()->getPicYuvOrg()->getLumaAddr(pcCU->getAddr(), 0);
  Int  iStrideOrig = pcCU->getPic()->getPicYuvOrg()->getStride();
  Pel  *pOrg;

  Int iSumHad = 0;
  for ( yBl=0; (yBl+iBlkSize)<=height; yBl+= iBlkSize)
  {
    for ( xBl=0; (xBl+iBlkSize)<=width; xBl+= iBlkSize)
    {
      pOrg = pOrgInit + iStrideOrig*yBl + xBl; 
      iSumHad += xCalcHADs8x8_ISlice(pOrg, iStrideOrig);
    }
  }
  return(iSumHad);
}
#endif

/** check RD costs for a CU block encoded with merge
 * \param rpcBestCU
 * \param rpcTempCU
 * \returns Void
 */
Void TEncCu::xCheckRDCostMerge2Nx2N( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, Bool *earlyDetectionSkipMode )
{
  assert( rpcTempCU->getSlice()->getSliceType() != I_SLICE );
  TComMvField  cMvFieldNeighbours[ 2 * MRG_MAX_NUM_CANDS ]; // double length for mv of both lists
  UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
  Int numValidMergeCand = 0;

  for( UInt ui = 0; ui < rpcTempCU->getSlice()->getMaxNumMergeCand(); ++ui )
  {
    uhInterDirNeighbours[ui] = 0;
  }
  UChar uhDepth = rpcTempCU->getDepth( 0 );
  rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
  rpcTempCU->setCUTransquantBypassSubParts( m_pcEncCfg->getCUTransquantBypassFlagValue(), 0, uhDepth );
  rpcTempCU->getInterMergeCandidates( 0, 0, cMvFieldNeighbours,uhInterDirNeighbours, numValidMergeCand );
  
  Int mergeCandBuffer[MRG_MAX_NUM_CANDS];
  for( UInt ui = 0; ui < numValidMergeCand; ++ui )
  {
    mergeCandBuffer[ui] = 0;
  }

  Bool bestIsSkip = false;

  UInt iteration;
  if ( rpcTempCU->isLosslessCoded(0))
  {
    iteration = 1;
  }
  else 
  {
    iteration = 2;
  }

  for( UInt uiNoResidual = 0; uiNoResidual < iteration; ++uiNoResidual )
  {
    for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
    {
      if(!(uiNoResidual==1 && mergeCandBuffer[uiMergeCand]==1))
      {
        if( !(bestIsSkip && uiNoResidual == 0) )
        {
          // set MC parameters
          rpcTempCU->setPredModeSubParts( MODE_INTER, 0, uhDepth ); // interprets depth relative to LCU level
          rpcTempCU->setCUTransquantBypassSubParts( m_pcEncCfg->getCUTransquantBypassFlagValue(),     0, uhDepth );
          rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
          rpcTempCU->setMergeFlagSubParts( true, 0, 0, uhDepth ); // interprets depth relative to LCU level
          rpcTempCU->setMergeIndexSubParts( uiMergeCand, 0, 0, uhDepth ); // interprets depth relative to LCU level
          rpcTempCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeCand], 0, 0, uhDepth ); // interprets depth relative to LCU level
          rpcTempCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level
          rpcTempCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level
          
          // do MC
          m_pcPredSearch->motionCompensation ( rpcTempCU, m_ppcPredYuvTemp[uhDepth] );
          // estimate residual and encode everything
          m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU,
                                                    m_ppcOrigYuv    [uhDepth],
                                                    m_ppcPredYuvTemp[uhDepth],
                                                    m_ppcResiYuvTemp[uhDepth],
                                                    m_ppcResiYuvBest[uhDepth],
                                                    m_ppcRecoYuvTemp[uhDepth],
                                                    (uiNoResidual? true:false));
          
          
          if ( uiNoResidual == 0 && rpcTempCU->getQtRootCbf(0) == 0 )
          {
            // If no residual when allowing for one, then set mark to not try case where residual is forced to 0
            mergeCandBuffer[uiMergeCand] = 1;
          }
          
          rpcTempCU->setSkipFlagSubParts( rpcTempCU->getQtRootCbf(0) == 0, 0, uhDepth );
          Int orgQP = rpcTempCU->getQP( 0 );
          xCheckDQP( rpcTempCU );
          xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth);
          rpcTempCU->initEstData( uhDepth, orgQP );
          
          if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip )
          {
            bestIsSkip = rpcBestCU->getQtRootCbf(0) == 0;
          }
        }
      }
    }
    
    if(uiNoResidual == 0 && m_pcEncCfg->getUseEarlySkipDetection())
    {
      if(rpcBestCU->getQtRootCbf( 0 ) == 0)
      {
        if( rpcBestCU->getMergeFlag( 0 ))
        {
          *earlyDetectionSkipMode = true;
        }
        else
        {
          Int absoulte_MV=0;
          for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if ( rpcBestCU->getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              TComCUMvField* pcCUMvField = rpcBestCU->getCUMvField(RefPicList( uiRefListIdx ));
              Int iHor = pcCUMvField->getMvd( 0 ).getAbsHor();
              Int iVer = pcCUMvField->getMvd( 0 ).getAbsVer();
              absoulte_MV+=iHor+iVer;
            }
          }
          
          if(absoulte_MV == 0)
          {
            *earlyDetectionSkipMode = true;
          }
        }
      }
    }
  }
}


#if AMP_MRG
Void TEncCu::xCheckRDCostInter( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize, Bool bUseMRG)
#else
Void TEncCu::xCheckRDCostInter( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize )
#endif
{
  UChar uhDepth = rpcTempCU->getDepth( 0 );
  
  rpcTempCU->setDepthSubParts( uhDepth, 0 );
  
  rpcTempCU->setSkipFlagSubParts( false, 0, uhDepth );

  rpcTempCU->setPartSizeSubParts  ( ePartSize,  0, uhDepth );
  rpcTempCU->setPredModeSubParts  ( MODE_INTER, 0, uhDepth );
  rpcTempCU->setCUTransquantBypassSubParts  ( m_pcEncCfg->getCUTransquantBypassFlagValue(),      0, uhDepth );
  
#if AMP_MRG
  rpcTempCU->setMergeAMP (true);
  m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth], false, bUseMRG );
#else  
  m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth] );
#endif

#if AMP_MRG
  if ( !rpcTempCU->getMergeAMP() )
  {
    return;
  }
#endif

#if RATE_CONTROL_LAMBDA_DOMAIN && !M0036_RC_IMPROVEMENT
  if ( m_pcEncCfg->getUseRateCtrl() && m_pcEncCfg->getLCULevelRC() && ePartSize == SIZE_2Nx2N && uhDepth <= m_addSADDepth )
  {
    UInt SAD = m_pcRdCost->getSADPart( g_bitDepthY, m_ppcPredYuvTemp[uhDepth]->getLumaAddr(), m_ppcPredYuvTemp[uhDepth]->getStride(),
      m_ppcOrigYuv[uhDepth]->getLumaAddr(), m_ppcOrigYuv[uhDepth]->getStride(),
      rpcTempCU->getWidth(0), rpcTempCU->getHeight(0) );
    m_temporalSAD = (Int)SAD;
  }
#endif

  m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcResiYuvBest[uhDepth], m_ppcRecoYuvTemp[uhDepth], false );
  rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

  xCheckDQP( rpcTempCU );
    //gcorrea 01/11/2013
   if(ePartSize==SIZE_2Nx2N)
           RDcost_2Nx2N = rpcTempCU->getTotalCost();
   else if(ePartSize==SIZE_2NxN)
           RDcost_2NxN = rpcTempCU->getTotalCost();
   else if(ePartSize==SIZE_Nx2N)
           RDcost_Nx2N = rpcTempCU->getTotalCost();
   else if(ePartSize==SIZE_NxN)
           RDcost_NxN = rpcTempCU->getTotalCost();
   else if(ePartSize==SIZE_2NxnU)
           RDcost_2NxnU = rpcTempCU->getTotalCost();
   else if(ePartSize==SIZE_2NxnD)
           RDcost_2NxnD = rpcTempCU->getTotalCost();
   else if(ePartSize==SIZE_nLx2N)
           RDcost_nLx2N = rpcTempCU->getTotalCost();
   else if(ePartSize==SIZE_nRx2N)
           RDcost_nRx2N = rpcTempCU->getTotalCost();
   //gcorrea 01/11/2013 ENDs 
  xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth);
}

Void TEncCu::xCheckRDCostIntra( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize eSize )
{
  UInt uiDepth = rpcTempCU->getDepth( 0 );
  
  rpcTempCU->setSkipFlagSubParts( false, 0, uiDepth );

  rpcTempCU->setPartSizeSubParts( eSize, 0, uiDepth );
  rpcTempCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );
  rpcTempCU->setCUTransquantBypassSubParts( m_pcEncCfg->getCUTransquantBypassFlagValue(), 0, uiDepth );
  
  Bool bSeparateLumaChroma = true; // choose estimation mode
  UInt uiPreCalcDistC      = 0;
  if( !bSeparateLumaChroma )
  {
    m_pcPredSearch->preestChromaPredMode( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth] );
  }
  m_pcPredSearch  ->estIntraPredQT      ( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth], uiPreCalcDistC, bSeparateLumaChroma );

  m_ppcRecoYuvTemp[uiDepth]->copyToPicLuma(rpcTempCU->getPic()->getPicYuvRec(), rpcTempCU->getAddr(), rpcTempCU->getZorderIdxInCU() );
  
  m_pcPredSearch  ->estIntraPredChromaQT( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth], uiPreCalcDistC );
  
  m_pcEntropyCoder->resetBits();
  if ( rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( rpcTempCU, 0,          true );
  }
  m_pcEntropyCoder->encodeSkipFlag ( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePredMode( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePartSize( rpcTempCU, 0, uiDepth, true );
  m_pcEntropyCoder->encodePredInfo( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodeIPCMInfo(rpcTempCU, 0, true );

  // Encode Coefficients
  Bool bCodeDQP = getdQPFlag();
  m_pcEntropyCoder->encodeCoeff( rpcTempCU, 0, uiDepth, rpcTempCU->getWidth (0), rpcTempCU->getHeight(0), bCodeDQP );
  setdQPFlag( bCodeDQP );
  
  if( m_bUseSBACRD ) m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
  
  rpcTempCU->getTotalBits() = m_pcEntropyCoder->getNumberOfWrittenBits();
  if(m_pcEncCfg->getUseSBACRD())
  {
    rpcTempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
  }
  rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
  
  xCheckDQP( rpcTempCU );
  xCheckBestMode(rpcBestCU, rpcTempCU, uiDepth);
}

/** Check R-D costs for a CU with PCM mode. 
 * \param rpcBestCU pointer to best mode CU data structure
 * \param rpcTempCU pointer to testing mode CU data structure
 * \returns Void
 * 
 * \note Current PCM implementation encodes sample values in a lossless way. The distortion of PCM mode CUs are zero. PCM mode is selected if the best mode yields bits greater than that of PCM mode.
 */
Void TEncCu::xCheckIntraPCM( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU )
{
  UInt uiDepth = rpcTempCU->getDepth( 0 );

  rpcTempCU->setSkipFlagSubParts( false, 0, uiDepth );

  rpcTempCU->setIPCMFlag(0, true);
  rpcTempCU->setIPCMFlagSubParts (true, 0, rpcTempCU->getDepth(0));
  rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth );
  rpcTempCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );
  rpcTempCU->setTrIdxSubParts ( 0, 0, uiDepth );
  rpcTempCU->setCUTransquantBypassSubParts( m_pcEncCfg->getCUTransquantBypassFlagValue(), 0, uiDepth );

  m_pcPredSearch->IPCMSearch( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth]);

  if( m_bUseSBACRD ) m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);

  m_pcEntropyCoder->resetBits();
  if ( rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( rpcTempCU, 0,          true );
  }
  m_pcEntropyCoder->encodeSkipFlag ( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePredMode ( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePartSize ( rpcTempCU, 0, uiDepth, true );
  m_pcEntropyCoder->encodeIPCMInfo ( rpcTempCU, 0, true );

  if( m_bUseSBACRD ) m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);

  rpcTempCU->getTotalBits() = m_pcEntropyCoder->getNumberOfWrittenBits();
  if(m_pcEncCfg->getUseSBACRD())
  {
    rpcTempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
  }
  rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

  xCheckDQP( rpcTempCU );
  xCheckBestMode( rpcBestCU, rpcTempCU, uiDepth );
}

/** check whether current try is the best with identifying the depth of current try
 * \param rpcBestCU
 * \param rpcTempCU
 * \returns Void
 */
Void TEncCu::xCheckBestMode( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth )
{
  if( rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost() )
  {
    TComYuv* pcYuv;
    // Change Information data
    TComDataCU* pcCU = rpcBestCU;
    rpcBestCU = rpcTempCU;
    rpcTempCU = pcCU;

    // Change Prediction data
    pcYuv = m_ppcPredYuvBest[uiDepth];
    m_ppcPredYuvBest[uiDepth] = m_ppcPredYuvTemp[uiDepth];
    m_ppcPredYuvTemp[uiDepth] = pcYuv;

    // Change Reconstruction data
    pcYuv = m_ppcRecoYuvBest[uiDepth];
    m_ppcRecoYuvBest[uiDepth] = m_ppcRecoYuvTemp[uiDepth];
    m_ppcRecoYuvTemp[uiDepth] = pcYuv;

    pcYuv = NULL;
    pcCU  = NULL;

    if( m_bUseSBACRD )  // store temp best CI for next CU coding
      m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);
  }
}

Void TEncCu::xCheckDQP( TComDataCU* pcCU )
{
  UInt uiDepth = pcCU->getDepth( 0 );

  if( pcCU->getSlice()->getPPS()->getUseDQP() && (g_uiMaxCUWidth>>uiDepth) >= pcCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    if ( pcCU->getCbf( 0, TEXT_LUMA, 0 ) || pcCU->getCbf( 0, TEXT_CHROMA_U, 0 ) || pcCU->getCbf( 0, TEXT_CHROMA_V, 0 ) )
    {
#if !RDO_WITHOUT_DQP_BITS
      m_pcEntropyCoder->resetBits();
      m_pcEntropyCoder->encodeQP( pcCU, 0, false );
      pcCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits
      if(m_pcEncCfg->getUseSBACRD())
      {
        pcCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
      }
      pcCU->getTotalCost() = m_pcRdCost->calcRdCost( pcCU->getTotalBits(), pcCU->getTotalDistortion() );
#endif
    }
    else
    {
      pcCU->setQPSubParts( pcCU->getRefQP( 0 ), 0, uiDepth ); // set QP to default QP
    }
  }
}

Void TEncCu::xCopyAMVPInfo (AMVPInfo* pSrc, AMVPInfo* pDst)
{
  pDst->iN = pSrc->iN;
  for (Int i = 0; i < pSrc->iN; i++)
  {
    pDst->m_acMvCand[i] = pSrc->m_acMvCand[i];
  }
}
Void TEncCu::xCopyYuv2Pic(TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth, TComDataCU* pcCU, UInt uiLPelX, UInt uiTPelY )
{
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
  Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx && 
    pcSlice->getSliceSegmentCurStartCUAddr() < rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) );
  Bool bSliceEnd   = pcSlice->getSliceSegmentCurEndCUAddr() > rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx && 
    pcSlice->getSliceSegmentCurEndCUAddr() < rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) );
  if(!bSliceEnd && !bSliceStart && ( uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
  {
    UInt uiAbsPartIdxInRaster = g_auiZscanToRaster[uiAbsPartIdx];
    UInt uiSrcBlkWidth = rpcPic->getNumPartInWidth() >> (uiSrcDepth);
    UInt uiBlkWidth    = rpcPic->getNumPartInWidth() >> (uiDepth);
    UInt uiPartIdxX = ( ( uiAbsPartIdxInRaster % rpcPic->getNumPartInWidth() ) % uiSrcBlkWidth) / uiBlkWidth;
    UInt uiPartIdxY = ( ( uiAbsPartIdxInRaster / rpcPic->getNumPartInWidth() ) % uiSrcBlkWidth) / uiBlkWidth;
    UInt uiPartIdx = uiPartIdxY * ( uiSrcBlkWidth / uiBlkWidth ) + uiPartIdxX;
    m_ppcRecoYuvBest[uiSrcDepth]->copyToPicYuv( rpcPic->getPicYuvRec (), uiCUAddr, uiAbsPartIdx, uiDepth - uiSrcDepth, uiPartIdx);
  }
  else
  {
    UInt uiQNumParts = ( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) )>>2;

    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      UInt uiSubCULPelX   = uiLPelX + ( g_uiMaxCUWidth >>(uiDepth+1) )*( uiPartUnitIdx &  1 );
      UInt uiSubCUTPelY   = uiTPelY + ( g_uiMaxCUHeight>>(uiDepth+1) )*( uiPartUnitIdx >> 1 );

      Bool bInSlice = rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+uiQNumParts > pcSlice->getSliceSegmentCurStartCUAddr() && 
        rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx < pcSlice->getSliceSegmentCurEndCUAddr();
      if(bInSlice&&( uiSubCULPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiSubCUTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
      {
        xCopyYuv2Pic( rpcPic, uiCUAddr, uiAbsPartIdx, uiDepth+1, uiSrcDepth, pcCU, uiSubCULPelX, uiSubCUTPelY );   // Copy Yuv data to picture Yuv
      }
    }
  }
}

Void TEncCu::xCopyYuv2Tmp( UInt uiPartUnitIdx, UInt uiNextDepth )
{
  UInt uiCurrDepth = uiNextDepth - 1;
  m_ppcRecoYuvBest[uiNextDepth]->copyToPartYuv( m_ppcRecoYuvTemp[uiCurrDepth], uiPartUnitIdx );
}

/** Function for filling the PCM buffer of a CU using its original sample array 
 * \param pcCU pointer to current CU
 * \param pcOrgYuv pointer to original sample array
 * \returns Void
 */
Void TEncCu::xFillPCMBuffer     ( TComDataCU*& pCU, TComYuv* pOrgYuv )
{

  UInt   width        = pCU->getWidth(0);
  UInt   height       = pCU->getHeight(0);

  Pel*   pSrcY = pOrgYuv->getLumaAddr(0, width); 
  Pel*   pDstY = pCU->getPCMSampleY();
  UInt   srcStride = pOrgYuv->getStride();

  for(Int y = 0; y < height; y++ )
  {
    for(Int x = 0; x < width; x++ )
    {
      pDstY[x] = pSrcY[x];
    }
    pDstY += width;
    pSrcY += srcStride;
  }

  Pel* pSrcCb       = pOrgYuv->getCbAddr();
  Pel* pSrcCr       = pOrgYuv->getCrAddr();;

  Pel* pDstCb       = pCU->getPCMSampleCb();
  Pel* pDstCr       = pCU->getPCMSampleCr();;

  UInt srcStrideC = pOrgYuv->getCStride();
  UInt heightC   = height >> 1;
  UInt widthC    = width  >> 1;

  for(Int y = 0; y < heightC; y++ )
  {
    for(Int x = 0; x < widthC; x++ )
    {
      pDstCb[x] = pSrcCb[x];
      pDstCr[x] = pSrcCr[x];
    }
    pDstCb += widthC;
    pDstCr += widthC;
    pSrcCb += srcStrideC;
    pSrcCr += srcStrideC;
  }
}

#if ADAPTIVE_QP_SELECTION
/** Collect ARL statistics from one block
  */
Int TEncCu::xTuCollectARLStats(TCoeff* rpcCoeff, Int* rpcArlCoeff, Int NumCoeffInCU, Double* cSum, UInt* numSamples )
{
  for( Int n = 0; n < NumCoeffInCU; n++ )
  {
    Int u = abs( rpcCoeff[ n ] );
    Int absc = rpcArlCoeff[ n ];

    if( u != 0 )
    {
      if( u < LEVEL_RANGE )
      {
        cSum[ u ] += ( Double )absc;
        numSamples[ u ]++;
      }
      else 
      {
        cSum[ LEVEL_RANGE ] += ( Double )absc - ( Double )( u << ARL_C_PRECISION );
        numSamples[ LEVEL_RANGE ]++;
      }
    }
  }

  return 0;
}

/** Collect ARL statistics from one LCU
 * \param pcCU
 */
Void TEncCu::xLcuCollectARLStats(TComDataCU* rpcCU )
{
  Double cSum[ LEVEL_RANGE + 1 ];     //: the sum of DCT coefficients corresponding to datatype and quantization output
  UInt numSamples[ LEVEL_RANGE + 1 ]; //: the number of coefficients corresponding to datatype and quantization output

  TCoeff* pCoeffY = rpcCU->getCoeffY();
  Int* pArlCoeffY = rpcCU->getArlCoeffY();

  UInt uiMinCUWidth = g_uiMaxCUWidth >> g_uiMaxCUDepth;
  UInt uiMinNumCoeffInCU = 1 << uiMinCUWidth;

  memset( cSum, 0, sizeof( Double )*(LEVEL_RANGE+1) );
  memset( numSamples, 0, sizeof( UInt )*(LEVEL_RANGE+1) );

  // Collect stats to cSum[][] and numSamples[][]
  for(Int i = 0; i < rpcCU->getTotalNumPart(); i ++ )
  {
    UInt uiTrIdx = rpcCU->getTransformIdx(i);

    if(rpcCU->getPredictionMode(i) == MODE_INTER)
    if( rpcCU->getCbf( i, TEXT_LUMA, uiTrIdx ) )
    {
      xTuCollectARLStats(pCoeffY, pArlCoeffY, uiMinNumCoeffInCU, cSum, numSamples);
    }//Note that only InterY is processed. QP rounding is based on InterY data only.
   
    pCoeffY  += uiMinNumCoeffInCU;
    pArlCoeffY  += uiMinNumCoeffInCU;
  }

  for(Int u=1; u<LEVEL_RANGE;u++)
  {
    m_pcTrQuant->getSliceSumC()[u] += cSum[ u ] ;
    m_pcTrQuant->getSliceNSamples()[u] += numSamples[ u ] ;
  }
  m_pcTrQuant->getSliceSumC()[LEVEL_RANGE] += cSum[ LEVEL_RANGE ] ;
  m_pcTrQuant->getSliceNSamples()[LEVEL_RANGE] += numSamples[ LEVEL_RANGE ] ;
}
#endif
//! \}

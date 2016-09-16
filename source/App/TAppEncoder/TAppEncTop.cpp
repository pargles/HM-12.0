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

/** \file     TAppEncTop.cpp
    \brief    Encoder application class
*/

#include <list>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <assert.h>
#include <math.h>
#include "TAppEncTop.h"
#include "TLibEncoder/AnnexBwrite.h"

using namespace std;

//gcorrea 29/08/2013
extern ofstream CU64x64data;
extern ofstream CU32x32data;
extern ofstream CU16x16data;
extern ofstream CU8x8data;

extern char filename_64x64[100];
extern char filename_32x32[100];
extern char filename_16x16[100];
extern char filename_8x8[100];
extern Char*  relation;

extern Pel **saveLumaPel;
extern Pel **saveHorGrad;
extern Pel **saveVerGrad;

extern int count_frame;

extern int saveResData2Nx2N;
extern double sumRes2Nx2N, medRes2Nx2N, sqdRes2Nx2N, varRes2Nx2N;
extern double res_sum_VP1, res_sum_VP2, res_med_VP1, res_med_VP2, res_sqd_VP1, res_sqd_VP2, res_var_VP1, res_var_VP2;
extern double res_sum_HP1, res_sum_HP2, res_med_HP1, res_med_HP2, res_sqd_HP1, res_sqd_HP2, res_var_HP1, res_var_HP2;
extern double ResHorGrad, ResVerGrad, ResGrad, res_RHV_grad;
extern double res_rhi_V, res_rhi_H, res_rhi_Q;
extern double res_RHV_sum, res_RHV_med, res_RHV_var, res_RHV_HI;

extern int curr_uiDepth;

extern int frameWidth, frameHeight;
extern int nCU_hor, nCU_ver, nCU32x32_hor, nCU32x32_ver, nCU16x16_hor, nCU16x16_ver, nCU8x8_hor, nCU8x8_ver, nCU;

extern long int count_64x64_MSM, count_64x64_MERGE, count_64x64_2Nx2N_MERGE, count_64x64_SKIP, count_64x64_2Nx2N_SKIP, count_64x64_2Nx2N_nonMSM, count_64x64_2Nx2N, count_64x64_2NxN, count_64x64_Nx2N, count_64x64_NxN, count_64x64_2NxnU, count_64x64_2NxnD, count_64x64_nLx2N, count_64x64_nRx2N;
extern long int count_32x32_MSM, count_32x32_MERGE, count_32x32_2Nx2N_MERGE, count_32x32_SKIP, count_32x32_2Nx2N_SKIP, count_32x32_2Nx2N_nonMSM, count_32x32_2Nx2N, count_32x32_2NxN, count_32x32_Nx2N, count_32x32_NxN, count_32x32_2NxnU, count_32x32_2NxnD, count_32x32_nLx2N, count_32x32_nRx2N;
extern long int count_16x16_MSM, count_16x16_MERGE, count_16x16_2Nx2N_MERGE, count_16x16_SKIP, count_16x16_2Nx2N_SKIP, count_16x16_2Nx2N_nonMSM, count_16x16_2Nx2N, count_16x16_2NxN, count_16x16_Nx2N, count_16x16_NxN, count_16x16_2NxnU, count_16x16_2NxnD, count_16x16_nLx2N, count_16x16_nRx2N;
extern long int count_8x8_MSM, count_8x8_MERGE, count_8x8_2Nx2N_MERGE, count_8x8_SKIP, count_8x8_2Nx2N_SKIP, count_8x8_2Nx2N_nonMSM, count_8x8_2Nx2N, count_8x8_2NxN, count_8x8_Nx2N, count_8x8_NxN, count_8x8_2NxnU, count_8x8_2NxnD, count_8x8_nLx2N, count_8x8_nRx2N;
//gcorrea 29/08/2013 END

//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

TAppEncTop::TAppEncTop()
{
  m_iFrameRcvd = 0;
  m_totalBytes = 0;
  m_essentialBytes = 0;
}

TAppEncTop::~TAppEncTop()
{
}

Void TAppEncTop::xInitLibCfg()
{
  TComVPS vps;
  
  vps.setMaxTLayers                       ( m_maxTempLayer );
  if (m_maxTempLayer == 1)
  {
    vps.setTemporalNestingFlag(true);
  }
  vps.setMaxLayers                        ( 1 );
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    vps.setNumReorderPics                 ( m_numReorderPics[i], i );
    vps.setMaxDecPicBuffering             ( m_maxDecPicBuffering[i], i );
  }
  m_cTEncTop.setVPS(&vps);

  m_cTEncTop.setProfile(m_profile);
  m_cTEncTop.setLevel(m_levelTier, m_level);
  m_cTEncTop.setProgressiveSourceFlag(m_progressiveSourceFlag);
  m_cTEncTop.setInterlacedSourceFlag(m_interlacedSourceFlag);
  m_cTEncTop.setNonPackedConstraintFlag(m_nonPackedConstraintFlag);
  m_cTEncTop.setFrameOnlyConstraintFlag(m_frameOnlyConstraintFlag);
  
  m_cTEncTop.setFrameRate                    ( m_iFrameRate );
  m_cTEncTop.setFrameSkip                    ( m_FrameSkip );
  m_cTEncTop.setSourceWidth                  ( m_iSourceWidth );
  m_cTEncTop.setSourceHeight                 ( m_iSourceHeight );
  m_cTEncTop.setConformanceWindow            ( m_confLeft, m_confRight, m_confTop, m_confBottom );
  m_cTEncTop.setFramesToBeEncoded            ( m_framesToBeEncoded );
  
  //====== Coding Structure ========
  m_cTEncTop.setIntraPeriod                  ( m_iIntraPeriod );
  m_cTEncTop.setDecodingRefreshType          ( m_iDecodingRefreshType );
  m_cTEncTop.setGOPSize                      ( m_iGOPSize );
  m_cTEncTop.setGopList                      ( m_GOPList );
  m_cTEncTop.setExtraRPSs                    ( m_extraRPSs );
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    m_cTEncTop.setNumReorderPics             ( m_numReorderPics[i], i );
    m_cTEncTop.setMaxDecPicBuffering         ( m_maxDecPicBuffering[i], i );
  }
  for( UInt uiLoop = 0; uiLoop < MAX_TLAYER; ++uiLoop )
  {
    m_cTEncTop.setLambdaModifier( uiLoop, m_adLambdaModifier[ uiLoop ] );
  }
  m_cTEncTop.setQP                           ( m_iQP );
  
  m_cTEncTop.setPad                          ( m_aiPad );
    
  m_cTEncTop.setMaxTempLayer                 ( m_maxTempLayer );
  m_cTEncTop.setUseAMP( m_enableAMP );
  
  //===== Slice ========
  
  //====== Loop/Deblock Filter ========
  m_cTEncTop.setLoopFilterDisable            ( m_bLoopFilterDisable       );
  m_cTEncTop.setLoopFilterOffsetInPPS        ( m_loopFilterOffsetInPPS );
  m_cTEncTop.setLoopFilterBetaOffset         ( m_loopFilterBetaOffsetDiv2  );
  m_cTEncTop.setLoopFilterTcOffset           ( m_loopFilterTcOffsetDiv2    );
  m_cTEncTop.setDeblockingFilterControlPresent( m_DeblockingFilterControlPresent);
  m_cTEncTop.setDeblockingFilterMetric       ( m_DeblockingFilterMetric );

  //====== Motion search ========
  m_cTEncTop.setFastSearch                   ( m_iFastSearch  );
  m_cTEncTop.setSearchRange                  ( m_iSearchRange );
  m_cTEncTop.setBipredSearchRange            ( m_bipredSearchRange );

  //====== Quality control ========
  m_cTEncTop.setMaxDeltaQP                   ( m_iMaxDeltaQP  );
  m_cTEncTop.setMaxCuDQPDepth                ( m_iMaxCuDQPDepth  );

  m_cTEncTop.setChromaCbQpOffset               ( m_cbQpOffset     );
  m_cTEncTop.setChromaCrQpOffset            ( m_crQpOffset  );

#if ADAPTIVE_QP_SELECTION
  m_cTEncTop.setUseAdaptQpSelect             ( m_bUseAdaptQpSelect   );
#endif

  Int lowestQP;
  lowestQP =  - 6*(g_bitDepthY - 8); // XXX: check

  if ((m_iMaxDeltaQP == 0 ) && (m_iQP == lowestQP) && (m_useLossless == true))
  {
    m_bUseAdaptiveQP = false;
  }
  m_cTEncTop.setUseAdaptiveQP                ( m_bUseAdaptiveQP  );
  m_cTEncTop.setQPAdaptationRange            ( m_iQPAdaptationRange );
  
  //====== Tool list ========
  m_cTEncTop.setUseSBACRD                    ( m_bUseSBACRD   );
  m_cTEncTop.setDeltaQpRD                    ( m_uiDeltaQpRD  );
  m_cTEncTop.setUseASR                       ( m_bUseASR      );
  m_cTEncTop.setUseHADME                     ( m_bUseHADME    );
  m_cTEncTop.setUseLossless                  ( m_useLossless );
  m_cTEncTop.setdQPs                         ( m_aidQP        );
  m_cTEncTop.setUseRDOQ                      ( m_useRDOQ     );
  m_cTEncTop.setUseRDOQTS                    ( m_useRDOQTS   );
  m_cTEncTop.setRDpenalty                 ( m_rdPenalty );
  m_cTEncTop.setQuadtreeTULog2MaxSize        ( m_uiQuadtreeTULog2MaxSize );
  m_cTEncTop.setQuadtreeTULog2MinSize        ( m_uiQuadtreeTULog2MinSize );
  m_cTEncTop.setQuadtreeTUMaxDepthInter      ( m_uiQuadtreeTUMaxDepthInter );
  m_cTEncTop.setQuadtreeTUMaxDepthIntra      ( m_uiQuadtreeTUMaxDepthIntra );
  m_cTEncTop.setUseFastEnc                   ( m_bUseFastEnc  );
  m_cTEncTop.setUseEarlyCU                   ( m_bUseEarlyCU  ); 
  m_cTEncTop.setUseFastDecisionForMerge      ( m_useFastDecisionForMerge  );
  m_cTEncTop.setUseCbfFastMode            ( m_bUseCbfFastMode  );
  m_cTEncTop.setUseEarlySkipDetection            ( m_useEarlySkipDetection );

  m_cTEncTop.setUseTransformSkip             ( m_useTransformSkip      );
  m_cTEncTop.setUseTransformSkipFast         ( m_useTransformSkipFast  );
  m_cTEncTop.setUseConstrainedIntraPred      ( m_bUseConstrainedIntraPred );
  m_cTEncTop.setPCMLog2MinSize          ( m_uiPCMLog2MinSize);
  m_cTEncTop.setUsePCM                       ( m_usePCM );
  m_cTEncTop.setPCMLog2MaxSize               ( m_pcmLog2MaxSize);
  m_cTEncTop.setMaxNumMergeCand              ( m_maxNumMergeCand );
  

  //====== Weighted Prediction ========
  m_cTEncTop.setUseWP                   ( m_useWeightedPred      );
  m_cTEncTop.setWPBiPred                ( m_useWeightedBiPred   );
  //====== Parallel Merge Estimation ========
  m_cTEncTop.setLog2ParallelMergeLevelMinus2 ( m_log2ParallelMergeLevel - 2 );

  //====== Slice ========
  m_cTEncTop.setSliceMode               ( m_sliceMode                );
  m_cTEncTop.setSliceArgument           ( m_sliceArgument            );

  //====== Dependent Slice ========
  m_cTEncTop.setSliceSegmentMode        ( m_sliceSegmentMode         );
  m_cTEncTop.setSliceSegmentArgument    ( m_sliceSegmentArgument     );
  Int iNumPartInCU = 1<<(m_uiMaxCUDepth<<1);
  if(m_sliceSegmentMode==FIXED_NUMBER_OF_LCU)
  {
    m_cTEncTop.setSliceSegmentArgument ( m_sliceSegmentArgument * iNumPartInCU );
  }
  if(m_sliceMode==FIXED_NUMBER_OF_LCU)
  {
    m_cTEncTop.setSliceArgument ( m_sliceArgument * iNumPartInCU );
  }
  if(m_sliceMode==FIXED_NUMBER_OF_TILES)
  {
    m_cTEncTop.setSliceArgument ( m_sliceArgument );
  }
  
  if(m_sliceMode == 0 )
  {
    m_bLFCrossSliceBoundaryFlag = true;
  }
  m_cTEncTop.setLFCrossSliceBoundaryFlag( m_bLFCrossSliceBoundaryFlag );
  m_cTEncTop.setUseSAO ( m_bUseSAO );
  m_cTEncTop.setMaxNumOffsetsPerPic (m_maxNumOffsetsPerPic);

  m_cTEncTop.setSaoLcuBoundary (m_saoLcuBoundary);
  m_cTEncTop.setSaoLcuBasedOptimization (m_saoLcuBasedOptimization);
  m_cTEncTop.setPCMInputBitDepthFlag  ( m_bPCMInputBitDepthFlag); 
  m_cTEncTop.setPCMFilterDisableFlag  ( m_bPCMFilterDisableFlag); 

  m_cTEncTop.setDecodedPictureHashSEIEnabled(m_decodedPictureHashSEIEnabled);
  m_cTEncTop.setRecoveryPointSEIEnabled( m_recoveryPointSEIEnabled );
  m_cTEncTop.setBufferingPeriodSEIEnabled( m_bufferingPeriodSEIEnabled );
  m_cTEncTop.setPictureTimingSEIEnabled( m_pictureTimingSEIEnabled );
  m_cTEncTop.setToneMappingInfoSEIEnabled                 ( m_toneMappingInfoSEIEnabled );
  m_cTEncTop.setTMISEIToneMapId                           ( m_toneMapId );
  m_cTEncTop.setTMISEIToneMapCancelFlag                   ( m_toneMapCancelFlag );
  m_cTEncTop.setTMISEIToneMapPersistenceFlag              ( m_toneMapPersistenceFlag );
  m_cTEncTop.setTMISEICodedDataBitDepth                   ( m_toneMapCodedDataBitDepth );
  m_cTEncTop.setTMISEITargetBitDepth                      ( m_toneMapTargetBitDepth );
  m_cTEncTop.setTMISEIModelID                             ( m_toneMapModelId );
  m_cTEncTop.setTMISEIMinValue                            ( m_toneMapMinValue );
  m_cTEncTop.setTMISEIMaxValue                            ( m_toneMapMaxValue );
  m_cTEncTop.setTMISEISigmoidMidpoint                     ( m_sigmoidMidpoint );
  m_cTEncTop.setTMISEISigmoidWidth                        ( m_sigmoidWidth );
  m_cTEncTop.setTMISEIStartOfCodedInterva                 ( m_startOfCodedInterval );
  m_cTEncTop.setTMISEINumPivots                           ( m_numPivots );
  m_cTEncTop.setTMISEICodedPivotValue                     ( m_codedPivotValue );
  m_cTEncTop.setTMISEITargetPivotValue                    ( m_targetPivotValue );
  m_cTEncTop.setTMISEICameraIsoSpeedIdc                   ( m_cameraIsoSpeedIdc );
  m_cTEncTop.setTMISEICameraIsoSpeedValue                 ( m_cameraIsoSpeedValue );
  m_cTEncTop.setTMISEIExposureCompensationValueSignFlag   ( m_exposureCompensationValueSignFlag );
  m_cTEncTop.setTMISEIExposureCompensationValueNumerator  ( m_exposureCompensationValueNumerator );
  m_cTEncTop.setTMISEIExposureCompensationValueDenomIdc   ( m_exposureCompensationValueDenomIdc );
  m_cTEncTop.setTMISEIRefScreenLuminanceWhite             ( m_refScreenLuminanceWhite );
  m_cTEncTop.setTMISEIExtendedRangeWhiteLevel             ( m_extendedRangeWhiteLevel );
  m_cTEncTop.setTMISEINominalBlackLevelLumaCodeValue      ( m_nominalBlackLevelLumaCodeValue );
  m_cTEncTop.setTMISEINominalWhiteLevelLumaCodeValue      ( m_nominalWhiteLevelLumaCodeValue );
  m_cTEncTop.setTMISEIExtendedWhiteLevelLumaCodeValue     ( m_extendedWhiteLevelLumaCodeValue );
  m_cTEncTop.setFramePackingArrangementSEIEnabled( m_framePackingSEIEnabled );
  m_cTEncTop.setFramePackingArrangementSEIType( m_framePackingSEIType );
  m_cTEncTop.setFramePackingArrangementSEIId( m_framePackingSEIId );
  m_cTEncTop.setFramePackingArrangementSEIQuincunx( m_framePackingSEIQuincunx );
  m_cTEncTop.setFramePackingArrangementSEIInterpretation( m_framePackingSEIInterpretation );
  m_cTEncTop.setDisplayOrientationSEIAngle( m_displayOrientationSEIAngle );
  m_cTEncTop.setTemporalLevel0IndexSEIEnabled( m_temporalLevel0IndexSEIEnabled );
  m_cTEncTop.setGradualDecodingRefreshInfoEnabled( m_gradualDecodingRefreshInfoEnabled );
  m_cTEncTop.setDecodingUnitInfoSEIEnabled( m_decodingUnitInfoSEIEnabled );
  m_cTEncTop.setSOPDescriptionSEIEnabled( m_SOPDescriptionSEIEnabled );
  m_cTEncTop.setScalableNestingSEIEnabled( m_scalableNestingSEIEnabled );
  m_cTEncTop.setUniformSpacingIdr          ( m_iUniformSpacingIdr );
  m_cTEncTop.setNumColumnsMinus1           ( m_iNumColumnsMinus1 );
  m_cTEncTop.setNumRowsMinus1              ( m_iNumRowsMinus1 );
  if(m_iUniformSpacingIdr==0)
  {
    m_cTEncTop.setColumnWidth              ( m_pColumnWidth );
    m_cTEncTop.setRowHeight                ( m_pRowHeight );
  }
  m_cTEncTop.xCheckGSParameters();
  Int uiTilesCount          = (m_iNumRowsMinus1+1) * (m_iNumColumnsMinus1+1);
  if(uiTilesCount == 1)
  {
    m_bLFCrossTileBoundaryFlag = true; 
  }
  m_cTEncTop.setLFCrossTileBoundaryFlag( m_bLFCrossTileBoundaryFlag );
  m_cTEncTop.setWaveFrontSynchro           ( m_iWaveFrontSynchro );
  m_cTEncTop.setWaveFrontSubstreams        ( m_iWaveFrontSubstreams );
  m_cTEncTop.setTMVPModeId ( m_TMVPModeId );
  m_cTEncTop.setUseScalingListId           ( m_useScalingListId  );
  m_cTEncTop.setScalingListFile            ( m_scalingListFile   );
  m_cTEncTop.setSignHideFlag(m_signHideFlag);
#if RATE_CONTROL_LAMBDA_DOMAIN
  m_cTEncTop.setUseRateCtrl         ( m_RCEnableRateControl );
  m_cTEncTop.setTargetBitrate       ( m_RCTargetBitrate );
  m_cTEncTop.setKeepHierBit         ( m_RCKeepHierarchicalBit );
  m_cTEncTop.setLCULevelRC          ( m_RCLCULevelRC );
  m_cTEncTop.setUseLCUSeparateModel ( m_RCUseLCUSeparateModel );
  m_cTEncTop.setInitialQP           ( m_RCInitialQP );
  m_cTEncTop.setForceIntraQP        ( m_RCForceIntraQP );
#else
  m_cTEncTop.setUseRateCtrl     ( m_enableRateCtrl);
  m_cTEncTop.setTargetBitrate   ( m_targetBitrate);
  m_cTEncTop.setNumLCUInUnit    ( m_numLCUInUnit);
#endif
  m_cTEncTop.setTransquantBypassEnableFlag(m_TransquantBypassEnableFlag);
  m_cTEncTop.setCUTransquantBypassFlagValue(m_CUTransquantBypassFlagValue);
  m_cTEncTop.setUseRecalculateQPAccordingToLambda( m_recalculateQPAccordingToLambda );
  m_cTEncTop.setUseStrongIntraSmoothing( m_useStrongIntraSmoothing );
  m_cTEncTop.setActiveParameterSetsSEIEnabled ( m_activeParameterSetsSEIEnabled ); 
  m_cTEncTop.setVuiParametersPresentFlag( m_vuiParametersPresentFlag );
  m_cTEncTop.setAspectRatioIdc( m_aspectRatioIdc );
  m_cTEncTop.setSarWidth( m_sarWidth );
  m_cTEncTop.setSarHeight( m_sarHeight );
  m_cTEncTop.setOverscanInfoPresentFlag( m_overscanInfoPresentFlag );
  m_cTEncTop.setOverscanAppropriateFlag( m_overscanAppropriateFlag );
  m_cTEncTop.setVideoSignalTypePresentFlag( m_videoSignalTypePresentFlag );
  m_cTEncTop.setVideoFormat( m_videoFormat );
  m_cTEncTop.setVideoFullRangeFlag( m_videoFullRangeFlag );
  m_cTEncTop.setColourDescriptionPresentFlag( m_colourDescriptionPresentFlag );
  m_cTEncTop.setColourPrimaries( m_colourPrimaries );
  m_cTEncTop.setTransferCharacteristics( m_transferCharacteristics );
  m_cTEncTop.setMatrixCoefficients( m_matrixCoefficients );
  m_cTEncTop.setChromaLocInfoPresentFlag( m_chromaLocInfoPresentFlag );
  m_cTEncTop.setChromaSampleLocTypeTopField( m_chromaSampleLocTypeTopField );
  m_cTEncTop.setChromaSampleLocTypeBottomField( m_chromaSampleLocTypeBottomField );
  m_cTEncTop.setNeutralChromaIndicationFlag( m_neutralChromaIndicationFlag );
  m_cTEncTop.setDefaultDisplayWindow( m_defDispWinLeftOffset, m_defDispWinRightOffset, m_defDispWinTopOffset, m_defDispWinBottomOffset );
  m_cTEncTop.setFrameFieldInfoPresentFlag( m_frameFieldInfoPresentFlag );
  m_cTEncTop.setPocProportionalToTimingFlag( m_pocProportionalToTimingFlag );
  m_cTEncTop.setNumTicksPocDiffOneMinus1   ( m_numTicksPocDiffOneMinus1    );
  m_cTEncTop.setBitstreamRestrictionFlag( m_bitstreamRestrictionFlag );
  m_cTEncTop.setTilesFixedStructureFlag( m_tilesFixedStructureFlag );
  m_cTEncTop.setMotionVectorsOverPicBoundariesFlag( m_motionVectorsOverPicBoundariesFlag );
  m_cTEncTop.setMinSpatialSegmentationIdc( m_minSpatialSegmentationIdc );
  m_cTEncTop.setMaxBytesPerPicDenom( m_maxBytesPerPicDenom );
  m_cTEncTop.setMaxBitsPerMinCuDenom( m_maxBitsPerMinCuDenom );
  m_cTEncTop.setLog2MaxMvLengthHorizontal( m_log2MaxMvLengthHorizontal );
  m_cTEncTop.setLog2MaxMvLengthVertical( m_log2MaxMvLengthVertical );
}

Void TAppEncTop::xCreateLib()
{
  // Video I/O
  m_cTVideoIOYuvInputFile.open( m_pchInputFile,     false, m_inputBitDepthY, m_inputBitDepthC, m_internalBitDepthY, m_internalBitDepthC );  // read  mode
  m_cTVideoIOYuvInputFile.skipFrames(m_FrameSkip, m_iSourceWidth - m_aiPad[0], m_iSourceHeight - m_aiPad[1]);

  if (m_pchReconFile)
    m_cTVideoIOYuvReconFile.open(m_pchReconFile, true, m_outputBitDepthY, m_outputBitDepthC, m_internalBitDepthY, m_internalBitDepthC);  // write mode
  
  // Neo Decoder
  m_cTEncTop.create();
}

Void TAppEncTop::xDestroyLib()
{
  // Video I/O
  m_cTVideoIOYuvInputFile.close();
  m_cTVideoIOYuvReconFile.close();
  
  // Neo Decoder
  m_cTEncTop.destroy();
}

Void TAppEncTop::xInitLib(Bool isFieldCoding)
{
  m_cTEncTop.init(isFieldCoding);
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 - create internal class
 - initialize internal variable
 - until the end of input YUV file, call encoding function in TEncTop class
 - delete allocated buffers
 - destroy internal class
 .
 */
Void TAppEncTop::encode()
{
  fstream bitstreamFile(m_pchBitstreamFile, fstream::binary | fstream::out);
  if (!bitstreamFile)
  {
    fprintf(stderr, "\nfailed to open bitstream file `%s' for writing\n", m_pchBitstreamFile);
    exit(EXIT_FAILURE);
  }

  TComPicYuv*       pcPicYuvOrg = new TComPicYuv;
  TComPicYuv*       pcPicYuvRec = NULL;
  
  // initialize internal class & member variables
  xInitLibCfg();
  xCreateLib();
  xInitLib(m_isField);
  
  // main encoder loop
  Int   iNumEncoded = 0;
  Bool  bEos = false;
  
  list<AccessUnit> outputAccessUnits; ///< list of access units to write out.  is populated by the encoding process

  // allocate original YUV buffer
  if( m_isField )
  {
    pcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeightOrg, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
  }
  else
  {
    pcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeight, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
  }

	count_64x64_MSM = count_64x64_MERGE = count_64x64_2Nx2N_MERGE = count_64x64_SKIP = count_64x64_2Nx2N_SKIP = count_64x64_2Nx2N_nonMSM = count_64x64_2Nx2N = count_64x64_2NxN = count_64x64_Nx2N = count_64x64_NxN = count_64x64_2NxnU = count_64x64_2NxnD = count_64x64_nLx2N = count_64x64_nRx2N = 0;
	count_32x32_MSM = count_32x32_MERGE = count_32x32_2Nx2N_MERGE = count_32x32_SKIP = count_32x32_2Nx2N_SKIP = count_32x32_2Nx2N_nonMSM = count_32x32_2Nx2N = count_32x32_2NxN = count_32x32_Nx2N = count_32x32_NxN = count_32x32_2NxnU = count_32x32_2NxnD = count_32x32_nLx2N = count_32x32_nRx2N = 0;
	count_16x16_MSM = count_16x16_MERGE = count_16x16_2Nx2N_MERGE = count_16x16_SKIP = count_16x16_2Nx2N_SKIP = count_16x16_2Nx2N_nonMSM = count_16x16_2Nx2N = count_16x16_2NxN = count_16x16_Nx2N = count_16x16_NxN = count_16x16_2NxnU = count_16x16_2NxnD = count_16x16_nLx2N = count_16x16_nRx2N = 0;
	count_8x8_MSM = count_8x8_MERGE = count_8x8_2Nx2N_MERGE = count_8x8_SKIP = count_8x8_2Nx2N_SKIP = count_8x8_2Nx2N_nonMSM = count_8x8_2Nx2N = count_8x8_2NxN = count_8x8_Nx2N = count_8x8_NxN = count_8x8_2NxnU = count_8x8_2NxnD = count_8x8_nLx2N = count_8x8_nRx2N = 0;

	// write header in the variance and splitting stats file
       
	CU64x64data.open(filename_64x64, ios::out | ios::app);
	if(CU64x64data.is_open()) {
            CU64x64data << "@relation "<<relation<<'\n'<<'\n'<<"@attribute RDcost_MSM numeric" << '\n' << "@attribute RDcost_2Nx2N numeric" << '\n' << "@attribute RDcost_2NxN numeric"  << '\n' <<  "@attribute RDcost_Nx2N numeric"  << '\n' <<  "@attribute part {0,1,2,3,4,5,6,7}"  << '\n' <<  "@attribute MergeFlag {0,1}"  << '\n' <<  "@attribute SkipMergeFlag {0,1}"  << '\n' <<  "@attribute med_med numeric"  << '\n' <<  "@attribute neighDepth_diff numeric"  << '\n' << "@attribute SplitQuadtree {0,1}"<<'\n'<<'\n'<<"@data"<<endl;
            CU64x64data.close();
	}
	CU32x32data.open(filename_32x32, ios::out | ios::app);
	if(CU32x32data.is_open()) {
            CU32x32data << "@relation "<<relation<< '\n'<<'\n'<<"@attribute RDcost_MSM numeric" << '\n' << "@attribute RDcost_2Nx2N numeric" << '\n' << "@attribute RDcost_2NxN numeric"  << '\n' <<  "@attribute RDcost_Nx2N numeric"  << '\n' <<  "@attribute part {0,1,2,3,4,5,6,7}"  << '\n' <<  "@attribute MergeFlag {0,1}"  << '\n' <<  "@attribute SkipMergeFlag {0,1}"  << '\n' <<  "@attribute med_med numeric"  << '\n' <<  "@attribute neighDepth_diff numeric"  << '\n' << "@attribute SplitQuadtree {0,1}"<<'\n'<<'\n'<<"@data"<<endl;
            CU32x32data.close();
	}
	CU16x16data.open(filename_16x16, ios::out | ios::app);
	if(CU16x16data.is_open()) {
            CU16x16data << "@relation "<<relation<<'\n'<<'\n'<< "@attribute RDcost_MSM numeric" << '\n' << "@attribute RDcost_2Nx2N numeric" << '\n' << "@attribute RDcost_2NxN numeric"  << '\n' <<  "@attribute RDcost_Nx2N numeric"  << '\n' <<  "@attribute part {0,1,2,3,4,5,6,7}"  << '\n' <<  "@attribute MergeFlag {0,1}"  << '\n' <<  "@attribute SkipMergeFlag {0,1}"  << '\n' <<  "@attribute med_med numeric"  << '\n' <<  "@attribute neighDepth_diff numeric"  << '\n' << "@attribute SplitQuadtree {0,1}"<<'\n'<<'\n'<<"@data"<<endl;
            CU16x16data.close();
	}
	CU8x8data.open(filename_8x8, ios::out | ios::app);
	if(CU8x8data.is_open()) {
            CU8x8data << "@relation "<<relation<< '\n'<<'\n'<<"@attribute RDcost_MSM numeric" << '\n' << "@attribute RDcost_2Nx2N numeric" << '\n' << "@attribute RDcost_2NxN numeric"  << '\n' <<  "@attribute RDcost_Nx2N numeric"  << '\n' <<  "@attribute part {0,1,2,3,4,5,6,7}"  << '\n' <<  "@attribute MergeFlag {0,1}"  << '\n' <<  "@attribute SkipMergeFlag {0,1}"  << '\n' <<  "@attribute med_med numeric"  << '\n' <<  "@attribute neighDepth_diff numeric"  << '\n' << "@attribute SplitQuadtree {0,1}"<<'\n'<<'\n'<<"@data"<<endl;
            CU8x8data.close();
	}
	//gcorrea: 29/08/2013 END

  
  while ( !bEos )
  {
    // get buffers
    xGetBuffer(pcPicYuvRec);

    // read input YUV file
    m_cTVideoIOYuvInputFile.read( pcPicYuvOrg, m_aiPad );

    // increase number of received frames
    m_iFrameRcvd++;
    
    bEos = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded) );
    Bool flush = 0;
    // if end of file (which is only detected on a read failure) flush the encoder of any queued pictures
    if (m_cTVideoIOYuvInputFile.isEof())
    {
      flush = true;
      bEos = true;
      m_iFrameRcvd--;
      m_cTEncTop.setFramesToBeEncoded(m_iFrameRcvd);
    }

    // call encoding function for one frame
    if ( m_isField )
    {
      m_cTEncTop.encode( bEos, flush ? 0 : pcPicYuvOrg, m_cListPicYuvRec, outputAccessUnits, iNumEncoded, m_isTopFieldFirst);
    }
    else
    {
      m_cTEncTop.encode( bEos, flush ? 0 : pcPicYuvOrg, m_cListPicYuvRec, outputAccessUnits, iNumEncoded );
    }
    
    // write bistream to file if necessary
    if ( iNumEncoded > 0 )
    {
      xWriteOutput(bitstreamFile, iNumEncoded, outputAccessUnits);
      outputAccessUnits.clear();
    }
  }

  m_cTEncTop.printSummary(m_isField);

  // delete original YUV buffer
  pcPicYuvOrg->destroy();
  delete pcPicYuvOrg;
  pcPicYuvOrg = NULL;
  
  // delete used buffers in encoder class
  m_cTEncTop.deletePicBuffer();
  
  // delete buffers & classes
  xDeleteBuffer();
  xDestroyLib();
  
  printRateSummary();

  return;
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 - application has picture buffer list with size of GOP
 - picture buffer list acts as ring buffer
 - end of the list has the latest picture
 .
 */
Void TAppEncTop::xGetBuffer( TComPicYuv*& rpcPicYuvRec)
{
  assert( m_iGOPSize > 0 );
  
  // org. buffer
  if ( m_cListPicYuvRec.size() == (UInt)m_iGOPSize )
  {
    rpcPicYuvRec = m_cListPicYuvRec.popFront();

  }
  else
  {
    rpcPicYuvRec = new TComPicYuv;
    
    rpcPicYuvRec->create( m_iSourceWidth, m_iSourceHeight, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );

  }
  m_cListPicYuvRec.pushBack( rpcPicYuvRec );
}

Void TAppEncTop::xDeleteBuffer( )
{
  TComList<TComPicYuv*>::iterator iterPicYuvRec  = m_cListPicYuvRec.begin();
  
  Int iSize = Int( m_cListPicYuvRec.size() );
  
  for ( Int i = 0; i < iSize; i++ )
  {
    TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
    pcPicYuvRec->destroy();
    delete pcPicYuvRec; pcPicYuvRec = NULL;
  }
  
}

/** \param iNumEncoded  number of encoded frames
 */
Void TAppEncTop::xWriteOutput(std::ostream& bitstreamFile, Int iNumEncoded, const std::list<AccessUnit>& accessUnits)
{
  if (m_isField)
  {
    //Reinterlace fields
    Int i;
    TComList<TComPicYuv*>::iterator iterPicYuvRec = m_cListPicYuvRec.end();
    list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();
    
    for ( i = 0; i < iNumEncoded; i++ )
    {
      --iterPicYuvRec;
    }
    
    for ( i = 0; i < iNumEncoded/2; i++ )
    {
      TComPicYuv*  pcPicYuvRecTop  = *(iterPicYuvRec++);
      TComPicYuv*  pcPicYuvRecBottom  = *(iterPicYuvRec++);
      
      if (m_pchReconFile)
      {
        m_cTVideoIOYuvReconFile.write( pcPicYuvRecTop, pcPicYuvRecBottom, m_confLeft, m_confRight, m_confTop, m_confBottom, m_isTopFieldFirst );
      }
      
      const AccessUnit& auTop = *(iterBitstream++);
      const vector<UInt>& statsTop = writeAnnexB(bitstreamFile, auTop);
      rateStatsAccum(auTop, statsTop);
      
      const AccessUnit& auBottom = *(iterBitstream++);
      const vector<UInt>& statsBottom = writeAnnexB(bitstreamFile, auBottom);
      rateStatsAccum(auBottom, statsBottom);
    }
  }
  else
  {
    Int i;
    TComList<TComPicYuv*>::iterator iterPicYuvRec = m_cListPicYuvRec.end();
    list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();
    
    for ( i = 0; i < iNumEncoded; i++ )
    {
      --iterPicYuvRec;
    }
    
    for ( i = 0; i < iNumEncoded; i++ )
    {
      TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
      if (m_pchReconFile)
      {
        m_cTVideoIOYuvReconFile.write( pcPicYuvRec, m_confLeft, m_confRight, m_confTop, m_confBottom );
      }
      
      const AccessUnit& au = *(iterBitstream++);
      const vector<UInt>& stats = writeAnnexB(bitstreamFile, au);
      rateStatsAccum(au, stats);
    }
  }
}

/**
 *
 */
void TAppEncTop::rateStatsAccum(const AccessUnit& au, const std::vector<UInt>& annexBsizes)
{
  AccessUnit::const_iterator it_au = au.begin();
  vector<UInt>::const_iterator it_stats = annexBsizes.begin();

  for (; it_au != au.end(); it_au++, it_stats++)
  {
    switch ((*it_au)->m_nalUnitType)
    {
    case NAL_UNIT_CODED_SLICE_TRAIL_R:
    case NAL_UNIT_CODED_SLICE_TRAIL_N:
    case NAL_UNIT_CODED_SLICE_TLA_R:
    case NAL_UNIT_CODED_SLICE_TSA_N:
    case NAL_UNIT_CODED_SLICE_STSA_R:
    case NAL_UNIT_CODED_SLICE_STSA_N:
    case NAL_UNIT_CODED_SLICE_BLA_W_LP:
    case NAL_UNIT_CODED_SLICE_BLA_W_RADL:
    case NAL_UNIT_CODED_SLICE_BLA_N_LP:
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_RADL_N:
    case NAL_UNIT_CODED_SLICE_RADL_R:
    case NAL_UNIT_CODED_SLICE_RASL_N:
    case NAL_UNIT_CODED_SLICE_RASL_R:
    case NAL_UNIT_VPS:
    case NAL_UNIT_SPS:
    case NAL_UNIT_PPS:
      m_essentialBytes += *it_stats;
      break;
    default:
      break;
    }

    m_totalBytes += *it_stats;
  }
}

void TAppEncTop::printRateSummary()
{
  Double time = (Double) m_iFrameRcvd / m_iFrameRate;
  printf("Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time);
#if VERBOSE_RATE
  printf("Bytes for SPS/PPS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes, 0.008 * m_essentialBytes / time);
#endif
}

//! \}

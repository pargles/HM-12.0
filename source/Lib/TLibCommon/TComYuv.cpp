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

/** \file     TComYuv.cpp
    \brief    general YUV buffer class
    \todo     this should be merged with TComPicYuv
*/
#include <stdlib.h>
#include <memory.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "CommonDef.h"
#include "TComYuv.h"
#include "TComInterpolationFilter.h"

using namespace std;

//gcorrea: 17/10/2013
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
extern double res_TotalGradVer_CB, res_TotalGradHor_CB, res_TotalGrad_CB, res_RHV_TotalGrad_CB;
extern double res_RHV_sumCoef;
extern double res_RHV_sumVar3x3, res_RVH_sumVar3x3, res_sumVar3x3_HP2, res_sumVar3x3_HP1, res_sumVar3x3_VP2, res_sumVar3x3_VP1;
extern double ResHorAccGrad, ResVerAccGrad;


//gcorrea: 17/10/2013 END

//! \ingroup TLibCommon
//! \{

TComYuv::TComYuv()
{
  m_apiBufY = NULL;
  m_apiBufU = NULL;
  m_apiBufV = NULL;
}

TComYuv::~TComYuv()
{
}

Void TComYuv::create( UInt iWidth, UInt iHeight )
{
  // memory allocation
  m_apiBufY  = (Pel*)xMalloc( Pel, iWidth*iHeight    );
  m_apiBufU  = (Pel*)xMalloc( Pel, iWidth*iHeight >> 2 );
  m_apiBufV  = (Pel*)xMalloc( Pel, iWidth*iHeight >> 2 );
  
  // set width and height
  m_iWidth   = iWidth;
  m_iHeight  = iHeight;
  m_iCWidth  = iWidth  >> 1;
  m_iCHeight = iHeight >> 1;
}

Void TComYuv::destroy()
{
  // memory free
  xFree( m_apiBufY ); m_apiBufY = NULL;
  xFree( m_apiBufU ); m_apiBufU = NULL;
  xFree( m_apiBufV ); m_apiBufV = NULL;
}

Void TComYuv::clear()
{
  ::memset( m_apiBufY, 0, ( m_iWidth  * m_iHeight  )*sizeof(Pel) );
  ::memset( m_apiBufU, 0, ( m_iCWidth * m_iCHeight )*sizeof(Pel) );
  ::memset( m_apiBufV, 0, ( m_iCWidth * m_iCHeight )*sizeof(Pel) );
}

Void TComYuv::copyToPicYuv   ( TComPicYuv* pcPicYuvDst, UInt iCuAddr, UInt uiAbsZorderIdx, UInt uiPartDepth, UInt uiPartIdx )
{
  copyToPicLuma  ( pcPicYuvDst, iCuAddr, uiAbsZorderIdx, uiPartDepth, uiPartIdx );
  copyToPicChroma( pcPicYuvDst, iCuAddr, uiAbsZorderIdx, uiPartDepth, uiPartIdx );
}

Void TComYuv::copyToPicLuma  ( TComPicYuv* pcPicYuvDst, UInt iCuAddr, UInt uiAbsZorderIdx, UInt uiPartDepth, UInt uiPartIdx )
{
  Int  y, iWidth, iHeight;
  iWidth  = m_iWidth >>uiPartDepth;
  iHeight = m_iHeight>>uiPartDepth;
  
  Pel* pSrc     = getLumaAddr(uiPartIdx, iWidth);
  Pel* pDst     = pcPicYuvDst->getLumaAddr ( iCuAddr, uiAbsZorderIdx );
  
  UInt  iSrcStride  = getStride();
  UInt  iDstStride  = pcPicYuvDst->getStride();
  
  for ( y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}

Void TComYuv::copyToPicChroma( TComPicYuv* pcPicYuvDst, UInt iCuAddr, UInt uiAbsZorderIdx, UInt uiPartDepth, UInt uiPartIdx )
{
  Int  y, iWidth, iHeight;
  iWidth  = m_iCWidth >>uiPartDepth;
  iHeight = m_iCHeight>>uiPartDepth;
  
  Pel* pSrcU      = getCbAddr(uiPartIdx, iWidth);
  Pel* pSrcV      = getCrAddr(uiPartIdx, iWidth);
  Pel* pDstU      = pcPicYuvDst->getCbAddr( iCuAddr, uiAbsZorderIdx );
  Pel* pDstV      = pcPicYuvDst->getCrAddr( iCuAddr, uiAbsZorderIdx );
  
  UInt  iSrcStride = getCStride();
  UInt  iDstStride = pcPicYuvDst->getCStride();
  for ( y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, sizeof(Pel)*(iWidth) );
    ::memcpy( pDstV, pSrcV, sizeof(Pel)*(iWidth) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
}

Void TComYuv::copyFromPicYuv   ( TComPicYuv* pcPicYuvSrc, UInt iCuAddr, UInt uiAbsZorderIdx )
{
  copyFromPicLuma  ( pcPicYuvSrc, iCuAddr, uiAbsZorderIdx );
  copyFromPicChroma( pcPicYuvSrc, iCuAddr, uiAbsZorderIdx );
}

Void TComYuv::copyFromPicLuma  ( TComPicYuv* pcPicYuvSrc, UInt iCuAddr, UInt uiAbsZorderIdx )
{
  Int  y;
  
  Pel* pDst     = m_apiBufY;
  Pel* pSrc     = pcPicYuvSrc->getLumaAddr ( iCuAddr, uiAbsZorderIdx );
  
  UInt  iDstStride  = getStride();
  UInt  iSrcStride  = pcPicYuvSrc->getStride();
  for ( y = m_iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*m_iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}

Void TComYuv::copyFromPicChroma( TComPicYuv* pcPicYuvSrc, UInt iCuAddr, UInt uiAbsZorderIdx )
{
  Int  y;
  
  Pel* pDstU      = m_apiBufU;
  Pel* pDstV      = m_apiBufV;
  Pel* pSrcU      = pcPicYuvSrc->getCbAddr( iCuAddr, uiAbsZorderIdx );
  Pel* pSrcV      = pcPicYuvSrc->getCrAddr( iCuAddr, uiAbsZorderIdx );
  
  UInt  iDstStride = getCStride();
  UInt  iSrcStride = pcPicYuvSrc->getCStride();
  for ( y = m_iCHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, sizeof(Pel)*(m_iCWidth) );
    ::memcpy( pDstV, pSrcV, sizeof(Pel)*(m_iCWidth) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
}

Void TComYuv::copyToPartYuv( TComYuv* pcYuvDst, UInt uiDstPartIdx )
{
  copyToPartLuma  ( pcYuvDst, uiDstPartIdx );
  copyToPartChroma( pcYuvDst, uiDstPartIdx );
}

Void TComYuv::copyToPartLuma( TComYuv* pcYuvDst, UInt uiDstPartIdx )
{
  Int  y;
  
  Pel* pSrc     = m_apiBufY;
  Pel* pDst     = pcYuvDst->getLumaAddr( uiDstPartIdx );
  
  UInt  iSrcStride  = getStride();
  UInt  iDstStride  = pcYuvDst->getStride();
  for ( y = m_iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*m_iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}

Void TComYuv::copyToPartChroma( TComYuv* pcYuvDst, UInt uiDstPartIdx )
{
  Int  y;
  
  Pel* pSrcU      = m_apiBufU;
  Pel* pSrcV      = m_apiBufV;
  Pel* pDstU      = pcYuvDst->getCbAddr( uiDstPartIdx );
  Pel* pDstV      = pcYuvDst->getCrAddr( uiDstPartIdx );
  
  UInt  iSrcStride = getCStride();
  UInt  iDstStride = pcYuvDst->getCStride();
  for ( y = m_iCHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, sizeof(Pel)*(m_iCWidth) );
    ::memcpy( pDstV, pSrcV, sizeof(Pel)*(m_iCWidth) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
}

Void TComYuv::copyPartToYuv( TComYuv* pcYuvDst, UInt uiSrcPartIdx )
{
  copyPartToLuma  ( pcYuvDst, uiSrcPartIdx );
  copyPartToChroma( pcYuvDst, uiSrcPartIdx );
}

Void TComYuv::copyPartToLuma( TComYuv* pcYuvDst, UInt uiSrcPartIdx )
{
  Int  y;
  
  Pel* pSrc     = getLumaAddr(uiSrcPartIdx);
  Pel* pDst     = pcYuvDst->getLumaAddr( 0 );
  
  UInt  iSrcStride  = getStride();
  UInt  iDstStride  = pcYuvDst->getStride();
  
  UInt uiHeight = pcYuvDst->getHeight();
  UInt uiWidth = pcYuvDst->getWidth();
  
  for ( y = uiHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*uiWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}

Void TComYuv::copyPartToChroma( TComYuv* pcYuvDst, UInt uiSrcPartIdx )
{
  Int  y;
  
  Pel* pSrcU      = getCbAddr( uiSrcPartIdx );
  Pel* pSrcV      = getCrAddr( uiSrcPartIdx );
  Pel* pDstU      = pcYuvDst->getCbAddr( 0 );
  Pel* pDstV      = pcYuvDst->getCrAddr( 0 );
  
  UInt  iSrcStride = getCStride();
  UInt  iDstStride = pcYuvDst->getCStride();
  
  UInt uiCHeight = pcYuvDst->getCHeight();
  UInt uiCWidth = pcYuvDst->getCWidth();
  
  for ( y = uiCHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, sizeof(Pel)*(uiCWidth) );
    ::memcpy( pDstV, pSrcV, sizeof(Pel)*(uiCWidth) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
}

Void TComYuv::copyPartToPartYuv   ( TComYuv* pcYuvDst, UInt uiPartIdx, UInt iWidth, UInt iHeight )
{
  copyPartToPartLuma   (pcYuvDst, uiPartIdx, iWidth, iHeight );
  copyPartToPartChroma (pcYuvDst, uiPartIdx, iWidth>>1, iHeight>>1 );
}

Void TComYuv::copyPartToPartLuma  ( TComYuv* pcYuvDst, UInt uiPartIdx, UInt iWidth, UInt iHeight )
{
  Pel* pSrc =           getLumaAddr(uiPartIdx);
  Pel* pDst = pcYuvDst->getLumaAddr(uiPartIdx);
  if( pSrc == pDst )
  {
    //th not a good idea
    //th best would be to fix the caller 
    return ;
  }
  
  UInt  iSrcStride = getStride();
  UInt  iDstStride = pcYuvDst->getStride();
  for ( UInt y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, iWidth * sizeof(Pel) );
    pSrc += iSrcStride;
    pDst += iDstStride;
  }
}

Void TComYuv::copyPartToPartChroma( TComYuv* pcYuvDst, UInt uiPartIdx, UInt iWidth, UInt iHeight )
{
  Pel*  pSrcU =           getCbAddr(uiPartIdx);
  Pel*  pSrcV =           getCrAddr(uiPartIdx);
  Pel*  pDstU = pcYuvDst->getCbAddr(uiPartIdx);
  Pel*  pDstV = pcYuvDst->getCrAddr(uiPartIdx);
  
  if( pSrcU == pDstU && pSrcV == pDstV)
  {
    //th not a good idea
    //th best would be to fix the caller 
    return ;
  }
  
  UInt   iSrcStride = getCStride();
  UInt   iDstStride = pcYuvDst->getCStride();
  for ( UInt y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, iWidth * sizeof(Pel) );
    ::memcpy( pDstV, pSrcV, iWidth * sizeof(Pel) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
}

Void TComYuv::copyPartToPartChroma( TComYuv* pcYuvDst, UInt uiPartIdx, UInt iWidth, UInt iHeight, UInt chromaId)
{
  if(chromaId == 0)
  {
    Pel*  pSrcU =           getCbAddr(uiPartIdx);
    Pel*  pDstU = pcYuvDst->getCbAddr(uiPartIdx);
    if( pSrcU == pDstU)
    {
      return ;
    }
    UInt   iSrcStride = getCStride();
    UInt   iDstStride = pcYuvDst->getCStride();
    for ( UInt y = iHeight; y != 0; y-- )
    {
      ::memcpy( pDstU, pSrcU, iWidth * sizeof(Pel) );
      pSrcU += iSrcStride;
      pDstU += iDstStride;
    }
  }
  else if (chromaId == 1)
  {
    Pel*  pSrcV =           getCrAddr(uiPartIdx);
    Pel*  pDstV = pcYuvDst->getCrAddr(uiPartIdx);
    if( pSrcV == pDstV)
    {
      return;
    }
    UInt   iSrcStride = getCStride();
    UInt   iDstStride = pcYuvDst->getCStride();
    for ( UInt y = iHeight; y != 0; y-- )
    { 
      ::memcpy( pDstV, pSrcV, iWidth * sizeof(Pel) );
      pSrcV += iSrcStride;
      pDstV += iDstStride;
    }
  }
  else
  {
    Pel*  pSrcU =           getCbAddr(uiPartIdx);
    Pel*  pSrcV =           getCrAddr(uiPartIdx);
    Pel*  pDstU = pcYuvDst->getCbAddr(uiPartIdx);
    Pel*  pDstV = pcYuvDst->getCrAddr(uiPartIdx);
    
    if( pSrcU == pDstU && pSrcV == pDstV)
    {
      //th not a good idea
      //th best would be to fix the caller 
      return ;
    }
    UInt   iSrcStride = getCStride();
    UInt   iDstStride = pcYuvDst->getCStride();
    for ( UInt y = iHeight; y != 0; y-- )
    {
      ::memcpy( pDstU, pSrcU, iWidth * sizeof(Pel) );
      ::memcpy( pDstV, pSrcV, iWidth * sizeof(Pel) );
      pSrcU += iSrcStride;
      pSrcV += iSrcStride;
      pDstU += iDstStride;
      pDstV += iDstStride;
    }
  }
}

Void TComYuv::addClip( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  addClipLuma   ( pcYuvSrc0, pcYuvSrc1, uiTrUnitIdx, uiPartSize     );
  addClipChroma ( pcYuvSrc0, pcYuvSrc1, uiTrUnitIdx, uiPartSize>>1  );
}

Void TComYuv::addClipLuma( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  Int x, y;
  
  Pel* pSrc0 = pcYuvSrc0->getLumaAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrc1 = pcYuvSrc1->getLumaAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
  
  UInt iSrc0Stride = pcYuvSrc0->getStride();
  UInt iSrc1Stride = pcYuvSrc1->getStride();
  UInt iDstStride  = getStride();
  for ( y = uiPartSize-1; y >= 0; y-- )
  {
    for ( x = uiPartSize-1; x >= 0; x-- )
    {
      pDst[x] = ClipY( pSrc0[x] + pSrc1[x] );
    }
    pSrc0 += iSrc0Stride;
    pSrc1 += iSrc1Stride;
    pDst  += iDstStride;
  }
}

Void TComYuv::addClipChroma( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  Int x, y;
  
  Pel* pSrcU0 = pcYuvSrc0->getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcU1 = pcYuvSrc1->getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcV0 = pcYuvSrc0->getCrAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcV1 = pcYuvSrc1->getCrAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDstU = getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDstV = getCrAddr( uiTrUnitIdx, uiPartSize );
  
  UInt  iSrc0Stride = pcYuvSrc0->getCStride();
  UInt  iSrc1Stride = pcYuvSrc1->getCStride();
  UInt  iDstStride  = getCStride();
  for ( y = uiPartSize-1; y >= 0; y-- )
  {
    for ( x = uiPartSize-1; x >= 0; x-- )
    {
      pDstU[x] = ClipC( pSrcU0[x] + pSrcU1[x] );
      pDstV[x] = ClipC( pSrcV0[x] + pSrcV1[x] );
    }
    
    pSrcU0 += iSrc0Stride;
    pSrcU1 += iSrc1Stride;
    pSrcV0 += iSrc0Stride;
    pSrcV1 += iSrc1Stride;
    pDstU  += iDstStride;
    pDstV  += iDstStride;
  }
}

Void TComYuv::subtract( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  subtractLuma  ( pcYuvSrc0, pcYuvSrc1,  uiTrUnitIdx, uiPartSize    );
  subtractChroma( pcYuvSrc0, pcYuvSrc1,  uiTrUnitIdx, uiPartSize>>1 );
}

Void TComYuv::subtractLuma( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  Int x, y;
  
  Pel* pSrc0 = pcYuvSrc0->getLumaAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrc1 = pcYuvSrc1->getLumaAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
  
  Int  iSrc0Stride = pcYuvSrc0->getStride();
  Int  iSrc1Stride = pcYuvSrc1->getStride();
  Int  iDstStride  = getStride();

  
  if(!saveResData2Nx2N) {	//gcorrea: 17/10/2013
	  for ( y = uiPartSize-1; y >= 0; y-- )
	  {
		for ( x = uiPartSize-1; x >= 0; x-- )
		{
		  pDst[x] = pSrc0[x] - pSrc1[x];
		}
		pSrc0 += iSrc0Stride;
		pSrc1 += iSrc1Stride;
		pDst  += iDstStride;
	  }
  }
  //gcorrea: 21/10/2013
  else {

	  // computes the average of the residue
	  sumRes2Nx2N = 0;
	  for ( y = uiPartSize-1; y >= 0; y-- )
	  {
		for ( x = uiPartSize-1; x >= 0; x-- )
		{
		  pDst[x] = pSrc0[x] - pSrc1[x];
		  sumRes2Nx2N += fabs(pDst[x]);
		}
		pSrc0 += iSrc0Stride;
		pSrc1 += iSrc1Stride;
		pDst  += iDstStride;
	  }
	  medRes2Nx2N = sumRes2Nx2N/(uiPartSize*uiPartSize);

	  // computes the variance of the residue
	  varRes2Nx2N = 0;
	  sqdRes2Nx2N = 0;
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = uiPartSize-1; y >= 0; y-- )
	  {
		for ( x = uiPartSize-1; x >= 0; x-- )
		{
		  sqdRes2Nx2N += ((fabs(pDst[x]) - medRes2Nx2N) * (fabs(pDst[x]) - medRes2Nx2N));
		}
		pDst  += iDstStride;
	  }
	  varRes2Nx2N = sqdRes2Nx2N / (uiPartSize*uiPartSize);

	  
	  // copy residual block
	  Pel **saveResBlk = new Pel* [uiPartSize];
	  for(int i = 0; i < uiPartSize; i++)
		  saveResBlk[i] = new Pel[uiPartSize];

	  pDst = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( int i = uiPartSize-1; i >= 0; i-- )
	  {
		for ( int j = uiPartSize-1; j >= 0; j-- )
		{
			saveResBlk[i][j] = pDst[j];
		}
		pDst  += iDstStride;
	  }

	 //for (int i = 0; i < uiPartSize; i++)
	 //{
		//for (int j = 0; j < uiPartSize; j++)
		//{
		//	cout << saveResBlk[i][j] << '\t';
		//}
		//cout << endl;
	 //}
	  
	 // GRADIENT CALCULATION
	 res_HP2grad = res_HP1grad = res_VP2grad = res_VP1grad = 0;
	 res_RHV_grad = res_TotalGradVer_CB = res_TotalGradHor_CB = res_TotalGrad_CB = res_RHV_TotalGrad_CB = 0;

	 // GRAD VP1
	 ResHorGrad = ResVerGrad = 0;
	 for (int i = 0; i < uiPartSize; i++)
	 {
		for (int j = 0; j < uiPartSize/2; j++)
		{
			if(j==0)
				ResHorGrad += fabs(saveResBlk[i][j+1] - saveResBlk[i][j]);
			else if(j==uiPartSize-1)
				ResHorGrad += fabs(saveResBlk[i][j] - saveResBlk[i][j-1]);
			else
				ResHorGrad += fabs(saveResBlk[i][j+1] - saveResBlk[i][j-1]);

			if(i==0)
				ResVerGrad += fabs(saveResBlk[i+1][j] - saveResBlk[i][j]);
			else if(i==uiPartSize-1)
				ResVerGrad += fabs(saveResBlk[i][j] - saveResBlk[i-1][j]);
			else
				ResVerGrad += fabs(saveResBlk[i+1][j] - saveResBlk[i-1][j]);
		}
	 }
	 res_VP1grad = (ResHorGrad+ResVerGrad)/((uiPartSize*uiPartSize)/2);


	 // GRAD VP2
	 ResHorGrad = ResVerGrad = 0;
	 for (int i = 0; i < uiPartSize; i++)
	 {
		for (int j = uiPartSize/2; j < uiPartSize; j++)
		{
			if(j==0)
				ResHorGrad += fabs(saveResBlk[i][j+1] - saveResBlk[i][j]);
			else if(j==uiPartSize-1)
				ResHorGrad += fabs(saveResBlk[i][j] - saveResBlk[i][j-1]);
			else
				ResHorGrad += fabs(saveResBlk[i][j+1] - saveResBlk[i][j-1]);

			if(i==0)
				ResVerGrad += fabs(saveResBlk[i+1][j] - saveResBlk[i][j]);
			else if(i==uiPartSize-1)
				ResVerGrad += fabs(saveResBlk[i][j] - saveResBlk[i-1][j]);
			else
				ResVerGrad += fabs(saveResBlk[i+1][j] - saveResBlk[i-1][j]);
		}
	 }
	 res_VP2grad = (ResHorGrad+ResVerGrad)/((uiPartSize*uiPartSize)/2);

	 // GRAD HP1
	 ResHorGrad = ResVerGrad = 0;
	 for (int i = 0; i < uiPartSize/2; i++)
	 {
		for (int j = 0; j < uiPartSize; j++)
		{
			if(j==0)
				ResHorGrad += fabs(saveResBlk[i][j+1] - saveResBlk[i][j]);
			else if(j==uiPartSize-1)
				ResHorGrad += fabs(saveResBlk[i][j] - saveResBlk[i][j-1]);
			else
				ResHorGrad += fabs(saveResBlk[i][j+1] - saveResBlk[i][j-1]);

			if(i==0)
				ResVerGrad += fabs(saveResBlk[i+1][j] - saveResBlk[i][j]);
			else if(i==uiPartSize-1)
				ResVerGrad += fabs(saveResBlk[i][j] - saveResBlk[i-1][j]);
			else
				ResVerGrad += fabs(saveResBlk[i+1][j] - saveResBlk[i-1][j]);
		}
	 }
	 res_HP1grad = (ResHorGrad+ResVerGrad)/((uiPartSize*uiPartSize)/2);

	 // GRAD HP2
	 ResHorGrad = ResVerGrad = 0;
	 for (int i = uiPartSize/2; i < uiPartSize; i++)
	 {
		for (int j = 0; j < uiPartSize; j++)
		{
			if(j==0)
				ResHorGrad += fabs(saveResBlk[i][j+1] - saveResBlk[i][j]);
			else if(j==uiPartSize-1)
				ResHorGrad += fabs(saveResBlk[i][j] - saveResBlk[i][j-1]);
			else
				ResHorGrad += fabs(saveResBlk[i][j+1] - saveResBlk[i][j-1]);

			if(i==0)
				ResVerGrad += fabs(saveResBlk[i+1][j] - saveResBlk[i][j]);
			else if(i==uiPartSize-1)
				ResVerGrad += fabs(saveResBlk[i][j] - saveResBlk[i-1][j]);
			else
				ResVerGrad += fabs(saveResBlk[i+1][j] - saveResBlk[i-1][j]);
		}
	 }
	 res_HP2grad = (ResHorGrad+ResVerGrad)/((uiPartSize*uiPartSize)/2);

	// GRAD TOTAL
	 ResHorGrad = ResVerGrad = 0;
	 for (int i = 0; i < uiPartSize; i++)
	 {
		for (int j = 0; j < uiPartSize; j++)
		{
			if(j==0)
				ResHorGrad += fabs(saveResBlk[i][j+1] - saveResBlk[i][j]);
			else if(j==uiPartSize-1)
				ResHorGrad += fabs(saveResBlk[i][j] - saveResBlk[i][j-1]);
			else
				ResHorGrad += fabs(saveResBlk[i][j+1] - saveResBlk[i][j-1]);

			if(i==0)
				ResVerGrad += fabs(saveResBlk[i+1][j] - saveResBlk[i][j]);
			else if(i==uiPartSize-1)
				ResVerGrad += fabs(saveResBlk[i][j] - saveResBlk[i-1][j]);
			else
				ResVerGrad += fabs(saveResBlk[i+1][j] - saveResBlk[i-1][j]);
		}
	 }
	res_TotalGradVer_CB = ResVerGrad/(uiPartSize*uiPartSize);
	res_TotalGradHor_CB = ResHorGrad/(uiPartSize*uiPartSize);

	res_TotalGrad_CB = res_TotalGradVer_CB+res_TotalGradHor_CB;



	// ACC GRAD TOTAL
	 ResHorAccGrad = ResVerAccGrad = 0;
	 int m = uiPartSize/2;
	 int n = uiPartSize/2-1;
	 if(uiPartSize == 8)
	 {
		 for (int i = 0; i < uiPartSize; i++)
		 {
			ResHorAccGrad += 4*fabs(saveResBlk[i][m] - saveResBlk[i][n]) + 3*fabs(saveResBlk[i][m+1] - saveResBlk[i][n-1]) + 2*fabs(saveResBlk[i][m+2] - saveResBlk[i][n-2]) + 1*fabs(saveResBlk[i][m+3] - saveResBlk[i][n-3]);
		 }
		 for (int i = 0; i < uiPartSize; i++)
		 {
			ResVerAccGrad += 4*fabs(saveResBlk[m][i] - saveResBlk[n][i]) + 3*fabs(saveResBlk[m+1][i] - saveResBlk[n-1][i]) + 2*fabs(saveResBlk[m+2][i] - saveResBlk[n-2][i]) + 1*fabs(saveResBlk[m+3][i] - saveResBlk[n-3][i]); 
		 }
	 }
	 else if(uiPartSize == 16)
	 {
		 for (int i = 0; i < uiPartSize; i++)
		 {
			ResHorAccGrad += 4*fabs(saveResBlk[i][m] - saveResBlk[i][n]) + 3*fabs(saveResBlk[i][m+2] - saveResBlk[i][n-2]) + 2*fabs(saveResBlk[i][m+4] - saveResBlk[i][n-4]) + 1*fabs(saveResBlk[i][m+6] - saveResBlk[i][n-6]);
		 }
		 for (int i = 0; i < uiPartSize; i++)
		 {
			ResVerAccGrad += 4*fabs(saveResBlk[m][i] - saveResBlk[n][i]) + 3*fabs(saveResBlk[m+2][i] - saveResBlk[n-2][i]) + 2*fabs(saveResBlk[m+4][i] - saveResBlk[n-4][i]) + 1*fabs(saveResBlk[m+6][i] - saveResBlk[n-6][i]);
		 }
	 }
	 else if(uiPartSize == 32)
	 {
		 for (int i = 0; i < uiPartSize; i++)
		 {
			ResHorAccGrad += 4*fabs(saveResBlk[i][m] - saveResBlk[i][n]) + 3*fabs(saveResBlk[i][m+4] - saveResBlk[i][n-4]) + 2*fabs(saveResBlk[i][m+8] - saveResBlk[i][n-8]) + 1*fabs(saveResBlk[i][m+12] - saveResBlk[i][n-12]);
		 }
		 for (int i = 0; i < uiPartSize; i++)
		 {
			ResVerAccGrad +=  4*fabs(saveResBlk[m][i] - saveResBlk[n][i]) + 3*fabs(saveResBlk[m+4][i] - saveResBlk[n-4][i]) + 2*fabs(saveResBlk[m+8][i] - saveResBlk[n-8][i]) + 1*fabs(saveResBlk[m+12][i] - saveResBlk[n-12][i]);
		 }
	 }
	 else if(uiPartSize == 64)
	 {
		 for (int i = 0; i < uiPartSize; i++)
		 {
			ResHorAccGrad += 4*fabs(saveResBlk[i][m] - saveResBlk[i][n]) + 3*fabs(saveResBlk[i][m+8] - saveResBlk[i][n-8]) + 2*fabs(saveResBlk[i][m+16] - saveResBlk[i][n-16]) + 1*fabs(saveResBlk[i][m+24] - saveResBlk[i][n-24]);
		 }
		 for (int i = 0; i < uiPartSize; i++)
		 {
			ResVerAccGrad += 4*fabs(saveResBlk[m][i] - saveResBlk[n][i]) + 3*fabs(saveResBlk[m+8][i] - saveResBlk[n-8][i]) + 2*fabs(saveResBlk[m+16][i] - saveResBlk[n-16][i]) + 1*fabs(saveResBlk[m+24][i] - saveResBlk[n-24][i]); 
		 }
	 }
	 //res_TotalGradVer_CB = ResVerGrad/(uiPartSize*uiPartSize);
	 //res_TotalGradHor_CB = ResHorGrad/(uiPartSize*uiPartSize);
	 
	 //res_TotalGrad_CB = res_TotalGradVer_CB+res_TotalGradHor_CB;


	 // SOBEL CALCULATION
	 double vertical_sobel, horizontal_sobel;
	 res_HP2Sobel = res_HP1Sobel = res_VP2Sobel = res_VP1Sobel = 0;
	 res_TotalSobelVer_CB = res_TotalSobelHor_CB = res_TotalSobel_CB = res_RHV_Sobel = res_RHV_TotalSobel = 0;

	 // SOBEL VP1
	 vertical_sobel = horizontal_sobel = 0;
	 for (int i = 1; i < uiPartSize-1; i++)
	 {
		for (int j = 1; j < uiPartSize/2-1; j++)
		{
			vertical_sobel += fabs(saveResBlk[i-1][j+1] + 2*saveResBlk[i][j+1] + saveResBlk[i+1][j+1] - saveResBlk[i-1][j-1] - 2*saveResBlk[i][j-1] - saveResBlk[i+1][j-1]);
			horizontal_sobel += fabs(saveResBlk[i+1][j-1] + 2*saveResBlk[i+1][j] + saveResBlk[i+1][j+1] - saveResBlk[i-1][j-1] - 2*saveResBlk[i-1][j] - saveResBlk[i-1][j+1]);
		}
	 }
	 vertical_sobel /= (uiPartSize*uiPartSize)/2;
	 horizontal_sobel /= (uiPartSize*uiPartSize)/2;
	 res_VP1Sobel = vertical_sobel+horizontal_sobel;

	 // SOBEL VP2
	 vertical_sobel = horizontal_sobel = 0;
	 for (int i = 1; i < uiPartSize-1; i++)
	 {
		for (int j = uiPartSize/2+1; j < uiPartSize-1; j++)
		{
			vertical_sobel += fabs(saveResBlk[i-1][j+1] + 2*saveResBlk[i][j+1] + saveResBlk[i+1][j+1] - saveResBlk[i-1][j-1] - 2*saveResBlk[i][j-1] - saveResBlk[i+1][j-1]);
			horizontal_sobel += fabs(saveResBlk[i+1][j-1] + 2*saveResBlk[i+1][j] + saveResBlk[i+1][j+1] - saveResBlk[i-1][j-1] - 2*saveResBlk[i-1][j] - saveResBlk[i-1][j+1]);
		}
	 }
	 vertical_sobel /= (uiPartSize*uiPartSize)/2;
	 horizontal_sobel /= (uiPartSize*uiPartSize)/2;
	 res_VP2Sobel = vertical_sobel+horizontal_sobel;

	 // SOBEL HP1
	 vertical_sobel = horizontal_sobel = 0;
	 for (int i = 1; i < uiPartSize/2-1; i++)
	 {
		for (int j = 1; j < uiPartSize-1; j++)
		{
			vertical_sobel += fabs(saveResBlk[i-1][j+1] + 2*saveResBlk[i][j+1] + saveResBlk[i+1][j+1] - saveResBlk[i-1][j-1] - 2*saveResBlk[i][j-1] - saveResBlk[i+1][j-1]);
			horizontal_sobel += fabs(saveResBlk[i+1][j-1] + 2*saveResBlk[i+1][j] + saveResBlk[i+1][j+1] - saveResBlk[i-1][j-1] - 2*saveResBlk[i-1][j] - saveResBlk[i-1][j+1]);
		}
	 }
	 vertical_sobel /= (uiPartSize*uiPartSize)/2;
	 horizontal_sobel /= (uiPartSize*uiPartSize)/2;
	 res_HP1Sobel = vertical_sobel+horizontal_sobel;

	 // SOBEL HP2
	 vertical_sobel = horizontal_sobel = 0;
	 for (int i = uiPartSize/2+1; i < uiPartSize-1; i++)
	 {
		for (int j = 1; j < uiPartSize-1; j++)
		{
			vertical_sobel += fabs(saveResBlk[i-1][j+1] + 2*saveResBlk[i][j+1] + saveResBlk[i+1][j+1] - saveResBlk[i-1][j-1] - 2*saveResBlk[i][j-1] - saveResBlk[i+1][j-1]);
			horizontal_sobel += fabs(saveResBlk[i+1][j-1] + 2*saveResBlk[i+1][j] + saveResBlk[i+1][j+1] - saveResBlk[i-1][j-1] - 2*saveResBlk[i-1][j] - saveResBlk[i-1][j+1]);
		}
	 }
	 vertical_sobel /= (uiPartSize*uiPartSize)/2;
	 horizontal_sobel /= (uiPartSize*uiPartSize)/2;
	 res_HP2Sobel = vertical_sobel+horizontal_sobel;
	 
	 // SOBEL TOTAL
	 vertical_sobel = horizontal_sobel = 0;
	 for (int i = 1; i < uiPartSize-1; i++)
	 {
		for (int j = 1; j < uiPartSize-1; j++)
		{
			vertical_sobel += fabs(saveResBlk[i-1][j+1] + 2*saveResBlk[i][j+1] + saveResBlk[i+1][j+1] - saveResBlk[i-1][j-1] - 2*saveResBlk[i][j-1] - saveResBlk[i+1][j-1]);
			horizontal_sobel += fabs(saveResBlk[i+1][j-1] + 2*saveResBlk[i+1][j] + saveResBlk[i+1][j+1] - saveResBlk[i-1][j-1] - 2*saveResBlk[i-1][j] - saveResBlk[i-1][j+1]);
		}
	 }
	 res_TotalSobelVer_CB = vertical_sobel/(uiPartSize*uiPartSize);
	 res_TotalSobelHor_CB = horizontal_sobel/(uiPartSize*uiPartSize);

	 res_TotalSobel_CB = res_TotalSobelVer_CB+res_TotalSobelHor_CB;


	  //CALC HADAMARD
  
	  // create coef block
	  Pel **coefBlk = new Pel* [uiPartSize];
	  for(int i = 0; i < uiPartSize; i++)
		  coefBlk[i] = new Pel[uiPartSize];
  
	  for (int i = 0; i < uiPartSize; i++)
	  {
		  for (int j = 0; j < uiPartSize; j++)
		  {
			  coefBlk[i][j] = 0;
		  }
	  }

	  m = uiPartSize;
	  for(int i = 0; i < uiPartSize; i++) {

		  unsigned j, bit, k;
		  double temp;

		  for (j = 0; j < m; j+=2) {
			  k = j+1;
			  coefBlk[i][j] = saveResBlk[i][j] + saveResBlk[i][k];
			  coefBlk[i][k] = saveResBlk[i][j] - saveResBlk[i][k];
		  }

		  for (bit = 2; bit < m; bit <<= 1) {   
			for (j = 0; j < m; j++) {
				if( (bit & j) == 0 ) {
					  k = j | bit;
					  temp = coefBlk[i][j];
					  coefBlk[i][j] = coefBlk[i][j] + coefBlk[i][k];
					  coefBlk[i][k] = temp - coefBlk[i][k];
				}
			}
		  }
	  }
  
	  int *sumCoefRow = new int [uiPartSize];
	  int *sumCoefCol = new int [uiPartSize];

	  for (int i = 0; i < uiPartSize; i++)
	  {
		  sumCoefRow[i] = 0;
		  sumCoefCol[i] = 0;
	  }  

	  for (int i = 0; i < uiPartSize; i++)
	  {
		  for (int j = 0; j < uiPartSize; j++)
		  {
			  sumCoefRow[i] += fabs(coefBlk[i][j]);
		  }
	  
	  }
	  sumCoefRow[0] -= fabs(coefBlk[0][0]);

	  for (int i = 0; i < uiPartSize; i++)
	  {
		  for (int j = 0; j < uiPartSize; j++)
		  {
			  sumCoefCol[i] += fabs(coefBlk[j][i]);
		  }
	  }
	  sumCoefCol[0] -= fabs(coefBlk[0][0]);


	 if(sumCoefCol[0]==0)
		res_RHV_sumCoef = (double)sumCoefRow[0];
	 else
		res_RHV_sumCoef =  (double)sumCoefRow[0] / (double)sumCoefCol[0];

	  //res_RHV_sumCoef = 0;
	  //for (int i = 0; i < uiPartSize; i++)
	  //{
		 // if(sumCoefCol[i]==0)
			//  res_RHV_sumCoef += sumCoefRow[i];
		 // else
			//  res_RHV_sumCoef +=  sumCoefRow[i]/ sumCoefCol[i];
	  //}
  
	  for(int i = 0; i < uiPartSize; i++) {
		  delete[] coefBlk[i];
	  }
	  delete[] coefBlk;

	  delete[] sumCoefRow;
	  delete[] sumCoefCol;






  // CALC VAR 3X3 VP1
  res_sumVar3x3_VP1 = 0;
  for (int i = 1; i < uiPartSize-1; i++)
  {
	  for (int j = 1; j < uiPartSize/2-1; j++)
	  {
		  double sum3x3 = 0;
		  double med3x3 = 0;
		  double sqd3x3 = 0;
		  double var3x3 = 0;
		  sum3x3 = (saveResBlk[i-1][j-1] + saveResBlk[i-1][j] + saveResBlk[i-1][j+1] + saveResBlk[i][j-1] + saveResBlk[i][j] + saveResBlk[i][j+1] + saveResBlk[i+1][j-1] + saveResBlk[i+1][j] + saveResBlk[i+1][j+1]);
		  med3x3 = (double)sum3x3 / (double)9;
		  sqd3x3 =  (saveResBlk[i-1][j-1]-med3x3)*(saveResBlk[i-1][j-1]-med3x3) +
					(saveResBlk[i-1][j]-med3x3)*(saveResBlk[i-1][j]-med3x3) +
					(saveResBlk[i-1][j+1]-med3x3)*(saveResBlk[i-1][j+1]-med3x3) +
					(saveResBlk[i][j-1]-med3x3)*(saveResBlk[i][j-1]-med3x3) +
					(saveResBlk[i][j]-med3x3)*(saveResBlk[i][j]-med3x3) +
					(saveResBlk[i][j+1]-med3x3)*(saveResBlk[i][j+1]-med3x3) +
					(saveResBlk[i+1][j-1]-med3x3)*(saveResBlk[i+1][j-1]-med3x3) +
					(saveResBlk[i+1][j]-med3x3)*(saveResBlk[i+1][j]-med3x3) +
					(saveResBlk[i+1][j+1]-med3x3)*(saveResBlk[i+1][j+1]-med3x3);
		  var3x3 = (double)sqd3x3 / (double)9;
		  res_sumVar3x3_VP1 += var3x3;
	  } 
  }

  // CALC VAR 3X3 VP2
  res_sumVar3x3_VP2 = 0;
  for (int i = 1; i < uiPartSize-1; i++)
  {
	  for (int j = uiPartSize/2+1; j < uiPartSize-1; j++)
	  {
		  double sum3x3 = 0;
		  double med3x3 = 0;
		  double sqd3x3 = 0;
		  double var3x3 = 0;
		  sum3x3 = (saveResBlk[i-1][j-1] + saveResBlk[i-1][j] + saveResBlk[i-1][j+1] + saveResBlk[i][j-1] + saveResBlk[i][j] + saveResBlk[i][j+1] + saveResBlk[i+1][j-1] + saveResBlk[i+1][j] + saveResBlk[i+1][j+1]);
		  med3x3 = (double)sum3x3 / (double)9;
		  sqd3x3 =  (saveResBlk[i-1][j-1]-med3x3)*(saveResBlk[i-1][j-1]-med3x3) +
					(saveResBlk[i-1][j]-med3x3)*(saveResBlk[i-1][j]-med3x3) +
					(saveResBlk[i-1][j+1]-med3x3)*(saveResBlk[i-1][j+1]-med3x3) +
					(saveResBlk[i][j-1]-med3x3)*(saveResBlk[i][j-1]-med3x3) +
					(saveResBlk[i][j]-med3x3)*(saveResBlk[i][j]-med3x3) +
					(saveResBlk[i][j+1]-med3x3)*(saveResBlk[i][j+1]-med3x3) +
					(saveResBlk[i+1][j-1]-med3x3)*(saveResBlk[i+1][j-1]-med3x3) +
					(saveResBlk[i+1][j]-med3x3)*(saveResBlk[i+1][j]-med3x3) +
					(saveResBlk[i+1][j+1]-med3x3)*(saveResBlk[i+1][j+1]-med3x3);
		  var3x3 = (double)sqd3x3 / (double)9;
		  res_sumVar3x3_VP2 += var3x3;
	  } 
  }

  // CALC VAR 3X3 HP1
  res_sumVar3x3_HP1 = 0;
  for (int i = 1; i < uiPartSize/2-1; i++)
  {
	  for (int j = 1; j < uiPartSize-1; j++)
	  {
		  double sum3x3 = 0;
		  double med3x3 = 0;
		  double sqd3x3 = 0;
		  double var3x3 = 0;
		  sum3x3 = (saveResBlk[i-1][j-1] + saveResBlk[i-1][j] + saveResBlk[i-1][j+1] + saveResBlk[i][j-1] + saveResBlk[i][j] + saveResBlk[i][j+1] + saveResBlk[i+1][j-1] + saveResBlk[i+1][j] + saveResBlk[i+1][j+1]);
		  med3x3 = (double)sum3x3 / (double)9;
		  sqd3x3 =  (saveResBlk[i-1][j-1]-med3x3)*(saveResBlk[i-1][j-1]-med3x3) +
					(saveResBlk[i-1][j]-med3x3)*(saveResBlk[i-1][j]-med3x3) +
					(saveResBlk[i-1][j+1]-med3x3)*(saveResBlk[i-1][j+1]-med3x3) +
					(saveResBlk[i][j-1]-med3x3)*(saveResBlk[i][j-1]-med3x3) +
					(saveResBlk[i][j]-med3x3)*(saveResBlk[i][j]-med3x3) +
					(saveResBlk[i][j+1]-med3x3)*(saveResBlk[i][j+1]-med3x3) +
					(saveResBlk[i+1][j-1]-med3x3)*(saveResBlk[i+1][j-1]-med3x3) +
					(saveResBlk[i+1][j]-med3x3)*(saveResBlk[i+1][j]-med3x3) +
					(saveResBlk[i+1][j+1]-med3x3)*(saveResBlk[i+1][j+1]-med3x3);
		  var3x3 = (double)sqd3x3 / (double)9;
		  res_sumVar3x3_HP1 += var3x3;
	  } 
  }

  // CALC VAR 3X3 HP2
  res_sumVar3x3_HP2 = 0;
  for (int i = uiPartSize/2+1; i < uiPartSize-1; i++)
  {
	  for (int j = 1; j < uiPartSize-1; j++)
	  {
		  double sum3x3 = 0;
		  double med3x3 = 0;
		  double sqd3x3 = 0;
		  double var3x3 = 0;
		  sum3x3 = (saveResBlk[i-1][j-1] + saveResBlk[i-1][j] + saveResBlk[i-1][j+1] + saveResBlk[i][j-1] + saveResBlk[i][j] + saveResBlk[i][j+1] + saveResBlk[i+1][j-1] + saveResBlk[i+1][j] + saveResBlk[i+1][j+1]);
		  med3x3 = (double)sum3x3 / (double)9;
		  sqd3x3 =  (saveResBlk[i-1][j-1]-med3x3)*(saveResBlk[i-1][j-1]-med3x3) +
					(saveResBlk[i-1][j]-med3x3)*(saveResBlk[i-1][j]-med3x3) +
					(saveResBlk[i-1][j+1]-med3x3)*(saveResBlk[i-1][j+1]-med3x3) +
					(saveResBlk[i][j-1]-med3x3)*(saveResBlk[i][j-1]-med3x3) +
					(saveResBlk[i][j]-med3x3)*(saveResBlk[i][j]-med3x3) +
					(saveResBlk[i][j+1]-med3x3)*(saveResBlk[i][j+1]-med3x3) +
					(saveResBlk[i+1][j-1]-med3x3)*(saveResBlk[i+1][j-1]-med3x3) +
					(saveResBlk[i+1][j]-med3x3)*(saveResBlk[i+1][j]-med3x3) +
					(saveResBlk[i+1][j+1]-med3x3)*(saveResBlk[i+1][j+1]-med3x3);
		  var3x3 = (double)sqd3x3 / (double)9;
		  res_sumVar3x3_HP2 += var3x3;
	  } 
  }


	  for(int i = 0; i < uiPartSize; i++)
		  delete[] saveResBlk[i];
	  delete[] saveResBlk;

	 //gcorrea: 21/10/2013 END

	  
	  // sum and mean of the residue for the two vertical halves of the PB
	  res_sum_VP1 = res_sum_VP2 = res_med_VP1 = res_med_VP2 = res_sqd_VP1 = res_sqd_VP2 = res_var_VP1 = res_var_VP2 = 0;

	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < uiPartSize; y++ )
	  {
		for ( x = 0; x < uiPartSize/2; x++ )
		{
		  res_sum_VP1 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  res_med_VP1 = res_sum_VP1/((uiPartSize*uiPartSize)/2);

	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < uiPartSize; y++ )
	  {
		for ( x = uiPartSize/2; x < uiPartSize; x++ )
		{
		  res_sum_VP2 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  res_med_VP2 = res_sum_VP2/((uiPartSize*uiPartSize)/2);


	  // sum of squared differences and var in VP1 and VP2
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < uiPartSize; y++ )
	  {
		for ( x = 0; x < uiPartSize/2; x++ )
		{
		  res_sqd_VP1 += (fabs(pDst[x]) - res_med_VP1) * (fabs(pDst[x]) - res_med_VP1);
		}
		pDst  += iDstStride;
	  }

	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < uiPartSize; y++ )
	  {
		for ( x = uiPartSize/2; x < uiPartSize; x++ )
		{
		  res_sqd_VP2 += (fabs(pDst[x]) - res_med_VP2) * (fabs(pDst[x]) - res_med_VP2);
		}
		pDst  += iDstStride;
	  }
	  
	  res_var_VP1 = res_sqd_VP2 / ((uiPartSize*uiPartSize)/2);		// variance of residue in the VP1
	  res_var_VP2 = res_sqd_VP1 / ((uiPartSize*uiPartSize)/2);		// variance of residue in the VP2
	  
	  	  
	  // sum and mean of the residue for the two horizontal halves of the PB
	  res_sum_HP1 = res_sum_HP2 = res_med_HP1 = res_med_HP2 = res_sqd_HP1 = res_sqd_HP2 = res_var_HP1 = res_var_HP2 = 0;

	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < uiPartSize/2; y++ )
	  {
		for ( x = 0; x < uiPartSize; x++ )
		{
		  res_sum_HP1 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  res_med_HP1 = res_sum_HP1/((uiPartSize*uiPartSize)/2);

	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < uiPartSize/2; y++ )
	  {
		  pDst  += iDstStride;
	  }
	  for ( y = uiPartSize/2; y < uiPartSize; y++ )
	  {
		for ( x = 0; x < uiPartSize; x++ )
		{
		  res_sum_HP2 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  res_med_HP2 = res_sum_HP2/((uiPartSize*uiPartSize)/2);


	  // sum of squared differences and var in HP1 and HP2
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < uiPartSize/2; y++ )
	  {
		for ( x = 0; x < uiPartSize; x++ )
		{
		  res_sqd_HP1 += (fabs(pDst[x]) - res_med_HP1) * (fabs(pDst[x]) - res_med_HP1);
		}
		pDst  += iDstStride;
	  }

	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < uiPartSize/2; y++ )
	  {
		  pDst  += iDstStride;
	  }
	  for ( y = uiPartSize/2; y < uiPartSize; y++ )
	  {
		for ( x = 0; x < uiPartSize; x++ )
		{
		  res_sqd_HP2 += (fabs(pDst[x]) - res_med_HP2) * (fabs(pDst[x]) - res_med_HP2);
		}
		pDst  += iDstStride;
	  }
	  
	  res_var_HP1 = res_sqd_HP2 / ((uiPartSize*uiPartSize)/2);		// variance of residue in the HP1
	  res_var_HP2 = res_sqd_HP1 / ((uiPartSize*uiPartSize)/2);		// variance of residue in the HP2


	  // computes the average of the residue in sub-block 1/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  double sumRes2Nx2N_r1 = 0;
	  for ( y = 0; y < uiPartSize/4; y++ )
	  {
		for ( x = 0; x < uiPartSize/4; x++ )
		{
		  sumRes2Nx2N_r1 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r1 = sumRes2Nx2N_r1/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 2/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  double sumRes2Nx2N_r2 = 0;
	  for ( y = 0; y < uiPartSize/4; y++ )
	  {
		for ( x = uiPartSize/4; x < uiPartSize/2; x++ )
		{
		  sumRes2Nx2N_r2 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r2 = sumRes2Nx2N_r2/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 3/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  double sumRes2Nx2N_r3 = 0;
	  for ( y = 0; y < uiPartSize/4; y++ )
	  {
		for ( x = uiPartSize/2; x < 3*uiPartSize/4; x++ )
		{
		  sumRes2Nx2N_r3 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r3 = sumRes2Nx2N_r3/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 4/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  double sumRes2Nx2N_r4 = 0;
	  for ( y = 0; y < uiPartSize/4; y++ )
	  {
		for ( x = 3*uiPartSize/4; x < uiPartSize; x++ )
		{
		  sumRes2Nx2N_r4 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r4 = sumRes2Nx2N_r4/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 5/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  double sumRes2Nx2N_r5 = 0;
	  for ( y = 0; y < (uiPartSize/4)-1; y++ )
	  {
		  pDst  += iDstStride;
	  }
	  for ( y = uiPartSize/4; y < uiPartSize/2; y++ )
	  {
		for ( x = 0; x < uiPartSize/4; x++ )
		{
		  sumRes2Nx2N_r5 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r5 = sumRes2Nx2N_r5/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 6/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  double sumRes2Nx2N_r6 = 0;
	  for ( y = 0; y < (uiPartSize/4)-1; y++ )
	  {
		  pDst  += iDstStride;
	  }
	  for ( y = uiPartSize/4; y < uiPartSize/2; y++ )
	  {
		for ( x = uiPartSize/4; x < uiPartSize/2; x++ )
		{
		  sumRes2Nx2N_r6 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r6 = sumRes2Nx2N_r6/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 7/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < (uiPartSize/4)-1; y++ )
	  {
		  pDst  += iDstStride;
	  }
	  double sumRes2Nx2N_r7 = 0;
	  for ( y = uiPartSize/4; y < uiPartSize/2; y++ )
	  {
		for ( x = uiPartSize/2; x < 3*uiPartSize/4; x++ )
		{
		  sumRes2Nx2N_r7 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r7 = sumRes2Nx2N_r7/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 8/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < (uiPartSize/4)-1; y++ )
	  {
		  pDst  += iDstStride;
	  }
	  double sumRes2Nx2N_r8 = 0;
	  for ( y = uiPartSize/4; y < uiPartSize/2; y++ )
	  {
		for ( x = 3*uiPartSize/4; x < uiPartSize; x++ )
		{
		  sumRes2Nx2N_r8 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r8 = sumRes2Nx2N_r8/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 9/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < (uiPartSize/2)-1; y++ )
	  {
		  pDst  += iDstStride;
	  }
	  double sumRes2Nx2N_r9 = 0;
	  for ( y = uiPartSize/2; y < 3*uiPartSize/4; y++ )
	  {
		for ( x = 0; x < uiPartSize/4; x++ )
		{
		  sumRes2Nx2N_r9 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r9 = sumRes2Nx2N_r9/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 10/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < (uiPartSize/2)-1; y++ )
	  {
		  pDst  += iDstStride;
	  }
	  double sumRes2Nx2N_r10 = 0;
	  for ( y = uiPartSize/2; y < 3*uiPartSize/4; y++ )
	  {
		for ( x = uiPartSize/4; x < uiPartSize/2; x++ )
		{
		  sumRes2Nx2N_r10 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r10 = sumRes2Nx2N_r10/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 11/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < (uiPartSize/2)-1; y++ )
	  {
		  pDst  += iDstStride;
	  }
	  double sumRes2Nx2N_r11 = 0;
	  for ( y = uiPartSize/2; y < 3*uiPartSize/4; y++ )
	  {
		for ( x = uiPartSize/2; x < 3*uiPartSize/4; x++ )
		{
		  sumRes2Nx2N_r11 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r11 = sumRes2Nx2N_r11/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 12/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < (uiPartSize/2)-1; y++ )
	  {
		  pDst  += iDstStride;
	  }
	  double sumRes2Nx2N_r12 = 0;
	  for ( y = uiPartSize/2; y < 3*uiPartSize/4; y++ )
	  {
		for ( x = 3*uiPartSize/4; x < uiPartSize; x++ )
		{
		  sumRes2Nx2N_r12 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r12 = sumRes2Nx2N_r12/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 13/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < (3*uiPartSize/4)-1; y++ )
	  {
		  pDst  += iDstStride;
	  }
	  double sumRes2Nx2N_r13 = 0;
	  for ( y = 3*uiPartSize/4; y < uiPartSize; y++ )
	  {
		for ( x = 0; x < uiPartSize/4; x++ )
		{
		  sumRes2Nx2N_r13 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r13 = sumRes2Nx2N_r13/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 14/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < (3*uiPartSize/4)-1; y++ )
	  {
		  pDst  += iDstStride;
	  }
	  double sumRes2Nx2N_r14 = 0;
	  for ( y = 3*uiPartSize/4; y < uiPartSize; y++ )
	  {
		for ( x = uiPartSize/4; x < uiPartSize/2; x++ )
		{
		  sumRes2Nx2N_r14 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r14 = sumRes2Nx2N_r14/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 15/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < (3*uiPartSize/4)-1; y++ )
	  {
		  pDst  += iDstStride;
	  }
	  double sumRes2Nx2N_r15 = 0;
	  for ( y = 3*uiPartSize/4; y < uiPartSize; y++ )
	  {
		for ( x = uiPartSize/2; x < 3*uiPartSize/4; x++ )
		{
		  sumRes2Nx2N_r15 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r15 = sumRes2Nx2N_r15/((uiPartSize/4)*(uiPartSize/4));

	  // computes the average of the residue in sub-block 16/16
	  pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
	  for ( y = 0; y < (3*uiPartSize/4)-1; y++ )
	  {
		  pDst  += iDstStride;
	  }
	  double sumRes2Nx2N_r16 = 0;
	  for ( y = 3*uiPartSize/4; y < uiPartSize; y++ )
	  {
		for ( x = 3*uiPartSize/4; x < uiPartSize; x++ )
		{
		  sumRes2Nx2N_r16 += fabs(pDst[x]);
		}
		pDst  += iDstStride;
	  }
	  double medRes2Nx2N_r16 = sumRes2Nx2N_r16/((uiPartSize/4)*(uiPartSize/4));

	  double r_mean_V1 = (medRes2Nx2N_r1 + medRes2Nx2N_r5 + medRes2Nx2N_r9 + medRes2Nx2N_r13) / 4;
	  double r_mean_V2 = (medRes2Nx2N_r2 + medRes2Nx2N_r6 + medRes2Nx2N_r10 + medRes2Nx2N_r14) / 4;
	  double r_mean_V3 = (medRes2Nx2N_r3 + medRes2Nx2N_r7 + medRes2Nx2N_r11 + medRes2Nx2N_r15) / 4;
	  double r_mean_V4 = (medRes2Nx2N_r4 + medRes2Nx2N_r8 + medRes2Nx2N_r12 + medRes2Nx2N_r16) / 4;
	  
	  double r_mean_H1 = (medRes2Nx2N_r1 + medRes2Nx2N_r2 + medRes2Nx2N_r3 + medRes2Nx2N_r4) / 4;
	  double r_mean_H2 = (medRes2Nx2N_r5 + medRes2Nx2N_r6 + medRes2Nx2N_r7 + medRes2Nx2N_r8) / 4;
	  double r_mean_H3 = (medRes2Nx2N_r9 + medRes2Nx2N_r10 + medRes2Nx2N_r11 + medRes2Nx2N_r12) / 4;
	  double r_mean_H4 = (medRes2Nx2N_r13 + medRes2Nx2N_r14 + medRes2Nx2N_r15 + medRes2Nx2N_r16) / 4;

	  double r_mean_Q1 = (medRes2Nx2N_r1 + medRes2Nx2N_r2 + medRes2Nx2N_r5 + medRes2Nx2N_r6) / 4;
	  double r_mean_Q2 = (medRes2Nx2N_r3 + medRes2Nx2N_r4 + medRes2Nx2N_r7 + medRes2Nx2N_r8) / 4;
	  double r_mean_Q3 = (medRes2Nx2N_r9 + medRes2Nx2N_r10 + medRes2Nx2N_r13 + medRes2Nx2N_r14) / 4;
	  double r_mean_Q4 = (medRes2Nx2N_r11 + medRes2Nx2N_r12 + medRes2Nx2N_r15 + medRes2Nx2N_r16) / 4;

	  double mad_V1 = (fabs(medRes2Nx2N_r1 - r_mean_V1) + fabs(medRes2Nx2N_r5 - r_mean_V1) + fabs(medRes2Nx2N_r9 - r_mean_V1) + fabs(medRes2Nx2N_r13 - r_mean_V1)) / 4;
	  double mad_V2 = (fabs(medRes2Nx2N_r2 - r_mean_V2) + fabs(medRes2Nx2N_r6 - r_mean_V2) + fabs(medRes2Nx2N_r10 - r_mean_V2) + fabs(medRes2Nx2N_r14 - r_mean_V2)) / 4;
	  double mad_V3 = (fabs(medRes2Nx2N_r3 - r_mean_V3) + fabs(medRes2Nx2N_r7 - r_mean_V3) + fabs(medRes2Nx2N_r11 - r_mean_V3) + fabs(medRes2Nx2N_r15 - r_mean_V3)) / 4;
	  double mad_V4 = (fabs(medRes2Nx2N_r4 - r_mean_V4) + fabs(medRes2Nx2N_r8 - r_mean_V4) + fabs(medRes2Nx2N_r12 - r_mean_V4) + fabs(medRes2Nx2N_r16 - r_mean_V4)) / 4;

	  double mad_H1 = (fabs(medRes2Nx2N_r1 - r_mean_H1) + fabs(medRes2Nx2N_r2 - r_mean_H1) + fabs(medRes2Nx2N_r3 - r_mean_H1) + fabs(medRes2Nx2N_r4 - r_mean_H1)) / 4;
	  double mad_H2 = (fabs(medRes2Nx2N_r5 - r_mean_H2) + fabs(medRes2Nx2N_r6 - r_mean_H2) + fabs(medRes2Nx2N_r7 - r_mean_H2) + fabs(medRes2Nx2N_r8 - r_mean_H2)) / 4;
	  double mad_H3 = (fabs(medRes2Nx2N_r9 - r_mean_H3) + fabs(medRes2Nx2N_r10 - r_mean_H3) + fabs(medRes2Nx2N_r11 - r_mean_H3) + fabs(medRes2Nx2N_r12 - r_mean_H3)) / 4;
	  double mad_H4 = (fabs(medRes2Nx2N_r13 - r_mean_H4) + fabs(medRes2Nx2N_r14 - r_mean_H4) + fabs(medRes2Nx2N_r15 - r_mean_H4) + fabs(medRes2Nx2N_r16 - r_mean_H4)) / 4;

	  double mad_Q1 = (fabs(medRes2Nx2N_r1 - r_mean_Q1) + fabs(medRes2Nx2N_r2 - r_mean_Q1) + fabs(medRes2Nx2N_r5 - r_mean_Q1) + fabs(medRes2Nx2N_r6 - r_mean_Q1)) / 4;
	  double mad_Q2 = (fabs(medRes2Nx2N_r3 - r_mean_Q2) + fabs(medRes2Nx2N_r4 - r_mean_Q2) + fabs(medRes2Nx2N_r7 - r_mean_Q2) + fabs(medRes2Nx2N_r8 - r_mean_Q2)) / 4;
	  double mad_Q3 = (fabs(medRes2Nx2N_r9 - r_mean_Q3) + fabs(medRes2Nx2N_r10 - r_mean_Q3) + fabs(medRes2Nx2N_r13 - r_mean_Q3) + fabs(medRes2Nx2N_r14 - r_mean_Q3)) / 4;
	  double mad_Q4 = (fabs(medRes2Nx2N_r11 - r_mean_Q4) + fabs(medRes2Nx2N_r12 - r_mean_Q4) + fabs(medRes2Nx2N_r15 - r_mean_Q4) + fabs(medRes2Nx2N_r16 - r_mean_Q4)) / 4;

	  res_rhi_V = (mad_V1 + mad_V2 + mad_V3 + mad_V4) / 4;
	  res_rhi_H = (mad_H1 + mad_H2 + mad_H3 + mad_H4) / 4;
	  res_rhi_Q = (mad_Q1 + mad_Q2 + mad_Q3 + mad_Q4) / 4;
	  
	  // calculate ratios
	  double denom;


	  if(res_sumVar3x3_VP2-res_sumVar3x3_VP1==0)
		  denom = 0.1;
	  else
		  denom = res_sumVar3x3_VP2-res_sumVar3x3_VP1;
	  res_RHV_sumVar3x3 = fabs(res_sumVar3x3_HP2-res_sumVar3x3_HP1) / fabs(denom);


	  if(res_sumVar3x3_HP2-res_sumVar3x3_HP1==0)
		  denom = 0.1;
	  else
		  denom = res_sumVar3x3_HP2-res_sumVar3x3_HP1;
	  res_RVH_sumVar3x3 = fabs(res_sumVar3x3_VP2-res_sumVar3x3_VP1) / fabs(denom);


	  if(res_sum_VP2-res_sum_VP1==0)
		  denom = 0.1;
	  else
		  denom = res_sum_VP2-res_sum_VP1;
	  res_RHV_sum = fabs(res_sum_HP2-res_sum_HP1) / fabs(denom);

	  if(res_med_VP2-res_med_VP1==0)
		  denom = 0.1;
	  else
		  denom = res_med_VP2-res_med_VP1;
	  res_RHV_med = fabs(res_med_HP2-res_med_HP1) / fabs(denom);

	  if(res_var_VP2-res_var_VP1==0)
		  denom = 0.1;
	  else
		  denom = res_var_VP2-res_var_VP1;
	  res_RHV_var = fabs(res_var_HP2-res_var_HP1) / fabs(denom);

	  if(res_rhi_V==0)
		  res_rhi_V = 0.1;
	  res_RHV_HI = (res_rhi_H / res_rhi_V);

	  if(res_HP2grad-res_HP1grad==0)
		  denom = 0.1;
	  else
		  denom = res_HP2grad-res_HP1grad;
	  res_RHV_grad = fabs(res_VP2grad-res_VP1grad) / fabs(denom);
	  
	  if(res_TotalGradHor_CB==0)
		  res_TotalGradHor_CB = 0.1;
	  res_RHV_TotalGrad_CB = res_TotalGradVer_CB / res_TotalGradHor_CB;
	 
	  if(res_HP2Sobel-res_HP1Sobel==0)
		  denom = 0.1;
	  else
		  denom = res_HP2Sobel-res_HP1Sobel;
	  res_RHV_Sobel = fabs(res_VP2Sobel-res_VP1Sobel) / fabs(denom);

	  if(res_TotalSobelHor_CB==0)
		  res_TotalSobelHor_CB = 0.1;
	  res_RHV_TotalSobel = ( res_TotalSobelVer_CB / res_TotalSobelHor_CB);
  }
  //gcorrea: 17/10/2013 END
}

Void TComYuv::subtractChroma( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  Int x, y;
  
  Pel* pSrcU0 = pcYuvSrc0->getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcU1 = pcYuvSrc1->getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcV0 = pcYuvSrc0->getCrAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcV1 = pcYuvSrc1->getCrAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDstU  = getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDstV  = getCrAddr( uiTrUnitIdx, uiPartSize );
  
  Int  iSrc0Stride = pcYuvSrc0->getCStride();
  Int  iSrc1Stride = pcYuvSrc1->getCStride();
  Int  iDstStride  = getCStride();
  for ( y = uiPartSize-1; y >= 0; y-- )
  {
    for ( x = uiPartSize-1; x >= 0; x-- )
    {
      pDstU[x] = pSrcU0[x] - pSrcU1[x];
      pDstV[x] = pSrcV0[x] - pSrcV1[x];
    }
    pSrcU0 += iSrc0Stride;
    pSrcU1 += iSrc1Stride;
    pSrcV0 += iSrc0Stride;
    pSrcV1 += iSrc1Stride;
    pDstU  += iDstStride;
    pDstV  += iDstStride;
  }
}

Void TComYuv::addAvg( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt iPartUnitIdx, UInt iWidth, UInt iHeight )
{
  Int x, y;
  
  Pel* pSrcY0  = pcYuvSrc0->getLumaAddr( iPartUnitIdx );
  Pel* pSrcU0  = pcYuvSrc0->getCbAddr  ( iPartUnitIdx );
  Pel* pSrcV0  = pcYuvSrc0->getCrAddr  ( iPartUnitIdx );
  
  Pel* pSrcY1  = pcYuvSrc1->getLumaAddr( iPartUnitIdx );
  Pel* pSrcU1  = pcYuvSrc1->getCbAddr  ( iPartUnitIdx );
  Pel* pSrcV1  = pcYuvSrc1->getCrAddr  ( iPartUnitIdx );
  
  Pel* pDstY   = getLumaAddr( iPartUnitIdx );
  Pel* pDstU   = getCbAddr  ( iPartUnitIdx );
  Pel* pDstV   = getCrAddr  ( iPartUnitIdx );
  
  UInt  iSrc0Stride = pcYuvSrc0->getStride();
  UInt  iSrc1Stride = pcYuvSrc1->getStride();
  UInt  iDstStride  = getStride();
  Int shiftNum = IF_INTERNAL_PREC + 1 - g_bitDepthY;
  Int offset = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;
  
  for ( y = 0; y < iHeight; y++ )
  {
    for ( x = 0; x < iWidth; x += 4 )
    {
      pDstY[ x + 0 ] = ClipY( ( pSrcY0[ x + 0 ] + pSrcY1[ x + 0 ] + offset ) >> shiftNum );
      pDstY[ x + 1 ] = ClipY( ( pSrcY0[ x + 1 ] + pSrcY1[ x + 1 ] + offset ) >> shiftNum );
      pDstY[ x + 2 ] = ClipY( ( pSrcY0[ x + 2 ] + pSrcY1[ x + 2 ] + offset ) >> shiftNum );
      pDstY[ x + 3 ] = ClipY( ( pSrcY0[ x + 3 ] + pSrcY1[ x + 3 ] + offset ) >> shiftNum );
    }
    pSrcY0 += iSrc0Stride;
    pSrcY1 += iSrc1Stride;
    pDstY  += iDstStride;
  }
  
  shiftNum = IF_INTERNAL_PREC + 1 - g_bitDepthC;
  offset = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;

  iSrc0Stride = pcYuvSrc0->getCStride();
  iSrc1Stride = pcYuvSrc1->getCStride();
  iDstStride  = getCStride();
  
  iWidth  >>=1;
  iHeight >>=1;
  
  for ( y = iHeight-1; y >= 0; y-- )
  {
    for ( x = iWidth-1; x >= 0; )
    {
      // note: chroma min width is 2
      pDstU[x] = ClipC((pSrcU0[x] + pSrcU1[x] + offset) >> shiftNum);
      pDstV[x] = ClipC((pSrcV0[x] + pSrcV1[x] + offset) >> shiftNum); x--;
      pDstU[x] = ClipC((pSrcU0[x] + pSrcU1[x] + offset) >> shiftNum);
      pDstV[x] = ClipC((pSrcV0[x] + pSrcV1[x] + offset) >> shiftNum); x--;
    }
    
    pSrcU0 += iSrc0Stride;
    pSrcU1 += iSrc1Stride;
    pSrcV0 += iSrc0Stride;
    pSrcV1 += iSrc1Stride;
    pDstU  += iDstStride;
    pDstV  += iDstStride;
  }
}

Void TComYuv::removeHighFreq( TComYuv* pcYuvSrc, UInt uiPartIdx, UInt uiWidht, UInt uiHeight )
{
  Int x, y;
  
  Pel* pSrc  = pcYuvSrc->getLumaAddr(uiPartIdx);
  Pel* pSrcU = pcYuvSrc->getCbAddr(uiPartIdx);
  Pel* pSrcV = pcYuvSrc->getCrAddr(uiPartIdx);
  
  Pel* pDst  = getLumaAddr(uiPartIdx);
  Pel* pDstU = getCbAddr(uiPartIdx);
  Pel* pDstV = getCrAddr(uiPartIdx);
  
  Int  iSrcStride = pcYuvSrc->getStride();
  Int  iDstStride = getStride();
  
  for ( y = uiHeight-1; y >= 0; y-- )
  {
    for ( x = uiWidht-1; x >= 0; x-- )
    {
#if DISABLING_CLIP_FOR_BIPREDME
      pDst[x ] = (pDst[x ]<<1) - pSrc[x ] ;
#else
      pDst[x ] = Clip( (pDst[x ]<<1) - pSrc[x ] );
#endif
    }
    pSrc += iSrcStride;
    pDst += iDstStride;
  }
  
  iSrcStride = pcYuvSrc->getCStride();
  iDstStride = getCStride();
  
  uiHeight >>= 1;
  uiWidht  >>= 1;
  
  for ( y = uiHeight-1; y >= 0; y-- )
  {
    for ( x = uiWidht-1; x >= 0; x-- )
    {
#if DISABLING_CLIP_FOR_BIPREDME
      pDstU[x ] = (pDstU[x ]<<1) - pSrcU[x ] ;
      pDstV[x ] = (pDstV[x ]<<1) - pSrcV[x ] ;
#else
      pDstU[x ] = Clip( (pDstU[x ]<<1) - pSrcU[x ] );
      pDstV[x ] = Clip( (pDstV[x ]<<1) - pSrcV[x ] );
#endif
    }
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
}
//! \}

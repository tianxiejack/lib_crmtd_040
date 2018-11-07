#include <math.h>
#include <stdio.h>
#include <stdlib.h>             /* malloc/free declarations */
#include <string.h>             /* memset declaration */
#include <omp.h>
#include "MSTracker.h"

namespace mv_detect{

static void MSTrkAcq(CMatchTracker *pTrackObj, cv::Mat inputFrame,cv::Rect inputParam)
{
	pTrackObj->MatchTrkAcq(inputFrame, inputParam);
	pTrackObj->bTrkAcq = TRUE;
}

static float MSTrkProc(CMatchTracker *pTrackObj, cv::Mat inputFrame,cv::Rect &outputParam)
{
	float similar;
	assert(pTrackObj->bTrkAcq);
	if(!pTrackObj->bTrkAcq){
		printf("%s: Please Acquistion Target Firstly! \n",__FUNCTION__);
		return 0.f;
	}
	similar = pTrackObj->MatchTrkProc(inputFrame, outputParam);

	return similar;
}

static void MSTrkStart(CMatchTracker *pTrackObj, bool start)
{
	pTrackObj->SetStart(start);
	if( start==false)
		pTrackObj->bTrkAcq = false;
}

static bool overlapRoi(cv::Rect rec1,	cv::Rect	rec2, cv::Rect &roi)
{
	cv::Point tl1,tl2;
	cv::Size sz1,sz2;
	tl1	= rec1.tl();
	tl2	= rec2.tl();
	sz1	= rec1.size();
	sz2	= rec2.size();
	int x_tl = std::max(tl1.x, tl2.x);
	int y_tl = std::max(tl1.y, tl2.y);
	int x_br = std::min(tl1.x + sz1.width, tl2.x + sz2.width);
	int y_br = std::min(tl1.y + sz1.height, tl2.y + sz2.height);
	if (x_tl < x_br && y_tl < y_br)
	{
		roi = cv::Rect(x_tl, y_tl, x_br - x_tl, y_br - y_tl);
		return true;
	}
	return false;
}

static void CopyTrkRect(TRKTarget	*trkTarget, cv::Rect	trkWnd)
{
	trkTarget->trkCenter.x = trkWnd.x+trkWnd.width/2;
	trkTarget->trkCenter.y = trkWnd.y+trkWnd.height/2;
	trkTarget->trkRect = trkWnd;

#if 0
	trkTarget->trkSpeed[trkTarget->trkIndx] = trkTarget->trkCenter;
	trkTarget->trkIndx = ((trkTarget->trkIndx+1)==RECORD_SPEED_FRMS)?0:(trkTarget->trkIndx+1);
#else
	if(trkTarget->trkIndx < RECORD_SPEED_FRMS){
		trkTarget->trkSpeed[trkTarget->trkIndx++] = trkTarget->trkCenter;
	}else{
		assert(trkTarget->trkIndx == RECORD_SPEED_FRMS);
		memmove(trkTarget->trkSpeed, trkTarget->trkSpeed+1, (RECORD_SPEED_FRMS-1)*sizeof(cv::Point));
		trkTarget->trkSpeed[RECORD_SPEED_FRMS-1] = trkTarget->trkCenter;
	}
#endif
}

CMSTracker::CMSTracker()
{
	memset(m_trkTarget, 0, sizeof(TRKTarget)*MAX_TGT_NUM);
	memset(m_trkTargetBak, 0, sizeof(TRKTarget)*MAX_TGT_NUM);
	memset(m_trkTargetOut, 0, sizeof(TRKTarget)*MAX_TGT_NUM);
	memset(m_trkTargetOutBak, 0, sizeof(TRKTarget)*MAX_TGT_NUM);
	memset(m_trkIdx, 0, sizeof(TRK_INDEX)*MAX_TGT_NUM);
	memset(m_trkPairIdx, 0, sizeof(TRK_INDEX)*MAX_TGT_NUM);
	m_validTrk = 0;
	m_tgtNum = MAX_TGT_NUM;
	m_overlapScaler = 1.25f;
	m_invFlag = 0;

	m_stillPixel = 6;
	m_movePixel = 16;
}

CMSTracker::~CMSTracker()
{
	int k;
	for(k=0; k<MAX_TGT_NUM; k++){
		m_matchTrack[k].unInitMatchTrk();
	}
}

void CMSTracker::SetTargetNum(int tgtNum)
{
	assert(tgtNum < MAX_TGT_NUM);
	m_tgtNum	= tgtNum;
}

void CMSTracker::SetMoveThred(int stillPixel, int movePixel)
{
	assert(stillPixel>0);
	assert(movePixel>0);
	assert(movePixel>stillPixel);

	m_stillPixel = stillPixel;
	m_movePixel = movePixel;
}

void CMSTracker::SetLapScaler(float lapScaler)
{
	assert(lapScaler>1.0);
	m_overlapScaler = lapScaler;
}

int	CMSTracker::SetKalmanFilter(bool bKalman)
{
	int k;
	for(k=0; k<MAX_TGT_NUM; k++){
		m_matchTrack[k].SetKalmanFilter(bKalman);
	}
	return 0;
}

void CMSTracker::initMS(Mat image, TRKTarget	*trkTarget, int tgtIndex)
{
	cv::Rect acqWnd = trkTarget->trkRect;

	memset(m_trkTarget[tgtIndex].trkSpeed, 0, sizeof(cv::Point)*RECORD_SPEED_FRMS);//Reset speed record
	m_trkTarget[tgtIndex].trkIndx = 0;

	MSTrkAcq(&m_matchTrack[tgtIndex], image, acqWnd);
	MSTrkStart(&m_matchTrack[tgtIndex], TRUE);
	CopyTrkRect(trkTarget, acqWnd);
	trkTarget->trkState = true;
	
	m_trkTarget[tgtIndex].trkLives = 0;
	m_trkTarget[tgtIndex].trkFrames = 0;
	m_trkPairIdx[tgtIndex].valid = 0;
}

int CMSTracker::DeletMSTarget(int tgtIndex)
{
	m_trkTarget[tgtIndex].valid = 0;    
	m_trkTarget[tgtIndex].trkState = false;
	m_trkTarget[tgtIndex].trkLives = 0;
	m_trkTarget[tgtIndex].trkFrames = 0;
	memset(m_trkTarget[tgtIndex].trkSpeed, 0, sizeof(cv::Point)*RECORD_SPEED_FRMS);
	m_trkTarget[tgtIndex].trkIndx = 0;
	MSTrkStart(&m_matchTrack[tgtIndex],FALSE);

	if(m_trkPairIdx[tgtIndex].valid){
		int pairIdx = m_trkPairIdx[tgtIndex].pairIdx;
		m_trkPairIdx[tgtIndex].valid = 0;
		m_trkIdx[pairIdx].valid = 0;

		m_trkTargetOut[pairIdx].valid = 0;
		m_trkTargetOut[pairIdx].trkState = false;
	}
	return 0;
}

void CMSTracker::ClearAllTrkTarget()
{
	int k;
	 for(k=0; k<MAX_TGT_NUM; k++){
		 DeletMSTarget(k);
	 }
}

void CMSTracker::MergeMSTarget(Mat frame, cv::Rect roiRect)
{
	int i, j, k;
	cv::Rect		trkRect1, trkRect2, roi;
	cv::Point 		pt0, pt1;
	int dist0, dist1;
	float	flap1, flap2, ratio;
	int x_minrange, x_maxrange, y_minrange, y_maxrange;
	for(k=0; k<MAX_TGT_NUM; k++){//nearest edge
		if(m_trkTarget[k].trkState){
			x_minrange = m_trkTarget[k].trkRect.x < (roiRect.x + BORADERWIDTH) ? 1:0;
			x_maxrange = (m_trkTarget[k].trkRect.x+ m_trkTarget[k].trkRect.width)> (roiRect.x+roiRect.width-BORADERWIDTH) ? 1:0;
			y_minrange = m_trkTarget[k].trkRect.y < (roiRect.y+BORADERWIDTH) ? 1:0;
			y_maxrange = (m_trkTarget[k].trkRect.y+ m_trkTarget[k].trkRect.height)>(roiRect.y+roiRect.height-BORADERWIDTH) ? 1:0;

			if(x_minrange ||  x_maxrange || y_minrange || y_maxrange) {
				DeletMSTarget( k);
			}
		}
	}
	for(i=0; i<MAX_TGT_NUM; i++){//overlap
		dist0 = 0;
		if(m_trkTarget[i].trkIndx == RECORD_SPEED_FRMS){
			pt0 = m_trkTarget[i].trkSpeed[0];
			pt1 = m_trkTarget[i].trkSpeed[RECORD_SPEED_FRMS-1];
			dist0 = (int)sqrt((pt0.x-pt1.x)*(pt0.x-pt1.x)+(pt0.y-pt1.y)*(pt0.y-pt1.y));
		}
		for(j=i+1; j<MAX_TGT_NUM; j++){
			if(m_trkTarget[i].trkState && m_trkTarget[j].trkState){
				trkRect1 = m_trkTarget[i].trkRect;
				trkRect2 = m_trkTarget[j].trkRect;
				if(overlapRoi(trkRect1, trkRect2, roi)){//overlap is very high
					flap1	= roi.area()*1.f/trkRect1.area();
					flap2	= roi.area()*1.f/trkRect2.area();
					dist1 = 0;
					if(m_trkTarget[j].trkIndx == RECORD_SPEED_FRMS){
						pt0 = m_trkTarget[j].trkSpeed[0];
						pt1 = m_trkTarget[j].trkSpeed[RECORD_SPEED_FRMS-1];
						dist1 = (int)sqrt((pt0.x-pt1.x)*(pt0.x-pt1.x)+(pt0.y-pt1.y)*(pt0.y-pt1.y));
					}
					ratio = flap1/flap2;
					if(m_trkPairIdx[i].valid==1 && m_trkPairIdx[j].valid==0){
						DeletMSTarget(j);
						continue;
					}else if(m_trkPairIdx[i].valid==0 && m_trkPairIdx[j].valid==1){
						DeletMSTarget(i);
						continue;
					}
					if(flap1 > flap2){
						if(flap1>0.1||flap2>0.1){
							if(ratio<1.4&&dist0>15 && dist1<15){
								DeletMSTarget(j);
							}else{
								DeletMSTarget(i);
							}
						}
					}else{
						if(flap1>0.1||flap2>0.1){
							if(ratio>0.7&&dist1>15&&dist0<15){
								DeletMSTarget(i);
							}else{
								DeletMSTarget(j);
							}
						}
					}
				}
			}
		}
	}
/*
	for(i=0; i<MAX_TGT_NUM; i++){//too nearest
		if(m_trkTarget[i].trkState){
			pt0 = m_trkTarget[i].trkCenter;
			for(j=i+1; j<MAX_TGT_NUM; j++){
				if(m_trkTarget[j].trkState){
					pt1 = m_trkTarget[j].trkCenter;

				}
			}
		}
	}
*/
	for(k=0; k<MAX_TGT_NUM; k++){//struct
		if(m_trkTarget[k].trkState){
			int SSIM;
			cv::Rect tgtPos;
			tgtPos.width = (int)(m_trkTarget[k].trkRect.width*1.5);
			tgtPos.height = (int)(m_trkTarget[k].trkRect.height*1.5);
			tgtPos.x = m_trkTarget[k].trkCenter.x - tgtPos.width/2;
			tgtPos.y = m_trkTarget[k].trkCenter.y- tgtPos.height/2;
			SSIM = m_matchTrack[k].CalTgtSTRUCT(frame, tgtPos, NULL);
	
			if(SSIM < 10){
				DeletMSTarget( k);
			}
		}
	}
}

void CMSTracker::MSprocess(Mat image, int tgtIndex)
{
	cv::Rect trkRect;
	float similar = MSTrkProc(&m_matchTrack[tgtIndex], image, trkRect);
	CopyTrkRect(&m_trkTarget[tgtIndex], trkRect);
	if(similar == 0.f){
		DeletMSTarget(tgtIndex);
	}
}

bool CMSTracker::JudgeTrkInTarget(std::vector<TRK_RECT_INFO>	MVTgtObj, cv::Rect trkRect)
{
	int i, tgtNum = MVTgtObj.size();
	cv::Rect tgtIncRect, tgtRect, roi;
	bool	bOverlap = false;

	for(i=0; i<tgtNum; i++){
		tgtRect	=MVTgtObj[i].targetRect;
		tgtIncRect.width = tgtRect.width*m_overlapScaler;
		tgtIncRect.height = tgtRect.height*m_overlapScaler;
		tgtIncRect.x = (tgtRect.x + tgtRect.width/2)-tgtIncRect.width/2;
		tgtIncRect.y = (tgtRect.y + tgtRect.height/2)-tgtIncRect.height/2;
		if(overlapRoi(tgtIncRect, trkRect, roi)){ // overlap
			bOverlap |= true;
			break;
		}
	}

	return bOverlap;
}

void CMSTracker::MSLiveJudge(std::vector<TRK_RECT_INFO>	MVTgtObj)
{
	int k;
	cv::Rect trkRect;
	bool	bOverlap;
	for(k=0; k<MAX_TGT_NUM; k++){
		if(m_matchTrack[k].TrkState){//already acq, and then track
			trkRect	= m_trkTarget[k].trkRect;
			bOverlap = JudgeTrkInTarget(MVTgtObj, trkRect);
			m_trkTarget[k].trkLives += (bOverlap)?1:0;
			m_trkTarget[k].trkFrames++;
			if(m_trkTarget[k].trkFrames >TRK_FRAME_NUM){
				if(m_trkTarget[k].trkLives == 0){
					DeletMSTarget( k);
				}
				m_trkTarget[k].trkLives = 0;
				m_trkTarget[k].trkFrames = 0;
			}
		}
	}
}

void CMSTracker::MSAcqTarget(Mat frame,  std::vector<TRK_RECT_INFO>	 MVTgtObj, int tgtIndex)
{
	int i, j, k, acqIndex;
	cv::Rect tgtRect, trkRect, roi, tgtIncRect;
	bool	bOverlap;
	int tgtNum = MVTgtObj.size();

	for(i=0; i<tgtNum; i++){

		bOverlap = false;
		tgtRect	=MVTgtObj[i].targetRect;

		tgtIncRect.width = tgtRect.width*m_overlapScaler;
		tgtIncRect.height = tgtRect.height*m_overlapScaler*1.2;
		tgtIncRect.x = (tgtRect.x + tgtRect.width/2)-tgtIncRect.width/2;
		tgtIncRect.y = (tgtRect.y + tgtRect.height/2)-tgtIncRect.height/2;
		if(tgtIncRect.x <0) tgtIncRect.x = 0;
		if(tgtIncRect.y <0) tgtIncRect.y = 0;
		if(tgtIncRect.x + tgtIncRect.width > frame.cols) tgtIncRect.width = frame.cols - tgtIncRect.x;
		if(tgtIncRect.y + tgtIncRect.height > frame.rows) tgtIncRect.height = frame.rows - tgtIncRect.y;

		for(k=0; k<MAX_TGT_NUM; k++){
			if(m_matchTrack[k].TrkState){
				trkRect = m_trkTarget[k].trkRect;
				if(overlapRoi(tgtIncRect, trkRect, roi)){ // overlap
					bOverlap |= true;
					break;
				}
			}
		}
		if(!bOverlap){//no overlap, and then acq target;
			m_trkTarget[tgtIndex].trkRect = tgtIncRect;//MVTgtObj[i].targetRect;
			initMS(frame, &m_trkTarget[tgtIndex], tgtIndex);
			break;
		}
	}
}

void CMSTracker::Process(Mat frame, std::vector<TRK_RECT_INFO>	MVTgtObj,  int bAcq /*= 1*/)
{
	int i, k;
#pragma omp parallel for
	for(k=0; k<MAX_TGT_NUM; k++){
		if(m_matchTrack[k].TrkState){//already acq, and then track
			MSprocess(frame, k);
		}
	}

	m_invFlag++;
	if(bAcq){
		for(k=0; k<MAX_TGT_NUM; k++){
			if(!m_matchTrack[k].TrkState){//need to acq target
				MSAcqTarget(frame, MVTgtObj, k);
			}
		}
	}
	cv::Rect roi;
	roi.x = roi.y = 0;
	roi.width = frame.cols;
	roi.height = frame.rows;
	MergeMSTarget(frame, roi);
	MSLiveJudge(MVTgtObj);
	MSTrkFilter();
}

void CMSTracker::ProcessRect(Mat frame, std::vector<TRK_RECT_INFO>	 MVTgtObj, cv::Rect roi, int bAcq /*= 1*/)
{
	int i, k;
#pragma omp parallel for
	for(k=0; k<MAX_TGT_NUM; k++){
		if(m_matchTrack[k].TrkState){//already acq, and then track
			MSprocess(frame, k);
		}
	}

	m_invFlag++;
	if(bAcq){
		for(k=0; k<MAX_TGT_NUM; k++){
			if(!m_matchTrack[k].TrkState){//need to acq target
				MSAcqTarget(frame, MVTgtObj, k);
			}
		}
	}

	MergeMSTarget(frame, roi);
	MSLiveJudge(MVTgtObj);
	MSTrkFilter();
}

void CMSTracker::MSTrkFilter()
{
	int i, j, k, index, dist;
	cv::Point pt0, pt1;
	for(k=0; k<m_tgtNum; k++){
		if(m_trkIdx[k].valid){
			m_trkTargetOut[k] = m_trkTarget[m_trkIdx[k].pairIdx];
			assert(m_trkPairIdx[m_trkIdx[k].pairIdx].valid == 1);
			assert(m_trkPairIdx[m_trkIdx[k].pairIdx].pairIdx == k);
		}
	}

	for(k=0; k<m_tgtNum; k++){
		if(m_trkIdx[k].valid == 0 ){
			index = -1;
			for(i=0; i<MAX_TGT_NUM; i++){
				if(m_trkPairIdx[i].valid == 0  && m_trkTarget[i].trkState){
					index = i;
					break;
				}
			}
			if(index != -1){
				m_trkIdx[k].pairIdx = index;
				m_trkIdx[k].valid = 1;
				m_trkPairIdx[index].pairIdx = k;
				m_trkPairIdx[index].valid = 1;

				m_trkTarget[index].trkIndx = 0;
				m_trkTargetOut[k] = m_trkTarget[m_trkIdx[k].pairIdx];
			}
		}
	}

	for(k=0; k<m_tgtNum; k++){
		if(m_trkIdx[k].valid && m_trkTargetOut[k].trkState==0 ){
			index = -1;
			for(i=0; i<MAX_TGT_NUM; i++){
				if(m_trkPairIdx[i].valid == 0 && m_trkTarget[i].trkState){
					index = i;
					break;
				}
			}
			if(index != -1){
				m_trkPairIdx[m_trkIdx[k].pairIdx].valid = 0;
				m_trkIdx[k].pairIdx = index;
				m_trkPairIdx[index].valid = 1;
				m_trkPairIdx[index].pairIdx = k;
				m_trkTarget[index].trkIndx = 0;
				m_trkTargetOut[k] = m_trkTarget[m_trkIdx[k].pairIdx];
			}
		}
	}

	for(k=m_tgtNum-1; k>=0; k--){
		if(m_trkIdx[k].valid && m_trkTargetOut[k].trkState){
			if(m_trkTargetOut[k].trkIndx == RECORD_SPEED_FRMS){
				pt0 = m_trkTargetOut[k].trkSpeed[0];
				pt1 = m_trkTargetOut[k].trkSpeed[RECORD_SPEED_FRMS-1];
				dist = (int)sqrt((pt0.x-pt1.x)*(pt0.x-pt1.x)+(pt0.y-pt1.y)*(pt0.y-pt1.y));
				if(dist <m_stillPixel){//still status
					index = -1;
					for(i=0; i<MAX_TGT_NUM; i++){
						if(m_trkPairIdx[i].valid == 0 && m_trkTarget[i].trkState && m_trkTarget[i].trkIndx == RECORD_SPEED_FRMS){
							pt0 = m_trkTarget[i].trkSpeed[0];
							pt1 = m_trkTarget[i].trkSpeed[RECORD_SPEED_FRMS-1];
							dist = (int)sqrt((pt0.x-pt1.x)*(pt0.x-pt1.x)+(pt0.y-pt1.y)*(pt0.y-pt1.y));
							if(dist>m_movePixel){
								index = i;
								break;
							}
						}
					}
					if(index != -1){
						m_trkPairIdx[m_trkIdx[k].pairIdx].valid = 0;
						m_trkIdx[k].pairIdx = index;
						m_trkPairIdx[index].valid = 1;
						m_trkPairIdx[index].pairIdx = k;
						m_trkTarget[index].trkIndx = 0;
						m_trkTargetOut[k] = m_trkTarget[m_trkIdx[k].pairIdx];
					}
				}
			}
		}
	}

	for(i=0; i<MAX_TGT_NUM; i++){
		if(m_trkPairIdx[i].valid == 0 && m_trkTarget[i].trkState && m_trkTarget[i].trkIndx == RECORD_SPEED_FRMS){
			pt0 = m_trkTarget[i].trkSpeed[0];
			pt1 = m_trkTarget[i].trkSpeed[RECORD_SPEED_FRMS-1];
			dist = (int)sqrt((pt0.x-pt1.x)*(pt0.x-pt1.x)+(pt0.y-pt1.y)*(pt0.y-pt1.y));
			if(dist <m_stillPixel){
				DeletMSTarget(i);
			}
		}
	}
}

void CMSTracker::DrawTrkTarget(cv::Mat src,  cv::Mat osd, bool bShow /*= true*/)
{
	char strDisplay[100];
	int i,nWidth, nHeight;
	TRKTarget	*pTrkTgt;
	cv::Scalar	color, colorword;
	cv::Rect	result;
	nWidth	= src.cols;
	nHeight	= src.rows;
	if(bShow){
		color = cv::Scalar(0x00, 0xFF, 0xFF,0xFF);
		pTrkTgt = (TRKTarget	*)m_trkTargetOut;
		colorword = cv::Scalar(0,255,0,0xFF);
	}else{
		color = cv::Scalar(0x00, 0xFF, 0xFF,0x00);
		pTrkTgt = (TRKTarget	*)m_trkTargetOutBak;
		colorword = cv::Scalar(0,255,0,0x00);
	}
	for(i = 0; i<m_tgtNum; i++){
		if(!pTrkTgt[i].trkState)
			continue;
		result	= pTrkTgt[i].trkRect;

		result.x = result.x*osd.cols/src.cols;
		result.y = result.y*osd.rows/src.rows;
		result.width = result.width*osd.cols/src.cols;
		result.height = result.height*osd.rows/src.rows;

		cv::rectangle( osd, cv::Point( result.x, result.y ), cv::Point( result.x+result.width, result.y+result.height), color, 4, 8 );
		sprintf(strDisplay, "%d ", i) ;
		putText(osd, strDisplay,Point( result.x+4, result.y+4 ), 1, 2, colorword,4);
	}
	if(bShow){
		memcpy(m_trkTargetOutBak, m_trkTargetOut, sizeof(TRKTarget)*MAX_TGT_NUM);
	}
}
}

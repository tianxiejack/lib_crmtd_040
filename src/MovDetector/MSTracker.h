#ifndef _MS_TRACKER_
#define _MS_TRACKER_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
//#include "prehead.h"
#include "mmthead.h"
#include "MatchTracker.h"
#include "infoHead.h"

using namespace cv;
using namespace std;

namespace mv_detect{

#define		BORADERWIDTH       32
#define		TRK_FRAME_NUM	 	40
#define		RECORD_SPEED_FRMS		15

typedef struct _trk_target_t{
	int				valid;
	int				trkLives;
	int				trkFrames;
	cv::Rect		trkRect;
	cv::Point		trkCenter;
	bool			trkState;
	cv::Point		trkSpeed[RECORD_SPEED_FRMS];
	int				trkIndx;
}TRKTarget;

typedef struct _index_t{
	int valid;
	int pairIdx;
}TRK_INDEX;

class CMSTracker
{
public:
	CMSTracker();
	~CMSTracker();

public:
	void initMS(Mat image, TRKTarget	*trkTarget, int tgtIndex);
	void MSprocess(Mat image, int tgtIndex);
	void Process(Mat frame, std::vector<TRK_RECT_INFO>	 MVTgtObj,int bAcq = 1);
	void ProcessRect(Mat frame, std::vector<TRK_RECT_INFO>	 MVTgtObj, cv::Rect roi, int bAcq = 1);
	void MSAcqTarget(Mat frame, std::vector<TRK_RECT_INFO> MVTgtObj, int tgtIndex);
	int	 DeletMSTarget(int tgtIndex);
	void MergeMSTarget(Mat frame, cv::Rect roiRect);
	void DrawTrkTarget(cv::Mat src, cv::Mat osd, bool bShow = true);
	void ClearAllTrkTarget();
	void SetTargetNum(int tgtNum);
	void MSTrkFilter();
	void SetMoveThred(int stillPixel, int movePixel);
	void SetLapScaler(float lapScaler);
	int	  SetKalmanFilter(bool bKalman);
	void MSLiveJudge(std::vector<TRK_RECT_INFO>	MVTgtObj);
	bool JudgeTrkInTarget(std::vector<TRK_RECT_INFO> MVTgtObj, cv::Rect trkRect);

public:
	CMatchTracker	m_matchTrack[MAX_TGT_NUM];
	TRKTarget		m_trkTarget[MAX_TGT_NUM];
	TRKTarget		m_trkTargetBak[MAX_TGT_NUM];
	int					m_validTrk;
	int					m_tgtNum;
	TRKTarget		m_trkTargetOut[MAX_TGT_NUM];
	TRKTarget		m_trkTargetOutBak[MAX_TGT_NUM];
	TRK_INDEX	m_trkIdx[MAX_TGT_NUM];
	TRK_INDEX	m_trkPairIdx[MAX_TGT_NUM];
	float				m_overlapScaler;
	int					m_invFlag;
	int					m_stillPixel;
	int					m_movePixel;
};

}

#endif

#ifndef		_MATCH_TRACKER_
#define		_MATCH_TRACKER_
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "Kalman.h"

namespace mv_detect{

class	CMatchTracker{
public:
	CMatchTracker();
	~CMatchTracker();

public:
	void	InitMatchTrk(cv::Rect acqRect, double DeltaT,int DP, int MP, int CP);
	void 	unInitMatchTrk();
	void	MatchTrkAcq(cv::Mat frame, cv::Rect	acqRect);
	float	MatchTrkProc(cv::Mat frame, cv::Rect &trkRect);//return similar
	int		SetStart(bool bStart);
	int		SetKalmanFilter(bool bKalman);

	void Kalman(double *measure, double *control);
	void KalmanPredict(int xout, int yout);

	void CalSSIM(unsigned char *pIMG0, unsigned char *pIMG1, int width, int height, double *SSIM);
	int CalTgtSTRUCT(cv::Mat image, cv::Rect inputParam, int *SSIM);

public:
	CKalman_mv* m_pKalmanProc;
	double*  m_pMeasure;
	double*  m_pControl;

	cv::Mat		m_tmplModel;
	cv::Mat		m_curImg;

	cv::Rect	m_trackParam;
	cv::Rect	m_inputParam;
	cv::Rect	m_outputParam;

	bool	m_bInited;
	bool	bTrkAcq;
	bool   TrkState;
	bool	m_bKalman;
};

}
#endif

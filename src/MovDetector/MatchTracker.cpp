#include "MatchTracker.h"
#define FALSE 0
#define TRUE  1

namespace mv_detect{

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

CMatchTracker::CMatchTracker():m_pKalmanProc(NULL),m_pMeasure(NULL),m_pControl(NULL)
{
	m_bInited = false;
	TrkState = false;
	bTrkAcq = false;
	m_bKalman = false;
}

CMatchTracker::~CMatchTracker()
{
	unInitMatchTrk();
}

void CMatchTracker::Kalman(double *measure, double *control)
{
	m_pKalmanProc->KalmanPredict(control);
	m_pKalmanProc->KalmanCorrect(measure);
}

void CMatchTracker::KalmanPredict(int xout, int yout)
{
	m_pMeasure[0] = (double)xout;
	m_pMeasure[1] = (double)yout;
	Kalman(m_pMeasure,	NULL);
}

void CMatchTracker::unInitMatchTrk()
{
	if (m_pKalmanProc != NULL)
	{
		delete m_pKalmanProc;
		m_pKalmanProc = NULL;
	}
	if (m_pMeasure != NULL)
	{
		delete m_pMeasure;
		m_pMeasure = NULL;
	}
	if (m_pControl != NULL)
	{
		delete m_pControl;
		m_pControl = NULL;
	}
	m_bInited = FALSE;
	TrkState = FALSE;
}

void	CMatchTracker::InitMatchTrk(cv::Rect acqRect, double DeltaT,int DP, int MP, int CP)
{
	int x0, y0;
	if (m_bInited){
		unInitMatchTrk();
	}
	if (m_pKalmanProc == NULL){
		m_pKalmanProc = new CKalman_mv();
		if ( m_pKalmanProc == NULL){
			m_bInited = FALSE;
			return;
		}
	}
	m_pKalmanProc->KalmanOpen(DP, MP, CP);
	if (!m_pKalmanProc->m_bInited){
		m_bInited = FALSE;
		return;
	}
	x0 = acqRect.x + acqRect.width/2;
	y0 = acqRect.y + acqRect.height/2;
	m_pKalmanProc->KalmanInitParam(x0, y0, DeltaT);

	if (m_pMeasure == NULL){
		m_pMeasure = new double[MP * 1];
		memset(m_pMeasure, 0, sizeof(*m_pMeasure));
	}
	if (m_pControl == NULL && CP > 0){
		m_pControl = new double[CP * 1];
		memset(m_pControl, 0, sizeof(*m_pControl));
	}
	m_bInited = TRUE;
}

void	CMatchTracker::MatchTrkAcq(cv::Mat frame, cv::Rect	acqRect)
{
	double delta = 1/30.0;
	int DP =	4, MP = 2, CP = 0;
	InitMatchTrk(acqRect, delta, DP, MP, CP);
	m_trackParam = acqRect;
	frame(acqRect).convertTo(m_tmplModel, CV_32F);
	m_inputParam = acqRect;
	m_outputParam = acqRect;
}

#define		INCREASE_WIDTH	25
float	CMatchTracker::MatchTrkProc(cv::Mat frame, cv::Rect &trkRect)
{
	if(m_pKalmanProc == NULL || !m_pKalmanProc->m_bInited || !m_bInited){
		return 0.0;
	}
	float similar = 0.f;
	int xin, yin, xin1, yin1, W, H, Win, Hin;
	cv::Rect	compRect, imageRect, roi;

	W = frame.cols;		H = frame.rows;
	Win = m_inputParam.width;	Hin = m_inputParam.height;

	xin = (m_inputParam.x+m_inputParam.width/2);
	yin = (m_inputParam.y+m_inputParam.height/2);
	if(m_bKalman){
		xin1 = (floor)( xin + m_pKalmanProc->state_post[2] *m_pKalmanProc->deltat + 0.5 );//Kalman filter
		yin1 = (floor)( yin + m_pKalmanProc->state_post[3] *m_pKalmanProc->deltat + 0.5 );
	}else{
		xin1 = (floor)( xin);
		yin1 = (floor)( yin);
	}

	xin1 -= Win/2;
	yin1 -= Hin/2;
	if(xin1<0)	xin1 = 0;
	if((xin1+Win/2)>W)	xin1 = W - Win/2;
	if(yin1<0)	yin1 = 0;
	if((yin1+Hin/2)>H)	yin1 = H - Hin/2;

	compRect.x = xin1-	INCREASE_WIDTH;
	compRect.y = yin1-	INCREASE_WIDTH;
	compRect.width = Win + INCREASE_WIDTH*2;
	compRect.height = Hin + INCREASE_WIDTH*2;
	imageRect.x = imageRect.y = 0;
	imageRect.width = W;	imageRect.height = H;
	assert(overlapRoi(compRect, imageRect, roi));
	compRect = roi;
	if(compRect.width < Win){
		compRect.width = Win;
		if((compRect.x+compRect.width)>W)	compRect.x = W-compRect.width;
	}
	if(compRect.height < Hin){
		compRect.height = Hin;
		if((compRect.y+compRect.height)>H)	compRect.y = H-compRect.height;
	}
	frame(compRect).convertTo(m_curImg, CV_32F);
	cv::Mat result, curImg;
	 double minVal, maxVal;
	 cv::Point minLoc,maxLoc;
	matchTemplate(m_curImg, m_tmplModel, result, CV_TM_CCOEFF_NORMED);
	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

	if(maxLoc.x == 0 && maxLoc.y == 0){
		maxVal = 0.0;
		trkRect = m_inputParam;
	}else{
		trkRect.x = compRect.x+maxLoc.x;
		trkRect.y = compRect.y+maxLoc.y;
		trkRect.width = Win;
		trkRect.height = Hin;
		m_inputParam = trkRect;
		m_outputParam = trkRect;
	}
	similar = maxVal;

//	if(maxVal > 0.75)
	{
		double alpha, beta, gamma;
		frame(trkRect).convertTo(curImg, CV_32F);
		alpha = 0.5;	beta = 1-alpha;	gamma = 0;
		addWeighted(curImg, alpha, m_tmplModel, beta, gamma, m_tmplModel);
	}
	if(m_bKalman){
		m_pMeasure[0] = (double)(trkRect.x+Win/2);
		m_pMeasure[1] = (double)(trkRect.y+Hin/2);
		Kalman(m_pMeasure,	NULL);
	}

	return similar;
}

int	CMatchTracker::SetStart(bool bStart)
{
	if(bStart)
		TrkState = 1;
	else
		TrkState = 0;
	return 0;
}

int	CMatchTracker::SetKalmanFilter(bool bKalman)
{
	m_bKalman = bKalman;
	return 0;
}

void CMatchTracker::CalSSIM(unsigned char *pIMG0, unsigned char *pIMG1, int width, int height, double *SSIM)
{
	int x, y;
	int meanv0, vsquared0, meanv1, vsquared1, mixsquare;
	unsigned char *pIn0, *pIn1;

	int w,h;
	double C1, C2, C3;
	double rslt1, rslt2, rslt3;

	h = height;
	w = width;

	meanv0 = meanv1 = 0;
	vsquared0 = vsquared1 = 0;
	mixsquare = 0;

	pIn0 = pIMG0;
	pIn1 = pIMG1;
	for(y=0; y<h; y++)
	{
		for(x=0; x<w; x++)
		{
			meanv0 += pIn0[y*w+x];
			meanv1 += pIn1[y*w+x];
		}
	}
	meanv0 /= (w*h);
	meanv1 /= (w*h);
	pIn0 = pIMG0;
	pIn1 = pIMG1;
	for(y=0; y<h; y++)
	{
		for(x=0; x<w; x++)
		{
			vsquared0 += (pIn0[y*w+x]-meanv0)*(pIn0[y*w+x]-meanv0);
			vsquared1 += (pIn1[y*w+x]-meanv1)*(pIn1[y*w+x]-meanv1);
			mixsquare += (pIn0[y*w+x]-meanv0)*(pIn1[y*w+x]-meanv1);
		}
	}
	vsquared0 /= (w*h);
	vsquared1 /= (w*h);
	mixsquare /= (w*h);
	vsquared0 = (int)(sqrt((double)vsquared0));
	vsquared1 = (int)(sqrt((double)vsquared1));

	C1 = ((meanv0==0 && meanv1==0) == 1)?(1e-6):(0.0);
	C2 = ((vsquared0==0 &&  vsquared1==0) == 1)?(1e-6):(0.0);
	C3 =  ((vsquared0==0 &&  vsquared1==0) == 1)?(1e-6):(0.0);

	rslt1 = ((2*meanv0*meanv1+C1)/(meanv0*meanv0+meanv1*meanv1+C1));
	rslt2 = ((2*vsquared0*vsquared1+C2)/(vsquared0*vsquared0+vsquared1*vsquared1+C2));
	rslt3 = ((mixsquare+C3)/(vsquared0*vsquared1+C3));
	*SSIM = rslt1*rslt2*rslt3;
}

int CMatchTracker::CalTgtSTRUCT(cv::Mat image, cv::Rect inputParam, int *SSIM)
{
	int vsquared = 0;
	cv::Scalar mean_v, sqruared_v;
	cv::Rect imageRect, roi;

	imageRect.x = imageRect.y = 0;
	imageRect.width = image.cols;
	imageRect.height = image.rows;
	overlapRoi(inputParam, imageRect, roi);
	inputParam = roi;

	meanStdDev(image(inputParam), mean_v, sqruared_v);

	vsquared = (int)sqruared_v.val[0];
	if(SSIM != NULL){
		*SSIM  = vsquared;
	}
	return vsquared;
}

}



#ifndef		_MMT_HEAD_
#define		_MMT_HEAD_

namespace mv_detect{

#define		MAX_TGT_NUM 	64

typedef struct{
	int valid;
	cv::Rect Box;
}TARGETBOX;

}

#endif

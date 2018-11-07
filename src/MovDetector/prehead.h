#ifndef  _PRE_HEAD_
#define  _PRE_HEAD_

#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#define FALSE 0
#define TRUE  1

#define MAKEYUV(y,u,v)		((unsigned int)(((unsigned int)(y)|((unsigned int)((unsigned int)(u))<<8))|(((unsigned int)(unsigned int)(v))<<16)))
#define GETYUV(y,u,v, yuv)	(y) = yuv & 0xff;	(u) = ((yuv)>>8) & 0xff; (v) = ((yuv)>>16) & 0xff
#define GETYVAL(yuv)		(yuv & 0xff)
#define GETUVAL(yuv)		(((yuv)>>8) & 0xff)
#define GETVVAL(yuv)		(((yuv)>>16) & 0xff)

#define WHITECOLOR 		0x008080BE
#define YELLOWCOLOR 	0x009110D2
#define CRAYCOLOR		0x0010A6AA
#define GREENCOLOR		0x00223691
#define MAGENTACOLOR	0x00DECA6A
#define REDCOLOR		0x00F05A51
#define BLUECOLOR		0x006EF029
#define BLACKCOLOR		0x00808010
#define BLANKCOLOR		0x00000000

typedef struct _cp_state_t{
	float  posX;
	float  posY;
	float  velocityX;
	float  velocityY;
}CP_STATE;

#define  StateP     4
#define  MeasureP   2
#define  ControlP   0

typedef struct _img_frame_t{
	unsigned char *addr[3];
	int nWidth;
	int nHeight;
	int  pitch[3];
}IMGFrame;

typedef struct _track_p{
	int  centerX;
	int  centerY;
	int  trackW; //half w
	int  trackH; //halt h
}TRACK_P;

typedef struct _track_error{
	int     centerX;
	int     centerY;
	int		offsetX;
	int		offsetY;
	double  corrValue;
}TRACK_ERROR;

typedef struct _track_point{
	float   pointX;
	float   pointY;
}TRACK_POINT;

#endif

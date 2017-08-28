#include "../../include/location_base/Marker.h"

Marker::Marker(void)
{
	id = -1;
	m_rotation.eye();
	m_translation.zeros();
}

Marker::~Marker(void)
{
}

Mat Marker::rotate(const Mat& _input)
{
	// 把code顺时针旋转90°
	Mat output;
	_input.copyTo(output);
	for (int i=0; i<_input.rows; i++)
	{
		for (int j=0; j<_input.cols; j++)
		{
			output.at<uchar>(i,j) = _input.at<uchar>(_input.cols-j-1, i);
		}
	}
	return output;
}

int Marker::decode(Mat& _input, int& _numRotation)
{
	CV_Assert(_input.rows == _input.cols);
	CV_Assert(_input.type() == CV_8UC1);

	// 处理为2值图像grey
	Mat grey;
	threshold(_input, grey, 127, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	/*imshow("improvedMarker", grey);
   waitKey(0);*/
	// Marker是4*4的区域，周围边框是全白，中心4*4是有信息的marker
	int patchSize = _input.rows / 4;
	// 得到中心16区域的值
	Mat codemap_init = Mat::zeros(4, 4, CV_8UC1);
	Mat codemap = Mat::zeros(4, 4, CV_8UC1);
	for (int i=0; i<4; i++)
	{
		for (int j=0; j<4; j++)
		{
			int x = j * patchSize;
			int y = i * patchSize;
			Mat patch = grey(Rect(x, y, patchSize, patchSize));
			int numNoZero = countNonZero(patch);
			if (numNoZero > (patchSize*patchSize/2) )
			{
				codemap_init.at<uchar>(i,j) = 0;
			}
			else
				codemap_init.at<uchar>(i,j) = 1;
		}
	}
	//cout<<"mat:"<<endl<<codemap_init<<endl;
	uchar to_num=0;
	to_num=codemap_init.at<uchar>(0,0)+codemap_init.at<uchar>(3,0)+codemap_init.at<uchar>(0,3)+codemap_init.at<uchar>(3,3);
	if(to_num==1)
	{
		if(codemap_init.at<uchar>(0,0)==1)
		{
			_numRotation=0;
			codemap=codemap_init;
		}

		if(codemap_init.at<uchar>(3,0)==1)
		{
			_numRotation=1;
			codemap=rotate(codemap_init);
		}
		if(codemap_init.at<uchar>(3,3)==1)
		{
			_numRotation=2;
			codemap=rotate(codemap_init);
			codemap=rotate(codemap);
		}
		if(codemap_init.at<uchar>(0,3)==1)
		{
			_numRotation=3;
			codemap=rotate(codemap_init);
			codemap_init=rotate(codemap);
			codemap=rotate(codemap_init);
		}
		//cout<<"_numRotation:"<<endl<<_numRotation<<endl;
		//cout<<"codemap:"<<endl<<codemap<<endl;
		int E4=(codemap.at<uchar>(0,1)+codemap.at<uchar>(0,2)+codemap.at<uchar>(1,0)+codemap.at<uchar>(1,1)+codemap.at<uchar>(1,2)+1)%2;
		int E3=(codemap.at<uchar>(0,1)+codemap.at<uchar>(1,3)+codemap.at<uchar>(2,0)+codemap.at<uchar>(2,1)+codemap.at<uchar>(2,2)+1)%2;
		int E2=(codemap.at<uchar>(0,2)+codemap.at<uchar>(1,0)+codemap.at<uchar>(1,3)+codemap.at<uchar>(2,0)+codemap.at<uchar>(2,3)+codemap.at<uchar>(3,1)+1)%2;
		int E1=(codemap.at<uchar>(0,2)+codemap.at<uchar>(1,1)+codemap.at<uchar>(1,3)+codemap.at<uchar>(2,1)+codemap.at<uchar>(2,3)+codemap.at<uchar>(3,2)+1)%2;
		int E=E3*4+E2*2+E1;
		//cout<<"E:"<<endl<<E<<endl;
		int id;
		if (E==0)
		{
			id= codemap.at<uchar>(0,1)*128+codemap.at<uchar>(0,2)*64+codemap.at<uchar>(1,0)*32+codemap.at<uchar>(1,1)*16+codemap.at<uchar>(1,3)*8+
				codemap.at<uchar>(2,0)*4+codemap.at<uchar>(2,1)*2+codemap.at<uchar>(2,3)*1;
			//cout<<"id="<<id<<endl;
			return id;
		}
		return -1;


	}
	return -1;	// 和id不匹配，不是marker
}

float Marker::calPerimeter()
{
	float sum = 0.0f;
	for (size_t i=0; i<m_points.size(); i++)
	{
		size_t j = (i+1) % m_points.size();
		Point2f vec = m_points[i] - m_points[j];
		sum += sqrt(vec.dot(vec));
	}
	return sum;
}
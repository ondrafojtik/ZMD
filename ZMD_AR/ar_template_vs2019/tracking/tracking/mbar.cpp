#include "stdafx.h"

#define MARKER_BINS 7	// [-]
#define BIN_SIZE 8	// [px]
#define MARKER_SIZE ( MARKER_BINS * BIN_SIZE ) // [px]

const std::vector<cv::Point2f> MBAR::corners_ = { cv::Point( 0, 0 ),
	cv::Point( MARKER_SIZE - 1, 0 ), cv::Point( MARKER_SIZE - 1, MARKER_SIZE - 1 ),
	cv::Point( 0, MARKER_SIZE - 1 ) };

const std::vector<cv::Point3f> MBAR::corners_ws_ = { cv::Point3f( -3.0f, 3.0f, 0.0f ),
cv::Point3f( 3.0f, 3.0f, 0.0f ), cv::Point3f( 3.0f, -3.0f, 0.0f ), cv::Point3f( -3.0f, -3.0f, 0.0f ) };

const std::vector<cv::Point3f> MBAR::axis_ws_ = { cv::Point3f( 0.0f, 0.0f, 0.0f ),
cv::Point3f( 1.0f, 0.0f, 0.0f ), cv::Point3f( 0.0f, 1.0f, 0.0f ), cv::Point3f( 0.0f, 0.0f, 1.0f ) };

//http://www.mathaddict.net/hamming.htm
//http://homepages.bw.edu/~rmolmen/multimedia/hammingcode.swf
unsigned char encode_2bits( const unsigned char data2 )
{
	return 0;
}

inline unsigned char get_bit( const unsigned char byte, const char bit )
{
	assert( bit >= 0 && bit < 8 );

	return ( ( byte >> bit ) & 1 )/* == 1*/;
}

unsigned char decode_2bits( const unsigned char codeword5, bool & error )
{
	return 0;
}

MBAR::MBAR( const std::string & file_name )
{
	image_ = cv::imread( file_name );
}

void MBAR::start( const std::string & file_name )
{
	image_ = cv::imread( file_name );
	start();
}

template<typename type>
std::string to_binary( const type & value )
{
	std::bitset<sizeof( type ) * 8> bs( value );

	return bs.to_string();
}

void MBAR::start()
{	
	contours_.clear();
	markers_.clear();

	preprocess_image();
	find_contours();
	crop_markers();
	read_markers();
	refine_corners();
	solve_pnp();


}

void MBAR::preprocess_image()
{	
	cvtColor(image_, image_gray_, CV_BGRA2GRAY);
	
	cv::imshow("image_gray", image_gray_);
	//cv::waitKey();
}

void MBAR::find_contours()
{
	adaptiveThreshold(image_gray_, image_bw_, 255.0, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 7, 3.0);
	
	cv::imshow("image_bw", image_bw_);
	//cv::waitKey();


	std::vector<std::vector<cv::Point>> contours_candidates;
	findContours(image_bw_.clone(), contours_candidates, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	for (auto candidate : contours_candidates)
	{
		if (candidate.size() < 4) continue;

		std::vector<cv::Point2f> approximated_contour;
		approxPolyDP(candidate, approximated_contour, 0.02 * arcLength(candidate, true), true);

		double _contour_area = contourArea(approximated_contour);
		bool _is_contour_convex = isContourConvex(approximated_contour);
		
		if (_contour_area <= 2500) continue; // 50x50
		if (_is_contour_convex == false) continue;
		
		
		std::vector<cv::Point2f> contour_ccw;
		convexHull(approximated_contour, contour_ccw, false);

		if (contour_ccw.size() < 4) continue;

		contours_.push_back(contour_ccw);

	}

	// draw contours
	int contour_cnter = 0;
	for (auto contour : contours_)
	{
		image_result = image_.clone();

		for (auto point : contour)
		{
			float scale = 20;
			float hs = scale / 2.0f;

			cv::Rect rect(point.x - hs, point.y - hs, scale, scale);
			float color = contour_cnter * 50;
			cv::rectangle(image_result, rect, cv::Scalar(color, color, color), 1);
			contour_cnter += 1;
		}

		cv::imshow("contours", image_result);
		//cv::waitKey();
	}


}

void MBAR::crop_markers()
{
	for (auto contour : contours_)
	{
		std::vector<cv::Point2f> corners_ = { cv::Point(0, 0), cv::Point(MARKER_SIZE - 1, 0), cv::Point(MARKER_SIZE - 1, MARKER_SIZE - 1), cv::Point(0, MARKER_SIZE - 1) };
		cv::Mat is2ms = getPerspectiveTransform(contour, corners_);

		cv::Mat marker;
		warpPerspective(image_gray_, marker, is2ms, cv::Size(MARKER_SIZE, MARKER_SIZE));
		
		threshold(marker, marker, 127.0, 255.0, cv::THRESH_OTSU);

		markers_.push_back(marker);

		//cv::imshow("marker", marker);
		//cv::waitKey();

	}

	
}

void MBAR::read_markers()
{
	
	for (int i = 0; i < markers_.size(); i++)
	{
		cv::Mat marker;
		//cv::flip(markers_[i].image, marker, 1);

		resize(markers_[i].image, marker, cv::Size(MARKER_BINS, MARKER_BINS));
		threshold(marker, marker, 127.0, 255.0, cv::THRESH_BINARY);

		cv::Mat m_rotation = getRotationMatrix2D(cv::Point2f((MARKER_BINS - 1) * 0.5f, (MARKER_BINS - 1) * 0.5f), -90.0, 1.0); // (+) ccw rotation, (-) cw rotation

		cv::Mat ref_marker = cv::imread("../../data/marker.png", CV_LOAD_IMAGE_GRAYSCALE);
		resize(ref_marker, ref_marker, cv::Size(MARKER_BINS, MARKER_BINS));
		threshold(ref_marker, ref_marker, 127.0, 255.0, cv::THRESH_BINARY);

		int min_diff = 1000;
		for (;;)
		{
			warpAffine(marker, marker, m_rotation, marker.size());
			rotate(contours_[i].begin(), contours_[i].end() - 1, contours_[i].end());

			
			// load data
			std::vector<std::vector<int>> data_;
			for (int y = 0; y < 7; y++)
			{
				if (y == 0 || y == 6) continue;
				
				
				std::vector<int> MSB_LSB;
				MSB_LSB.push_back((int)marker.data[(y * 7) + 1] / 255);
				MSB_LSB.push_back((int)marker.data[(y * 7) + 2] / 255);
				MSB_LSB.push_back((int)marker.data[(y * 7) + 3] / 255);
				MSB_LSB.push_back((int)marker.data[(y * 7) + 4] / 255);
				MSB_LSB.push_back((int)marker.data[(y * 7) + 5] / 255);

				//std::cout << (int)marker.data[(y * 7) + 1] << std::endl; // MSB
				//std::cout << (int)marker.data[(y * 7) + 2] << std::endl;
				//std::cout << (int)marker.data[(y * 7) + 3] << std::endl;
				//std::cout << (int)marker.data[(y * 7) + 4] << std::endl;
				//std::cout << (int)marker.data[(y * 7) + 5] << std::endl; // LSB
				//std::cout << std::endl;

				std::vector<int> LSB_MSB;
				LSB_MSB.push_back((int)marker.data[(y * 7) + 5] / 255);
				LSB_MSB.push_back((int)marker.data[(y * 7) + 4] / 255);
				LSB_MSB.push_back((int)marker.data[(y * 7) + 3] / 255);
				LSB_MSB.push_back((int)marker.data[(y * 7) + 2] / 255);
				LSB_MSB.push_back((int)marker.data[(y * 7) + 1] / 255);

				data_.push_back(LSB_MSB);
			}
			
			// flip bits in first col
			for (int i_ = 0; i_ < 5; i_++)
				data_[i_][4] = abs(1 - data_[i_][4]);

			int img_diff = 0;
			for (int y = 0; y < 5; y++)
			{
				int P1 = data_[y][0];
				int P2 = data_[y][1];
				int D1 = data_[y][2];
				int P3 = data_[y][3];
				int D2 = data_[y][4];

				// expected
				
				int eP1 = D1 + D2;
				if (eP1 % 2 == 0) eP1 = 0;
				else eP1 = 1;

				int eP2 = D2;
				if (eP2 % 2 == 0) eP2 = 0;
				else eP2 = 1;

				int eP3 = D2;
				if (eP3 % 2 == 0) eP3 = 0;
				else eP3 = 1;

				std::vector<int> _line_reconst;
				_line_reconst.push_back(eP1);
				_line_reconst.push_back(eP2);
				_line_reconst.push_back(D1);
				_line_reconst.push_back(eP3);
				_line_reconst.push_back(D2);

				// compare lines
				std::vector<int> _line_original = data_[y];
				//std::vector<int> _line_original;

				//for (int h_ = 4; h_ >= 0; h_--)
				//	_line_original.push_back(data_[y][h_]);


				int diff = 0;
				for (int g_ = 0; g_ < 5; g_++)
					if (_line_original[g_] != _line_reconst[g_]) diff += 1;
				
				img_diff += diff;



				std::cout << "diff: " << diff << std::endl;

			}

			if (img_diff == min_diff) break;

			if (img_diff < min_diff) 
				min_diff = img_diff;



			// print
			for (int y = 0; y < 5; y++)
			{
				for (int x = 0; x < 5; x++)
					std::cout << data_[y][x];
				std::cout << std::endl;
			}

			cv::imshow("mark", marker);
			//cv::waitKey();

			int dist = 0;
			for (int y_ = 0; y_ < ref_marker.cols; y_++)
				for (int x_ = 0; x_ < ref_marker.rows; x_++)
				{
					int ref_val = (int)ref_marker.at<uchar>(x_, y_);
					int mrk_val = (int)marker.at<uchar>(x_, y_);

					dist += abs(ref_val - mrk_val);
				}

			//if (dist == 0) break;

		}

		markers_[i].image = marker;
		markers_[i].corners_is = contours_[i];

	}

	
	
}

void MBAR::refine_corners()
{
	for (int i = 0; i < markers_.size(); i++)
		cornerSubPix(image_gray_, markers_[i].corners_is, cv::Size(5, 5), cvSize(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER, 30, 0.1));
	
}


cv::Point2f convert_to_image_space(cv::Point3f p, cv::Mat m_M)
{
	cv::Mat point_ws_(4, 1, CV_64FC1);
	point_ws_.at<double>(0, 0) = p.x;
	point_ws_.at<double>(1, 0) = p.y;
	point_ws_.at<double>(2, 0) = p.z;
	point_ws_.at<double>(3, 0) = 1;

	cv::Mat point_is = m_M * point_ws_;
	point_is /= point_is.at<double>(2, 0);

	cv::Point2f res((float)point_is.at<double>(0, 0), (float)point_is.at<double>(1, 0));

	return res;
}



void MBAR::solve_pnp()
{
	cv::Mat m_camera = camera_matrix();
	cv::Mat distortion_coefficients = cv::Mat::zeros( 4, 1, CV_64FC1 );
	double & k1 = distortion_coefficients.at<double>( 0, 0 );
	double & k2 = distortion_coefficients.at<double>( 1, 0 );
	double & p1 = distortion_coefficients.at<double>( 2, 0 );
	double & p2 = distortion_coefficients.at<double>( 3, 0 );
	k1 = 0; // radial distortion
	k2 = 0;
	p1 = 0; // tangential distortion
	p2 = 0;

	for (auto marker : markers_)
	{
		cv::Mat rvec, tvec;
		solvePnP(corners_ws_, marker.corners_is, m_camera, distortion_coefficients, rvec, tvec);
		cv::Mat m_R(3, 3, CV_64FC1);
		Rodrigues(rvec, m_R);
		cv::Mat m_T;
		hconcat(m_R, tvec, m_T);
		cv::Mat m_M = m_camera * m_T;



		// draw
		std::vector<cv::Point> points_is;

		for (auto point : corners_ws_)
		{
			auto p = convert_to_image_space(point, m_M);
			
			float scale = 20.0f;
			float hs = scale / 2.0f;
			
			auto p1 = p;
			auto p2 = p;
			auto p3 = p;
			auto p4 = p;
			
			p1.x -= hs;
			p2.x += hs;

			p3.y -= hs;
			p4.y += hs;

			cv::line(image_result, p1, p2, cv::Scalar(255, 255, 0), 1);
			cv::line(image_result, p3, p4, cv::Scalar(255, 255, 0), 1);

			points_is.push_back(p);

		}

		fillConvexPoly(image_result, &points_is[0], points_is.size(), cv::Scalar(200, 100, 255), 1);
		
		{
			auto center = convert_to_image_space(axis_ws_[0], m_M);
			auto x =	  convert_to_image_space(axis_ws_[1]*3, m_M);
			auto y =	  convert_to_image_space(axis_ws_[2]*3, m_M);
			auto z =	  convert_to_image_space(axis_ws_[3]*3, m_M);


			arrowedLine(image_result, center, x, cv::Scalar(0, 0, 255), 2);
			arrowedLine(image_result, center, y, cv::Scalar(0, 255, 0), 2);
			arrowedLine(image_result, center, z, cv::Scalar(255, 0, 0), 2);

		}


		
		cv::Mat result = image_result.clone();
		float scale = 1.5f;
		resize(result, result, cv::Size(image_result.cols * scale, image_result.rows * scale));

		cv::imshow("image", result);
		cv::waitKey();


	}

	



}

cv::Mat MBAR::camera_matrix() const
{
	cv::Mat camera_matrix = cv::Mat::zeros( 3, 3, CV_64FC1 );
	double & fx = camera_matrix.ptr<double>( 0 )[0];
	double & fy = camera_matrix.ptr<double>( 1 )[1];
	double & cx = camera_matrix.ptr<double>( 0 )[2];
	double & cy = camera_matrix.ptr<double>( 1 )[2];	
	camera_matrix.ptr<double>( 2 )[2] = 1.0;
	const cv::Size size = image_.size();
	const double fov_y = DEG2RAD( 42.185 );
	fx = ( size.height * 0.5 ) / tan( fov_y * 0.5 );
	fy = fx;
	cx = size.width * 0.5;
	cy = size.height * 0.5;

	return camera_matrix;
}

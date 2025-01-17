// DIP.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <vector>

#define analog 0
#define bayer 0
#define hdr_exer 1
#define calibration 0


// Mat M = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

cv::Mat mat_rgb2yuv = (cv::Mat_<float>(3, 3) << 
	 0.299f,    0.587f,   0.114f,
	-0.14713f, -0.2886f,  0.436f,
	 0.615f,   -0.51499f,-0.10001f
	);

cv::Mat mat_yuv2rgb = (cv::Mat_<float>(3, 3) <<
	1.0f,  0.0f,      1.13983f,
	1.0f, -0.39465f, -0.58060f,
	1.0f,  2.03211f,  0.0f
	);

cv::Mat rgb2yuv(cv::Mat src)
{
	cv::Mat res = src.clone();

	for (int y_ = 0; y_ < res.rows; y_++)
		for (int x_ = 0; x_ < res.cols; x_++)
		{
			cv::Vec3f rgb = src.at<cv::Vec3f>(y_, x_);
			cv::Mat yuv = mat_rgb2yuv * cv::Mat(rgb);
			cv::Vec3f yuv_ = cv::Vec3f(yuv);
			res.at<cv::Vec3f>(y_, x_) = yuv_;
		}

	return res;
}

cv::Mat yuv2rgb(cv::Mat src)
{
	cv::Mat res = src.clone();

	for (int y_ = 0; y_ < res.rows; y_++)
		for (int x_ = 0; x_ < res.cols; x_++)
		{
			cv::Vec3f yuv = src.at<cv::Vec3f>(y_, x_);
			cv::Mat rgb = mat_yuv2rgb * cv::Mat(yuv);
			cv::Vec3f rgb_ = cv::Vec3f(rgb);
			res.at<cv::Vec3f>(y_, x_) = rgb_;
		}

	return res;
}

struct val_or_zero
{
	float value = 0;
	int outside = 0;
};

val_or_zero value_or_zero(cv::Mat src, int x, int y)
{
	int width = src.cols;
	int height = src.rows;

	if (x < 0) return {0, 0};
	if (x >= width) return {0, 0};

	if (y < 0) return {0, 0};
	if (y >= height) return {0, 0};

	uchar val = src.at<uchar>(y, x);
	return { (float)val, 1 };
}

cv::Mat Bayer(cv::Mat src)
{

	/*
	R	G	R	G	R
	G	B	G	B	G
	R	G	R	G	R
	G	B	G	B	G
	R	G	R	G	R
	*/

	cv::Mat res = cv::Mat(src.size(), CV_8UC3);

	for (int y_ = 0; y_ < res.rows; y_++)
		for (int x_ = 0; x_ < res.cols; x_++)
		{

			float pixelval = src.at<uchar>(y_, x_);
			float r = 0;
			float g = 0;
			float b = 0;


			if (y_ % 2 == 0)
			{
				if (x_ % 2 == 0) // R
				{
					r = pixelval;
					
					int gc = 0;
					g += value_or_zero(src, x_ + 1, y_).value;
					g += value_or_zero(src, x_ - 1, y_).value;
					g += value_or_zero(src, x_, y_ + 1).value;
					g += value_or_zero(src, x_, y_ - 1).value;
					
					gc += value_or_zero(src, x_ + 1, y_).outside;
					gc += value_or_zero(src, x_ - 1, y_).outside;
					gc += value_or_zero(src, x_, y_ + 1).outside;
					gc += value_or_zero(src, x_, y_ - 1).outside;

					g = g / gc;

					int bc = 0;
					b += value_or_zero(src, x_ + 1, y_ + 1).value;
					b += value_or_zero(src, x_ + 1, y_ - 1).value;
					b += value_or_zero(src, x_ - 1, y_ + 1).value;
					b += value_or_zero(src, x_ - 1, y_ - 1).value;
					
					bc += value_or_zero(src, x_ + 1, y_ + 1).outside;
					bc += value_or_zero(src, x_ + 1, y_ - 1).outside;
					bc += value_or_zero(src, x_ - 1, y_ + 1).outside;
					bc += value_or_zero(src, x_ - 1, y_ - 1).outside;

					b = b / bc;
				}
				else // G
				{
					r += value_or_zero(src, x_ + 1, y_).value;
					r += value_or_zero(src, x_ - 1, y_).value;
					int rc = 0;
					rc += value_or_zero(src, x_ + 1, y_).outside;
					rc += value_or_zero(src, x_ - 1, y_).outside;

					r = r / rc;

					g = pixelval;
					b = 0;

					b += value_or_zero(src, x_, y_ + 1).value;
					b += value_or_zero(src, x_, y_ - 1).value;

					int bc = 0;
					bc += value_or_zero(src, x_, y_ + 1).outside;
					bc += value_or_zero(src, x_, y_ - 1).outside;

					b = b / bc;

				}
				
			}
			else
			{
				if (x_ % 2 == 0) // G
				{
					r = 0;
					r += value_or_zero(src, x_, y_ + 1).value;
					r += value_or_zero(src, x_, y_ - 1).value;
					
					int rc = 0;
					rc += value_or_zero(src, x_, y_ + 1).outside;
					rc += value_or_zero(src, x_, y_ - 1).outside;
					
					r = r / rc;

					g = pixelval;
					b += value_or_zero(src, x_ + 1, y_).value;
					b += value_or_zero(src, x_ - 1, y_).value;
					int bc = 0;
					bc += value_or_zero(src, x_ + 1, y_).outside;
					bc += value_or_zero(src, x_ - 1, y_).outside;
					
					b = b / bc;
				}
				else // B
				{
					r += value_or_zero(src, x_ + 1, y_ + 1).value;
					r += value_or_zero(src, x_ + 1, y_ - 1).value;
					r += value_or_zero(src, x_ - 1, y_ + 1).value;
					r += value_or_zero(src, x_ - 1, y_ + 1).value;

					int rc = 0;
					rc += value_or_zero(src, x_ + 1, y_ + 1).outside;
					rc += value_or_zero(src, x_ + 1, y_ - 1).outside;
					rc += value_or_zero(src, x_ - 1, y_ + 1).outside;
					rc += value_or_zero(src, x_ - 1, y_ + 1).outside;

					r = r / rc;

					g += value_or_zero(src, x_ + 1, y_).value;
					g += value_or_zero(src, x_ - 1, y_).value;
					g += value_or_zero(src, x_, y_ + 1).value;
					g += value_or_zero(src, x_, y_ - 1).value;

					int gc = 0;
					gc += value_or_zero(src, x_ + 1, y_).outside;
					gc += value_or_zero(src, x_ - 1, y_).outside;
					gc += value_or_zero(src, x_, y_ + 1).outside;
					gc += value_or_zero(src, x_, y_ - 1).outside;

					g = g / gc;

					b = pixelval;
				}
			}

			cv::Vec3b rgb{ (uchar)b,(uchar)g,(uchar)r };

			res.at<cv::Vec3b>(y_, x_) = rgb;

		}


	return res;


}

struct HDR_result
{
	cv::Mat result_gray;
	cv::Mat result_rgb;
};

HDR_result HDR_gray(std::vector<cv::Mat> images, std::vector<cv::Mat> images_rgb)
{
	cv::Mat res = cv::Mat(images[0].size(), CV_8UC1);
	cv::Mat res_rgb = cv::Mat(images[0].size(), CV_32FC3);

	std::vector<cv::Mat> weight_map;
	for (int i = 0; i < images.size(); i++)
	{
		cv::Mat m = cv::Mat(images[i].size(), CV_32F);
		weight_map.push_back(m);
	}

	cv::Mat weight_map_sum = cv::Mat(res.size(), CV_32F);

	uchar blue = 255;

	float mu = 0.5f;
	float sigma = 0.2f;

	for (int y_ = 0; y_ < res.rows; y_++)
		for (int x_ = 0; x_ < res.cols; x_++)
		{
			
			// sum of images
			float final_pixel = 0;
			float wk_sum = 0;

			for (int i = 0; i < images.size(); i++)
			{
				float pixel_value = (float)images[i].at<uchar>(y_, x_);

				float upper = pow(pixel_value - (255.0f * mu), 2);
				float lower = 2 * pow(255 * sigma, 2);

				float wk = exp(-1 * upper / lower);
				//float wk = exp(-1 * pow((pixel_value - 255.0f * mu), 2) / (2 * pow(255 * sigma, 2)));
				//wk = wk / 2;
				//uchar this_image_pixel = wk * pixel_value;
				
				weight_map[i].at<float>(y_, x_) = wk;
				//final_pixel += this_image_pixel;
				wk_sum += wk;
			}
			weight_map_sum.at<float>(y_, x_) = wk_sum;

			// normalize
			for (int i = 0; i < images.size(); i++)
			{
				float value = weight_map[i].at<float>(y_, x_);
				float sum = weight_map_sum.at<float>(y_, x_);
				value = value / sum;
				weight_map[i].at<float>(y_, x_) = value;
			}
			
			
			//res.at<uchar>(y_, x_) = (uchar)final_pixel;
		}

	// assign values
	for (int y_ = 0; y_ < res.rows; y_++)
		for (int x_ = 0; x_ < res.cols; x_++)
		{
			float final_pixel = 0;
			cv::Vec3f final_pixel_rgb{ 0, 0, 0 };

			for (int i = 0; i < images.size(); i++)
			{
				float pixel_value = images[i].at<uchar>(y_, x_);
				float weight = weight_map[i].at<float>(y_, x_);
				final_pixel += pixel_value * weight;

				cv::Vec3f pixel_value_rgb = images_rgb[i].at<cv::Vec3f>(y_, x_);
				final_pixel_rgb += pixel_value_rgb * weight;
				
			}
			res.at<uchar>(y_, x_) = (uchar)final_pixel;
			res_rgb.at<cv::Vec3f>(y_, x_) = final_pixel_rgb;
		}

	HDR_result result{};
	result.result_gray = res;
	result.result_rgb = res_rgb;

	return result;
}

// calibration 
// https://github.com/opencv/opencv/blob/3.4/samples/cpp/tutorial_code/calib3d/camera_calibration/camera_calibration.cpp
// https://docs.opencv.org/3.4/d4/d94/tutorial_camera_calibration.html

int main()
{
	cv::Mat src_8uc3_img = cv::imread("images/lena.png", CV_LOAD_IMAGE_COLOR); // load color image from file system to Mat variable, this will be loaded using 8 bits (uchar)

	src_8uc3_img.convertTo(src_8uc3_img, CV_32FC3, 1.0 / 255);

	if (src_8uc3_img.empty()) {
		printf("Unable to read input file (%s, %d).", __FILE__, __LINE__);
	}

#if analog
	cv::Mat yuv = rgb2yuv(src_8uc3_img);
	cv::Mat rgb = yuv2rgb(yuv);


	cv::imshow("original", src_8uc3_img);
	cv::imshow("yuv", yuv);
	cv::imshow("converted", rgb);
	

	cv::waitKey(0); // wait until keypressed
#endif

# if bayer
	cv::Mat bayer_ = cv::imread("images/bayer.bmp", CV_LOAD_IMAGE_GRAYSCALE); // load color image from file system to Mat variable, this will be loaded using 8 bits (uchar)

	cv::Mat bayer_res = Bayer(bayer_);

	cv::imshow("bayer original", bayer_);
	cv::imshow("bayer", bayer_res);

	cv::waitKey(0);
#endif

#if hdr_exer

	std::vector<cv::Mat> images_;
	std::vector<cv::Mat> images_gray;
	cv::Mat s0 = cv::imread("images/s1_0.png", CV_LOAD_IMAGE_COLOR);
	s0.convertTo(s0, CV_32FC3, 1.0 / 255);
	images_.push_back(s0);
	s0  = cv::imread("images/s1_0.png", CV_LOAD_IMAGE_GRAYSCALE); // load color image 
	images_gray.push_back(s0);

	cv::Mat s1 = cv::imread("images/s1_1.png", CV_LOAD_IMAGE_COLOR);
	s1.convertTo(s1, CV_32FC3, 1.0 / 255);
	images_.push_back(s1);
	s1 = cv::imread("images/s1_1.png", CV_LOAD_IMAGE_GRAYSCALE); // load color image 
	images_gray.push_back(s1);

	cv::Mat s2 = cv::imread("images/s1_2.png", CV_LOAD_IMAGE_COLOR);
	s2.convertTo(s2, CV_32FC3, 1.0 / 255);
	images_.push_back(s2);
	s2 = cv::imread("images/s1_2.png", CV_LOAD_IMAGE_GRAYSCALE); // load color image 
	images_gray.push_back(s2);

	cv::Mat s3 = cv::imread("images/s1_3.png", CV_LOAD_IMAGE_COLOR);
	s3.convertTo(s3, CV_32FC3, 1.0 / 255);
	images_.push_back(s3);
	s3 = cv::imread("images/s1_3.png", CV_LOAD_IMAGE_GRAYSCALE); // load color image 
	images_gray.push_back(s3);

	cv::Mat s4 = cv::imread("images/s1_4.png", CV_LOAD_IMAGE_COLOR);
	s4.convertTo(s4, CV_32FC3, 1.0 / 255);
	images_.push_back(s4);
	s4 = cv::imread("images/s1_4.png", CV_LOAD_IMAGE_GRAYSCALE); // load color image 
	images_gray.push_back(s4);

	//cv::Mat hrd_res = HDR(images_);
	HDR_result hdr_res = HDR_gray(images_gray, images_);

	cv::imshow("hdr_rgb", hdr_res.result_rgb);
	cv::imshow("hdr_gray", hdr_res.result_gray);
	cv::waitKey(0);
#endif

	// calibration
	//std::cout << calibration_() << std::endl;

	/*

	cv::Mat gray_8uc1_img; // declare variable to hold grayscale version of img variable, gray levels wil be represented using 8 bits (uchar)
	cv::Mat gray_32fc1_img; // declare variable to hold grayscale version of img variable, gray levels wil be represented using 32 bits (float)

	cv::cvtColor(src_8uc3_img, gray_8uc1_img, CV_BGR2GRAY); // convert input color image to grayscale one, CV_BGR2GRAY specifies direction of conversion
	gray_8uc1_img.convertTo(gray_32fc1_img, CV_32FC1, 1.0 / 255.0); // convert grayscale image from 8 bits to 32 bits, resulting values will be in the interval 0.0 - 1.0

	int x = 10, y = 15; // pixel coordinates

	uchar p1 = gray_8uc1_img.at<uchar>(y, x); // read grayscale value of a pixel, image represented using 8 bits
	float p2 = gray_32fc1_img.at<float>(y, x); // read grayscale value of a pixel, image represented using 32 bits
	cv::Vec3b p3 = src_8uc3_img.at<cv::Vec3b>(y, x); // read color value of a pixel, image represented using 8 bits per color channel

	// print values of pixels
	printf("p1 = %d\n", p1);
	printf("p2 = %f\n", p2);
	printf("p3[ 0 ] = %d, p3[ 1 ] = %d, p3[ 2 ] = %d\n", p3[0], p3[1], p3[2]);

	gray_8uc1_img.at<uchar>(y, x) = 0; // set pixel value to 0 (black)

	// draw a rectangle
	cv::rectangle(gray_8uc1_img, cv::Point(65, 84), cv::Point(75, 94),
		cv::Scalar(50), CV_FILLED);

	// declare variable to hold gradient image with dimensions: width= 256 pixels, height= 50 pixels.
	// Gray levels wil be represented using 8 bits (uchar)
	cv::Mat gradient_8uc1_img(50, 256, CV_8UC1);

	// For every pixel in image, assign a brightness value according to the x coordinate.
	// This wil create a horizontal gradient.
	for (int y = 0; y < gradient_8uc1_img.rows; y++) {
		for (int x = 0; x < gradient_8uc1_img.cols; x++) {
			gradient_8uc1_img.at<uchar>(y, x) = x;
		}
	}

	// diplay images
	cv::imshow("Gradient", gradient_8uc1_img);
	cv::imshow("Lena gray", gray_8uc1_img);
	cv::imshow("Lena gray 32f", gray_32fc1_img);

	*/

	
	return 0;
}

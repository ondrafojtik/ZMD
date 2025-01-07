#ifndef MBAR_H_
#define MBAR_H_

//enum class Rotations : char { NA = -1, R0, R90, R180, R270 };

struct Marker
{
	cv::Mat image;
	//char rotation;
	int value;
	std::vector<cv::Point2f> corners_is;

	Marker( const cv::Mat & image )
	{
		this->image = image;
		//rotation = -1;
		value = -1;
	}
};

class MBAR
{
public:
	MBAR( const std::string & file_name );
	
	void start( const std::string & file_name );
	void start();

private:	
	void preprocess_image();
	void find_contours();
	void crop_markers();
	void read_markers();	
	void refine_corners();
	void solve_pnp();

	cv::Mat camera_matrix() const;

	cv::Mat image_;
	cv::Mat image_gray_;
	cv::Mat image_bw_;
	std::vector<std::vector<cv::Point2f>> contours_;
	std::vector<Marker> markers_;	

	cv::Mat image_result;

	static const std::vector<cv::Point2f> corners_; // corners in local marker space
	static const std::vector<cv::Point3f> corners_ws_; // corners in ws
	static const std::vector<cv::Point3f> axis_ws_; // corners in ws
};

#endif

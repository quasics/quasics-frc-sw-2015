#include "MultipleOutputs.h"

namespace grip {

MultipleOutputs::MultipleOutputs() {
}
/**
* Runs an iteration of the pipeline and updates outputs.
*/
void MultipleOutputs::Process(cv::Mat& source0){
	//Step CV_resize0:
	//input
	cv::Mat cvResizeSrc = source0;
	cv::Size cvResizeDsize(0, 0);
	double cvResizeFx = 0.7;  // default Double
	double cvResizeFy = 0.7;  // default Double
    int cvResizeInterpolation = cv::INTER_LINEAR;
	cvResize(cvResizeSrc, cvResizeDsize, cvResizeFx, cvResizeFy, cvResizeInterpolation, this->cvResizeOutput);
	//Step HSV_Threshold0:
	//input
	cv::Mat hsvThreshold0Input = cvResizeOutput;
	double hsvThreshold0Hue[] = {0.0, 180.0};
	double hsvThreshold0Saturation[] = {105.64971751412429, 236.91489361702128};
	double hsvThreshold0Value[] = {84.03954802259886, 216.56914893617022};
	hsvThreshold(hsvThreshold0Input, hsvThreshold0Hue, hsvThreshold0Saturation, hsvThreshold0Value, this->hsvThreshold0Output);
	//Step CV_erode0:
	//input
	cv::Mat cvErode0Src = hsvThreshold0Output;
	cv::Mat cvErode0Kernel;
	cv::Point cvErode0Anchor(-1, -1);
	double cvErode0Iterations = 1.0;  // default Double
    int cvErode0Bordertype = cv::BORDER_CONSTANT;
	cv::Scalar cvErode0Bordervalue(-1);
	cvErode(cvErode0Src, cvErode0Kernel, cvErode0Anchor, cvErode0Iterations, cvErode0Bordertype, cvErode0Bordervalue, this->cvErode0Output);
	//Step Find_Contours0:
	//input
	cv::Mat findContours0Input = cvErode0Output;
	bool findContours0ExternalOnly = false;  // default Boolean
	findContours(findContours0Input, findContours0ExternalOnly, this->findContours0Output);
	//Step Filter_Contours0:
	//input
	std::vector<std::vector<cv::Point> > filterContours0Contours = findContours0Output;
	double filterContours0MinArea = 9.0;  // default Double
	double filterContours0MinPerimeter = 0.0;  // default Double
	double filterContours0MinWidth = 0.0;  // default Double
	double filterContours0MaxWidth = 1000.0;  // default Double
	double filterContours0MinHeight = 20.0;  // default Double
	double filterContours0MaxHeight = 1000.0;  // default Double
	double filterContours0Solidity[] = {0, 100};
	double filterContours0MaxVertices = 1000000.0;  // default Double
	double filterContours0MinVertices = 36.0;  // default Double
	double filterContours0MinRatio = 0.0;  // default Double
	double filterContours0MaxRatio = 1000.0;  // default Double
	filterContours(filterContours0Contours, filterContours0MinArea, filterContours0MinPerimeter, filterContours0MinWidth, filterContours0MaxWidth, filterContours0MinHeight, filterContours0MaxHeight, filterContours0Solidity, filterContours0MaxVertices, filterContours0MinVertices, filterContours0MinRatio, filterContours0MaxRatio, this->filterContours0Output);
	//Step HSV_Threshold1:
	//input
	cv::Mat hsvThreshold1Input = cvResizeOutput;
	double hsvThreshold1Hue[] = {17.729545177417386, 58.88679108271004};
	double hsvThreshold1Saturation[] = {130.71043165467623, 255.0};
	double hsvThreshold1Value[] = {66.50179856115108, 255.0};
	hsvThreshold(hsvThreshold1Input, hsvThreshold1Hue, hsvThreshold1Saturation, hsvThreshold1Value, this->hsvThreshold1Output);
	//Step CV_erode1:
	//input
	cv::Mat cvErode1Src = hsvThreshold1Output;
	cv::Mat cvErode1Kernel;
	cv::Point cvErode1Anchor(-1, -1);
	double cvErode1Iterations = 1.0;  // default Double
    int cvErode1Bordertype = cv::BORDER_CONSTANT;
	cv::Scalar cvErode1Bordervalue(-1);
	cvErode(cvErode1Src, cvErode1Kernel, cvErode1Anchor, cvErode1Iterations, cvErode1Bordertype, cvErode1Bordervalue, this->cvErode1Output);
	//Step Find_Contours1:
	//input
	cv::Mat findContours1Input = cvErode1Output;
	bool findContours1ExternalOnly = false;  // default Boolean
	findContours(findContours1Input, findContours1ExternalOnly, this->findContours1Output);
	//Step Filter_Contours1:
	//input
	std::vector<std::vector<cv::Point> > filterContours1Contours = findContours1Output;
	double filterContours1MinArea = 0.0;  // default Double
	double filterContours1MinPerimeter = 0.0;  // default Double
	double filterContours1MinWidth = 10.0;  // default Double
	double filterContours1MaxWidth = 1000.0;  // default Double
	double filterContours1MinHeight = 10.0;  // default Double
	double filterContours1MaxHeight = 1000.0;  // default Double
	double filterContours1Solidity[] = {0.0, 100};
	double filterContours1MaxVertices = 100000.0;  // default Double
	double filterContours1MinVertices = 0.0;  // default Double
	double filterContours1MinRatio = 0.0;  // default Double
	double filterContours1MaxRatio = 1000.0;  // default Double
	filterContours(filterContours1Contours, filterContours1MinArea, filterContours1MinPerimeter, filterContours1MinWidth, filterContours1MaxWidth, filterContours1MinHeight, filterContours1MaxHeight, filterContours1Solidity, filterContours1MaxVertices, filterContours1MinVertices, filterContours1MinRatio, filterContours1MaxRatio, this->filterContours1Output);
}

/**
 * This method is a generated getter for the output of a CV_resize.
 * @return Mat output from CV_resize.
 */
cv::Mat* MultipleOutputs::GetCvResizeOutput(){
	return &(this->cvResizeOutput);
}
/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* MultipleOutputs::GetHsvThreshold0Output(){
	return &(this->hsvThreshold0Output);
}
/**
 * This method is a generated getter for the output of a CV_erode.
 * @return Mat output from CV_erode.
 */
cv::Mat* MultipleOutputs::GetCvErode0Output(){
	return &(this->cvErode0Output);
}
/**
 * This method is a generated getter for the output of a Find_Contours.
 * @return ContoursReport output from Find_Contours.
 */
std::vector<std::vector<cv::Point> >* MultipleOutputs::GetFindContours0Output(){
	return &(this->findContours0Output);
}
/**
 * This method is a generated getter for the output of a Filter_Contours.
 * @return ContoursReport output from Filter_Contours.
 */
std::vector<std::vector<cv::Point> >* MultipleOutputs::GetFilterContours0Output(){
	return &(this->filterContours0Output);
}
/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* MultipleOutputs::GetHsvThreshold1Output(){
	return &(this->hsvThreshold1Output);
}
/**
 * This method is a generated getter for the output of a CV_erode.
 * @return Mat output from CV_erode.
 */
cv::Mat* MultipleOutputs::GetCvErode1Output(){
	return &(this->cvErode1Output);
}
/**
 * This method is a generated getter for the output of a Find_Contours.
 * @return ContoursReport output from Find_Contours.
 */
std::vector<std::vector<cv::Point> >* MultipleOutputs::GetFindContours1Output(){
	return &(this->findContours1Output);
}
/**
 * This method is a generated getter for the output of a Filter_Contours.
 * @return ContoursReport output from Filter_Contours.
 */
std::vector<std::vector<cv::Point> >* MultipleOutputs::GetFilterContours1Output(){
	return &(this->filterContours1Output);
}
	/**
	 * Resizes an Image.
	 * @param src The image to resize.
	 * @param dSize size to set the image.
	 * @param fx scale factor along X axis.
	 * @param fy scale factor along Y axis.
	 * @param interpolation type of interpolation to use.
	 * @param dst output image.
	 */
	void MultipleOutputs::cvResize(cv::Mat &src, cv::Size &dSize, double fx, double fy, int interpolation, cv::Mat &dst) {
		cv::resize(src, dst, dSize, fx, fy, interpolation);
	}

	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue.
	 * @param sat The min and max saturation.
	 * @param val The min and max value.
	 * @param output The image in which to store the output.
	 */
	void MultipleOutputs::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
		cv::cvtColor(input, out, cv::COLOR_BGR2HSV);
		cv::inRange(out,cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Expands area of lower value in an image.
	 * @param src the Image to erode.
	 * @param kernel the kernel for erosion.
	 * @param anchor the center of the kernel.
	 * @param iterations the number of times to perform the erosion.
	 * @param borderType pixel extrapolation method.
	 * @param borderValue value to be used for a constant border.
	 * @param dst Output Image.
	 */
	void MultipleOutputs::cvErode(cv::Mat &src, cv::Mat &kernel, cv::Point &anchor, double iterations, int borderType, cv::Scalar &borderValue, cv::Mat &dst) {
		cv::erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}

	/**
	 * Finds contours in an image.
	 *
	 * @param input The image to find contours in.
	 * @param externalOnly if only external contours are to be found.
	 * @param contours vector of contours to put contours in.
	 */
	void MultipleOutputs::findContours(cv::Mat &input, bool externalOnly, std::vector<std::vector<cv::Point> > &contours) {
		std::vector<cv::Vec4i> hierarchy;
		contours.clear();
		int mode = externalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
		int method = cv::CHAIN_APPROX_SIMPLE;
		cv::findContours(input, contours, hierarchy, mode, method);
	}


	/**
	 * Filters through contours.
	 * @param inputContours is the input vector of contours.
	 * @param minArea is the minimum area of a contour that will be kept.
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept.
	 * @param minWidth minimum width of a contour.
	 * @param maxWidth maximum width.
	 * @param minHeight minimum height.
	 * @param maxHeight  maximimum height.
	 * @param solidity the minimum and maximum solidity of a contour.
	 * @param minVertexCount minimum vertex Count of the contours.
	 * @param maxVertexCount maximum vertex Count.
	 * @param minRatio minimum ratio of width to height.
	 * @param maxRatio maximum ratio of width to height.
	 * @param output vector of filtered contours.
	 */
	void MultipleOutputs::filterContours(std::vector<std::vector<cv::Point> > &inputContours, double minArea, double minPerimeter, double minWidth, double maxWidth, double minHeight, double maxHeight, double solidity[], double maxVertexCount, double minVertexCount, double minRatio, double maxRatio, std::vector<std::vector<cv::Point> > &output) {
		std::vector<cv::Point> hull;
		output.clear();
		for (std::vector<cv::Point> contour: inputContours) {
			cv::Rect bb = boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth) continue;
			if (bb.height < minHeight || bb.height > maxHeight) continue;
			double area = cv::contourArea(contour);
			if (area < minArea) continue;
			if (arcLength(contour, true) < minPerimeter) continue;
			cv::convexHull(cv::Mat(contour, true), hull);
			double solid = 100 * area / cv::contourArea(hull);
			if (solid < solidity[0] || solid > solidity[1]) continue;
			if (contour.size() < minVertexCount || contour.size() > maxVertexCount)	continue;
			double ratio = (double) bb.width / (double) bb.height;
			if (ratio < minRatio || ratio > maxRatio) continue;
			output.push_back(contour);
		}
	}



} // end grip namespace


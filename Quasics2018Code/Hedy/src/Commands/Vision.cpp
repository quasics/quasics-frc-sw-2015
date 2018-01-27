#include "Vision.h"

namespace grip {

Vision::Vision() {
}
/**
* Runs an iteration of the pipeline and updates outputs.
*/
void Vision::Process(cv::Mat& source0){
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
	cv::Mat hsvThresholdInput = cvResizeOutput;
	double hsvThresholdHue[] = {19.424460431654676, 93.99317406143345};
	double hsvThresholdSaturation[] = {130.71043165467623, 202.78156996587032};
	double hsvThresholdValue[] = {66.50179856115108, 255.0};
	hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, this->hsvThresholdOutput);
	//Step CV_erode0:
	//input
	cv::Mat cvErodeSrc = hsvThresholdOutput;
	cv::Mat cvErodeKernel;
	cv::Point cvErodeAnchor(-1, -1);
	double cvErodeIterations = 1;  // default Double
    int cvErodeBordertype = cv::BORDER_CONSTANT;
	cv::Scalar cvErodeBordervalue(-1);
	cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, this->cvErodeOutput);
	//Step Mask0:
	//input
	cv::Mat maskInput = cvResizeOutput;
	cv::Mat maskMask = cvErodeOutput;
	mask(maskInput, maskMask, this->maskOutput);
	//Step Find_Contours0:
	//input
	cv::Mat findContoursInput = cvErodeOutput;
	bool findContoursExternalOnly = false;  // default Boolean
	findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);
}

/**
 * This method is a generated getter for the output of a CV_resize.
 * @return Mat output from CV_resize.
 */
cv::Mat* Vision::GetCvResizeOutput(){
	return &(this->cvResizeOutput);
}
/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* Vision::GetHsvThresholdOutput(){
	return &(this->hsvThresholdOutput);
}
/**
 * This method is a generated getter for the output of a CV_erode.
 * @return Mat output from CV_erode.
 */
cv::Mat* Vision::GetCvErodeOutput(){
	return &(this->cvErodeOutput);
}
/**
 * This method is a generated getter for the output of a Mask.
 * @return Mat output from Mask.
 */
cv::Mat* Vision::GetMaskOutput(){
	return &(this->maskOutput);
}
/**
 * This method is a generated getter for the output of a Find_Contours.
 * @return ContoursReport output from Find_Contours.
 */
std::vector<std::vector<cv::Point> >* Vision::GetFindContoursOutput(){
	return &(this->findContoursOutput);
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
	void Vision::cvResize(cv::Mat &src, cv::Size &dSize, double fx, double fy, int interpolation, cv::Mat &dst) {
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
	void Vision::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
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
	void Vision::cvErode(cv::Mat &src, cv::Mat &kernel, cv::Point &anchor, double iterations, int borderType, cv::Scalar &borderValue, cv::Mat &dst) {
		cv::erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}

		/**
		 * Filter out an area of an image using a binary mask.
		 *
		 * @param input The image on which the mask filters.
		 * @param mask The binary image that is used to filter.
		 * @param output The image in which to store the output.
		 */
		void Vision::mask(cv::Mat &input, cv::Mat &mask, cv::Mat &output) {
			mask.convertTo(mask, CV_8UC1);
			cv::bitwise_xor(output, output, output);
			input.copyTo(output, mask);
		}

	/**
	 * Finds contours in an image.
	 *
	 * @param input The image to find contours in.
	 * @param externalOnly if only external contours are to be found.
	 * @param contours vector of contours to put contours in.
	 */
	void Vision::findContours(cv::Mat &input, bool externalOnly, std::vector<std::vector<cv::Point> > &contours) {
		std::vector<cv::Vec4i> hierarchy;
		contours.clear();
		int mode = externalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
		int method = cv::CHAIN_APPROX_SIMPLE;
		cv::findContours(input, contours, hierarchy, mode, method);
	}



} // end grip namespace


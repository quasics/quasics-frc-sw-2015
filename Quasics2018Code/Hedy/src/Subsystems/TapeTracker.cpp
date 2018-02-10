#include "TapeTracker.h"
#include <iostream>

#define IMG_WIDTH 320
#define IMG_HEIGHT 240

#define WIDTH_SCALING
#define HEIGHT_SCALING
TapeTracker::TapeTracker() : frc::Subsystem("TapeTracker") {
	cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
	camera.SetResolution(IMG_WIDTH, IMG_HEIGHT);
	m_lock = new std::mutex;
}

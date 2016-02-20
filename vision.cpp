#include <stdio.h>
#include <iostream>
#include <libfreenect.hpp>
#include <libfreenect.h>
//#include <libfreenect_registration.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>
#include <ctime>
//#include <thread.hpp>


//OPENCV Includes
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std; 

class Mtx {
public:
    Mtx() {
      pthread_mutex_init( &m_mutex, NULL );
    }
  void lock() {
    pthread_mutex_lock( &m_mutex );
  }
  void unlock() {
    pthread_mutex_unlock( &m_mutex );
  }
  
private:
  pthread_mutex_t m_mutex;
};

  
  ///Kinect Hardware Connection Class
  /* thanks to Yoda---- from IRC */
  class MyFreenectDevice : public Freenect::FreenectDevice {
  public:
    MyFreenectDevice(freenect_context *_ctx, int _index)
      : Freenect::FreenectDevice(_ctx, _index), 
	depth(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED).bytes),
	m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes), 
	m_new_rgb_frame(false), 
	m_new_depth_frame(false)
    {
      
    }
    //~MyFreenectDevice(){}
    // Do not call directly even in child
    void VideoCallback(void* _rgb, uint32_t timestamp) {
      m_rgb_mutex.lock();
      uint8_t* rgb = static_cast<uint8_t*>(_rgb);
      std::copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
      m_new_rgb_frame = true;
      m_rgb_mutex.unlock();
    };
    // Do not call directly even in child
    void DepthCallback(void* _depth, uint32_t timestamp) {
      m_depth_mutex.lock();
      depth.clear();
      uint16_t* call_depth = static_cast<uint16_t*>(_depth);
      for (size_t i = 0; i < 640*480 ; i++) {
	depth.push_back(call_depth[i]);
      }
      m_new_depth_frame = true;
      m_depth_mutex.unlock();
    }
    bool getRGB(std::vector<uint8_t> &buffer) {
      //printf("Getting RGB!\n");
      m_rgb_mutex.lock();
      if (!m_new_rgb_frame) {
	//printf("No new RGB Frame.\n");
	return false;
		}
      buffer.swap(m_buffer_video);
      m_new_rgb_frame = false;
      m_rgb_mutex.unlock();
      return true;
    }
    
    bool getDepth(std::vector<uint16_t> &buffer) {
      m_rgb_mutex.lock();
      if (!m_new_depth_frame)
	return false;
      buffer.swap(depth);
      m_new_depth_frame = false;
      m_rgb_mutex.unlock();
      return true;
    }
    
  private:
    std::vector<uint16_t> depth;
    std::vector<uint8_t> m_buffer_video;
    Mtx m_rgb_mutex;
    Mtx m_depth_mutex;
    bool m_new_rgb_frame;
    bool m_new_depth_frame;
  };
  
  
  ///Start the PCL/OK Bridging
  
  //OK
  Freenect::Freenect freenect;
  MyFreenectDevice* device;
  freenect_video_format requested_format(FREENECT_VIDEO_RGB);
  double freenect_angle(0);
  int got_frames(0),window(0);
  int g_argc;
  char **g_argv;
  int user_data = 0;
  
  
  //OpenCV
  cv::Mat mCorners;
  cv::Mat mOut;
  cv::Mat mGray;
  cv::Size boardSize(10,7); //interior number of corners
  cv::Size imageSize;
  float squareSize = 0.023; //23 mm
  cv::Mat cameraMatrix, distCoeffs;
  vector<vector<cv::Point2f> > imagePoints;
  vector<cv::Point2f> pointbuf;
  float aspectRatio = 1.0f;
  vector<cv::Mat> rvecs, tvecs;
  vector<float> reprojErrs;
  cv::Mat map1, map2;
  cv::Mat mCalib;

int main (int argc, char** argv)
{
  cout << "Hello Vision" << endl;
  return 0;
}



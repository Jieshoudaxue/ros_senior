#include "ar_track_alvar/CvTestbed.h"

CvTestbed::CvTestbed()
{
  cap = NULL;
  running = false;
  videocallback = NULL;
  keycallback = NULL;
  images.clear();
}

CvTestbed::~CvTestbed()
{
  for (size_t i = 0; i < images.size(); i++)
  {
    if (images[i].release_at_exit)
    {
      images[i].ipl.release();
    }
  }
  images.clear();
}

void CvTestbed::default_videocallback(cv::Mat& image)
{
  // TODO: Skip frames if we are too slow? Impossible using OpenCV?
  /*
  static bool semaphore=false;
  if (semaphore) return;
  semaphore = true;
  */

  if (CvTestbed::Instance().videocallback)
  {
    CvTestbed::Instance().videocallback(image);
  }
  CvTestbed::Instance().ShowVisibleImages();

  // semaphore = false;
}

void CvTestbed::WaitKeys()
{
  running = true;
  static bool pause = false;
  while (running)
  {
    if (cap)
    {
      cv::Mat& frame = cap->captureImage();
      if (!frame.empty())
      {
        default_videocallback(frame);
        if (wintitle.size() > 0)
        {
          cv::imshow(wintitle, frame);
        }
      }
    }
    int key;
#ifdef WIN32
    if ((key = cvWaitKey(1)) >= 0)
    {
#else
    if ((key = cv::waitKey(20)) >= 0)
    {
#endif
      if (keycallback)
      {
        key = keycallback(key);
      }
      // If 'keycallback' didn't handle key - Use default handling
      // By default 'C' for calibration
      if (key == 'C')
      {
        if (cap)
        {
          cap->showSettingsDialog();
        }
      }
      // By default '0'-'9' toggles visible images
      else if ((key >= '0') && (key <= '9'))
      {
        int index = key - '0';
        ToggleImageVisible(index);
      }
      else if (key == 'p')
      {
        pause = !pause;
      }
      else if (key > 0)
      {
        running = false;
      }
    }
  }
}

void CvTestbed::ShowVisibleImages()
{
  for (size_t i = 0; i < images.size(); i++)
  {
    if (images[i].visible)
    {
      cv::imshow(images[i].title, images[i].ipl);
    }
  }
}

CvTestbed& CvTestbed::Instance()
{
  static CvTestbed obj;
  return obj;
}

void CvTestbed::SetVideoCallback(void (*_videocallback)(cv::Mat& image))
{
  videocallback = _videocallback;
}

void CvTestbed::SetKeyCallback(int (*_keycallback)(int key))
{
  keycallback = _keycallback;
}

bool CvTestbed::StartVideo(Capture* _cap, const char* _wintitle)
{
  bool clean = false;
  cap = _cap;
  if (cap == NULL)
  {
    CaptureFactory::CaptureDeviceVector vec =
        CaptureFactory::instance()->enumerateDevices();
    if (vec.size() < 1)
      return false;
    cap = CaptureFactory::instance()->createCapture(vec[0]);
    if (!cap->start())
    {
      delete cap;
      return false;
    }
    clean = true;
  }
  if (_wintitle)
  {
    wintitle = _wintitle;
    cv::namedWindow(_wintitle, 1);
  }
  WaitKeys();  // Call the main loop
  if (clean)
  {
    cap->stop();
    delete cap;
  }
  return true;
}

size_t CvTestbed::SetImage(const char* title, const cv::Mat& ipl,
                           bool release_at_exit /* =false */)
{
  size_t index = GetImageIndex(title);
  if (index == -1)
  {
    // If the title wasn't found create new
    Image i(ipl, title, false, release_at_exit);
    images.push_back(i);
    return (images.size() - 1);
  }
  // If the title was found replace the image
  if (images[index].release_at_exit)
  {
    images[index].ipl.release();
  }
  images[index].ipl = ipl;
  images[index].release_at_exit = release_at_exit;
  return index;
}

cv::Mat CvTestbed::CreateImage(const char* title, cv::Size size, int depth,
                               int channels)
{
  cv::Mat ipl = cv::Mat(size, CV_MAKETYPE(depth, channels));
  SetImage(title, ipl, true);
  return ipl;
}

cv::Mat CvTestbed::CreateImageWithProto(const char* title, cv::Mat& proto,
                                        int depth /* =0 */,
                                        int channels /* =0 */)
{
  if (depth == 0)
    depth = proto.depth();
  if (channels == 0)
    channels = proto.channels();
  cv::Mat ipl =
      cv::Mat(cv::Size(proto.cols, proto.rows), CV_MAKETYPE(depth, channels));
  SetImage(title, ipl, true);
  return ipl;
}

cv::Mat CvTestbed::GetImage(size_t index)
{
  if (index < 0)
    return cv::Mat();
  if (index >= images.size())
    return cv::Mat();
  return images[index].ipl;
}

size_t CvTestbed::GetImageIndex(const char* title)
{
  std::string s(title);
  for (size_t i = 0; i < images.size(); i++)
  {
    if (s.compare(images[i].title) == 0)
    {
      return i;
    }
  }
  return (size_t)-1;
}

cv::Mat CvTestbed::GetImage(const char* title)
{
  return GetImage(GetImageIndex(title));
}

bool CvTestbed::ToggleImageVisible(size_t index, int flags)
{
  if (index >= images.size())
    return false;
  if (images[index].visible == false)
  {
    images[index].visible = true;
    cv::namedWindow(images[index].title, flags);
    return true;
  }
  else
  {
    images[index].visible = false;
    cv::destroyWindow(images[index].title);
    return false;
  }
}

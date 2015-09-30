/*!
  \file        eyes_builder.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/9/30

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

Dynamically create and move eyes.
 */
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>

int main(int, char**) {
  cv::VideoCapture cap(0); // open the default camera
  //int w = 640, h = 480;
  int w = 1600, h = 1200;
  printf("w:%i, h:%i\n", w, h);
  assert(cap.isOpened());  // check if we succeeded
  cap.set(CV_CAP_PROP_FRAME_WIDTH, w);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, h);

  cv::namedWindow("frame",1);
  for(;;) {
      cv::Mat frame;
      cap >> frame; // get a new frame from camera
      cv::imshow("frame", frame);
      if(cv::waitKey(30) >= 0) break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}

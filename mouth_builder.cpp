/*!
  \file
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/10/25

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

Dynamically create and move mouth.
 */
#include "mouth_builder.h"
#include <opencv2/highgui/highgui.hpp>
int main(int, char**) {
  std::string default_path = "data/mini_mouth";
  MouthBuilder b;
  while(true) {
    for (int i = 0; i < b.nleds(); ++i)
      b.set_led_state(i, (rand()%2 ? Led::ON : Led::OFF));
    b.redraw_mouth();
    cv::imshow("mouth", b.get_mouth());
    cv::waitKey(1000);
  }
  return 0;
}

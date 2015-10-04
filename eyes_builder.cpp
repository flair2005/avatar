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
#include "eyes_builder.h"
int main(int, char**) {
  std::string default_path = "data/mini_eyes";
  EyeBuilder b;
  b.load_eyes(default_path);
  for (int t = 0; t < 10000; ++t) {
    b.move_both_iris(cos(t/10.), sin(t/10.));
    b.redraw_eyes();
    //    cv::imshow("leye", b.get_leye());
    //    cv::imshow("reye", b.get_reye());
    cv::imshow("eyes", b.get_eyes());
    char c = cv::waitKey(25);
    if ((int) c == 27)
      break;
    else if (c == 'a')
      b.set_state("angry");
    else if (c == 'z')
      b.set_state("laughing");
    else if (c == 'e')
      b.set_state("normal");
    else if (c == 'r')
      b.set_state("sad");
    else if (c == 't')
      b.set_state("sleeping");
    else if (c == 'y')
      b.set_state("sleepy");
    else if (c == 'u')
      b.set_state("surprised");
    else if (c == 'q')
      b.load_eyes(default_path);
    else if (c == 's')
      b.load_default_eyes();
  }
  return 0;
}

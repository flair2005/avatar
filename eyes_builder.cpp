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

void test_eyes(std::string path = "") {
  Eye b;
  if (path.size())
    b.load_imgs(path);
  cv::Mat3b out(b.get_size());
  cv::Vec3b bg_color(0,100,0);
  out.setTo(bg_color);
  std::vector<cv::Mat3b> out_vec;
  std::vector<bool> flips(1, false);
  out_vec.push_back(out);
  for (int t = 0; t < 10000; ++t) {
    b.move_iris(cos(t/10.), sin(t/10.));
    b.redraw(out_vec, flips);
    cv::imshow("eyes", out);
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
    else if (c == 'q') {
      b.load_imgs(path);
      out.create(b.get_size());
      out.setTo(bg_color);
    }
    else if (c == 's') {
      b.load_default_imgs();
      out.create(b.get_size());
      out.setTo(bg_color);
    }
  } // end while(true)
} // end test_eyes();

////////////////////////////////////////////////////////////////////////////////

void test_led() {
  Led l;
  cv::Mat3b out(l.get_size());
  for (int t = 0; t < 10000; ++t) {
    l.set_state((rand()%2 ? Led::ON : Led::OFF));
    l.redraw(out);
    cv::imshow("avatar", out);
    cv::waitKey(1000);
  }
}

////////////////////////////////////////////////////////////////////////////////

void test_avatar() {
  std::string default_path = "data/mini_avatar";
  Avatar b;
  for (int t = 0; t < 10000; ++t) {
    for (int i = 0; i < b.nleds(); ++i)
      b.set_led_state(i, (rand()%2 ? Led::ON : Led::OFF));
    b.move_iris(cos(t/10.), sin(t/10.));
    b.redraw();
    cv::imshow("avatar", b.get_avatar());
    cv::waitKey(50);
  }
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  int idx = 1, choice = 1;
  if (argc < 2) {
    printf("%i: test_eyes()\n", idx++);
    printf("%i: test_eyes('data/mini_eyes'')\n", idx++);
    printf("%i: test_led()\n", idx++);
    printf("%i: test_avatar();\n", idx++);
    return -1;
  }
  choice = atoi(argv[1]);

  if (choice == idx++)
    test_eyes();
  else if (choice == idx++)
    test_eyes("data/mini_eyes");
  else if (choice == idx++)
    test_led();
  else if (choice == idx++)
    test_avatar();
  return 0;
}

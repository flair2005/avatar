/*!
  \file        mouth_builder.h
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

\todo Description of the file
 */
#ifndef MOUTH_BUILDER_H
#define MOUTH_BUILDER_H

#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>

class Led {
public:
  enum State { OFF = 0, ON = 2, UNSET = 3 };
  Led() : _state(OFF), _last_drawn_state(UNSET), _auto_mode(false) {
    load_default_imgs();
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void set_state(Led::State state) { _state = state; }

  //////////////////////////////////////////////////////////////////////////////

  void set_auto_mode(const double & auto_mode_thres) {
    _auto_mode = true;
    _auto_mode_thres = auto_mode_thres;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void unset_auto_mode() { _auto_mode = true; }

  //////////////////////////////////////////////////////////////////////////////

  inline unsigned int get_w() const { return _on_img.rows; }
  inline unsigned int get_h() const { return _on_img.cols; }

  //////////////////////////////////////////////////////////////////////////////

  bool load_imgs(const std::string & led_folder) {
    printf("load_imgs('%s')\n", led_folder.c_str());
  } // end load_imgs();

  //////////////////////////////////////////////////////////////////////////////

  bool redraw_eyes(cv::Mat3b & out, double auto_mode_value = -1) {
    // apply thres if needed
    if (_auto_mode)
      _state = (auto_mode_value > _auto_mode_thres ? ON : OFF);
    if (_state == _last_drawn_state) {
      printf("Led::redraw_eyes(): nothing to do!\n");
      return true; // nothing to do
    }
    cv::Mat3b* to_copy = (_state == OFF ? &_off_img : &_on_img);
    if (out.empty() || to_copy->size() != out.size()){
      printf("Led: size mismatch!\n");
      return false;
    }
    to_copy->copyTo(out);
    _state = _last_drawn_state;
    return true;
  } // end redraw_eyes();

  //////////////////////////////////////////////////////////////////////////////

protected:
  void load_default_imgs() {
    _off_img.create(32, 32);
    _off_img.setTo(cv::Vec3b(0,0,0));
    _off_img.copyTo(_on_img);
    cv::circle(_on_img, cv::Point(16, 16), 16, CV_RGB(0,0,255),-1);
  }

  State _state, _last_drawn_state;
  bool _auto_mode;
  cv::Mat3b _off_img, _on_img;
  double _auto_mode_thres; //! threshold for on/off
}; // end class Led

////////////////////////////////////////////////////////////////////////////////

class MouthBuilder {
public:
  MouthBuilder() {load_default_mouth();}

  //////////////////////////////////////////////////////////////////////////////

  void load_default_mouth() {
    // bg
    unsigned int nleds = 5, mw = 250, mh = 100;
    _bg.create(mh, mw);
    _bg.setTo(cv::Vec3b(0,0,0));
    _bg.copyTo(_mouth);
    // leds
    _leds.resize(nleds, Led()); // default Led
    unsigned int lh = _leds.front().get_h(), lw = _leds.front().get_w();
    _led_rois.clear();
    for (int iled = 1; iled <= nleds; ++iled) {
      cv::Mat3b roi = _mouth(cv::Rect(iled*mw/(nleds+1) - lw/2, mh/2 - lh/2, lw, lh));
      _led_rois.push_back(roi);
    }
    redraw_mouth();
  } // end load_default_mouth();

  //////////////////////////////////////////////////////////////////////////////

  void configure(unsigned int w, unsigned int h) {
  }

  //////////////////////////////////////////////////////////////////////////////

  bool set_led_state(unsigned int led_idx, Led::State state){
    if (led_idx < 0 || led_idx >= _leds.size()) {
      printf("MouthBuilder::set_led(): size mismatch!\n");
      return false;
    }
    _leds[led_idx].set_state(state);
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool redraw_mouth() {
    _bg.copyTo(_mouth);
    unsigned int nleds_ = nleds();
    if (_led_rois.size() != nleds_) {
      printf("MouthBuilder::redraw_mouth(): size mismatch!\n");
      return false;
    }
    for (int iled = 0; iled < nleds_; ++iled) {
      if (!_leds[iled].redraw_eyes(_led_rois[iled])) {
        printf("MouthBuilder::redraw_mouth(): redraw_eyes() went wrong!\n");
        return false;
      }
    }
    return true;
  } // end redraw_eyes()

  //////////////////////////////////////////////////////////////////////////////

  inline const cv::Mat3b & get_mouth() const { return _mouth; }
  inline const unsigned int nleds() const { return _leds.size(); }

protected:
  cv::Mat3b _bg, _mouth;
  std::vector<Led> _leds;
  std::vector<cv::Mat3b> _led_rois;
};

#endif // MOUTH_BUILDER_H


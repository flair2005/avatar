/*!
  \file
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/10/4

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
#ifndef EYES_BUILDER_H
#define EYES_BUILDER_H

#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   printf(__VA_ARGS__)

// utils
#include "timer.h"
// C++
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
// C
#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>

////////////////////////////////////////////////////////////////////////////////

// http://jepsonsblog.blogspot.com.es/2012/10/overlay-transparent-image-in-opencv.html
inline void overlayImage(const cv::Mat3b &background, const cv::Mat4b &foreground,
                         cv::Mat &output, cv::Point2i location) {
  if (background.empty()) {
    //output.release();
    return;
  }
  background.copyTo(output);
  if (foreground.empty())
    return;
  // start at the row indicated by location, or at row 0 if location.y is negative.
  for(int y = std::max(location.y , 0); y < background.rows; ++y) {
    int fY = y - location.y; // because of the translation
    // we are done of we have processed all rows of the foreground image.
    if(fY >= foreground.rows)
      break;
    // start at the column indicated by location,
    // or at column 0 if location.x is negative.
    for(int x = std::max(location.x, 0); x < background.cols; ++x) {
      int fX = x - location.x; // because of the translation.
      // we are done with this row if the column is outside of the foreground image.
      if(fX >= foreground.cols)
        break;
      // determine the opacity of the foregrond pixel, using its fourth (alpha) channel.
      double opacity =
          ((double)foreground.data[fY * foreground.step + fX * foreground.channels() + 3])
          / 255.;
      // and now combine the background and foreground pixel, using the opacity,
      // but only if opacity > 0.
      for(int c = 0; opacity > 0 && c < output.channels(); ++c) {
        unsigned char foregroundPx =
            foreground.data[fY * foreground.step + fX * foreground.channels() + c];
        unsigned char backgroundPx =
            background.data[y * background.step + x * background.channels() + c];
        output.data[y*output.step + output.channels()*x + c] =
            backgroundPx * (1.-opacity) + foregroundPx * opacity;
      } // end for c
    } // end for x
  } // end for y
} // end overlayImage();

////////////////////////////////////////////////////////////////////////////////

// https://stackoverflow.com/questions/5043403/listing-only-folders-in-directory
inline int all_subfolders(const std::string & dir_path,
                          std::vector<std::string> & ans) {
  ans.clear();
  if (!boost::filesystem::exists( dir_path ))
    return -1;
  boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
  for ( boost::filesystem::directory_iterator itr( dir_path ); itr != end_itr; ++itr ) {
    if (!boost::filesystem::is_directory(itr->status()))
      continue;
    ans.push_back(boost::filesystem::basename(itr->path()));
  }
  std::sort(ans.begin(), ans.end()); // alphabetical sort
} // end all_subfolders()

////////////////////////////////////////////////////////////////////////////////

//! \return <= 0 if problem
inline int imread_all_files_in_dir(const std::string & folder,
                                   std::vector<cv::Mat4b> & ans,
                                   const std::string pattern = "") {
  ans.clear();
  DIR *dir = opendir (folder.c_str());
  if (dir == NULL) {
    printf("imread_all_files_in_dir: could not open directory:'%s'\n", folder.c_str());
    return -1;
  }
  struct dirent *ent;
  std::string folder_and_slash = folder + std::string("/");
  std::vector<std::string> filenames;
  while ((ent = readdir (dir)) != NULL) {
    std::string filename(ent->d_name); // pattern check
    if (pattern.size() > 0 && filename.find(pattern) == std::string::npos)
      continue;
    filenames.push_back(filename);
  }
  closedir (dir);
  if (filenames.empty()) {
    printf("Could not read any 4-channel image in '%s'with pattern '%s'\n",
           folder.c_str(), pattern.c_str());
    return -1;
  }

  std::sort(filenames.begin(), filenames.end()); // alphabetical sort
  // now read images
  for (int i = 0; i < filenames.size(); ++i) {
    //DEBUG_PRINT("filename:'%s'\n", filenames[i].c_str());
    cv::Mat im = cv::imread(folder_and_slash + filenames[i], cv::IMREAD_UNCHANGED);
    if (im.empty()) {
      printf("imread_all_files_in_dir: could not read file '%s'\n", filenames[i].c_str());
      continue;
    }
    if (im.channels() != 4) {
      printf("imread_all_files_in_dir: image '%s' does not have 4 channels!\n",
             filenames[i].c_str());
      continue;
    }
    ans.push_back(im);
  } // end loop i
  if (ans.empty())
    printf("Could not read any 4-channel image in '%s'with pattern '%s'\n",
           folder.c_str(), pattern.c_str());
  return ans.size();
} // end imread_all_files_in_dir()

////////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   returns a random number with a gaussian law
 * (standard normal distribution).
 * Its probability density function is
 * phi(x) = 1 / sqrt(2 * PI) * exp(-x^2 / 2)
 */
inline double rand_gaussian() {
  // Box–Muller method
  return sqrt(-2 * log(drand48())) * cos(2.f * M_PI * drand48());
} // end rand_gaussian()

////////////////////////////////////////////////////////////////////////////////

template<typename T>
inline T clamp(T Value, T Min, T Max) {
  return (Value < Min)? Min : (Value > Max)? Max : Value;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class Eye {
public:
  typedef std::string StateName;
  enum Substate { OPEN = 0, BEGIN = 2, BLINK = 3, END = 4 };
  class StateData {
  public:
    StateData() : _blink_period(3), _next_blink_time(3), _eyelid_duration(.06) {}
    StateName _name;
    double _blink_period, _eyelid_duration, _next_blink_time;
    std::vector<cv::Mat4b> _eyelids_blink,  _eyelids_begin,  _eyelids_open;
    unsigned int           _neyelids_blink,  _neyelids_begin,  _neyelids_open;
  }; // end class StateData


  //////////////////////////////////////////////////////////////////////////////

  Eye() { load_default_imgs();}

  //////////////////////////////////////////////////////////////////////////////

  bool load_imgs(const std::string & eyes_folder) {
    DEBUG_PRINT("Eye::load_imgs('%s')\n", eyes_folder.c_str());
    _states_data.clear();
    if (!boost::filesystem::exists(eyes_folder)
        || !boost::filesystem::is_directory(eyes_folder)) {
      printf("Directory '%s' doesn't exist, loading default eyes!\n", eyes_folder.c_str());
      return load_default_imgs();
    }
    // load background RGB
    _bg = cv::imread(eyes_folder + "/bg.png", cv::IMREAD_UNCHANGED);
    if (_bg.empty() || _bg.channels() != 4) {
      printf("Could not load 4-channel bg image @ '%s/bg.png', loading default eyes!\n",
             eyes_folder.c_str());
      return load_default_imgs();
    }
    _force_redraw = true;
    // load iris RGBA
    _iris = cv::imread(eyes_folder + "/iris.png", cv::IMREAD_UNCHANGED);
    if (_iris.empty() || _iris.channels() != 4) {
      printf("Could not load 4-channel iris image @ '%s/iris.png', loading default eyes!\n",
             eyes_folder.c_str());
      return load_default_imgs();
    }
    // load all states
    std::vector<std::string> states;
    all_subfolders(eyes_folder, states);
    for (int i = 0; i < states.size(); ++i) {
      DEBUG_PRINT("state:'%s'\n", states[i].c_str());
      load_state(states[i], eyes_folder + "/" + states[i]);
    }
    if (_states_data.empty()) {
      printf("Could not load any state, loading default eyes!\n");
      return load_default_imgs();
    }

    // safe default values
    _curr_statedata = NULL;
    _curr_eyelid = NULL;
    StateName first_state = _states_data.begin()->second._name;
    if (!set_state_notransition("normal")
        && !set_state_notransition(first_state)) {
      printf("Could not set state 'normal' nor first state '%s', loading default eyes!\n",
             first_state.c_str());
      return load_default_imgs();
    }
    move_iris(0, 0);
    return true;
  } // end load_imgs();

  //////////////////////////////////////////////////////////////////////////////

  bool load_default_imgs() {
    DEBUG_PRINT("Eye::load_default_imgs()\n");
    _states_data.clear();
    int we = 100, wi = we / 2;
    _bg.create(we, we);
    _bg.setTo(cv::Vec4b(0,0,0,0));
    cv::circle(_bg, cv::Point(we/2,we/2), we/2, cv::Scalar(255,255,255,255),-1);
    _iris.create(wi, wi);
    _iris.setTo(cv::Vec4b(0,0,0,0));
    cv::circle(_iris, cv::Point(wi/2,wi/2), wi/2, cv::Scalar(50,0,00,255),-1); // blue iris
    cv::circle(_iris, cv::Point(.7*wi,.3*wi), wi/10, cv::Scalar(255,255,255,255),-1); // white point
    StateData data;
    data._name = "normal";
    cv::Mat4b eyelid(we, we, cv::Vec4b(0,0,0,0));
    // normal
    data._eyelids_begin.push_back(eyelid.clone());
    data._neyelids_begin = 1;
    data._eyelids_open.push_back(eyelid.clone());
    data._neyelids_open = 1;
    for (int i = 1; i < 8; ++i) {
      eyelid.setTo(cv::Vec4b(0,0,0,0));
      cv::circle(eyelid, cv::Point(we/2,we/2), we/2, cv::Scalar(0,0,0,255),-1);
      cv::rectangle(eyelid, cv::Point(0, (i<4?i:8-i) *we/4),
                    cv::Point(we,we), cv::Scalar(0,0,0,0), -1);
      data._eyelids_blink.push_back(eyelid.clone());
    }
    data._neyelids_blink = data._eyelids_blink.size();
    _states_data.insert(std::pair<StateName, StateData>("normal", data));
    // default values
    _curr_statedata = NULL;
    _curr_eyelid = NULL;
    if (!set_state_notransition("normal")) {
      printf("Could not set state 'normal'!\n");
      return false;
    }
    move_iris(0, 0);
    _force_redraw = true;
    return true;
  } // end load_default_imgs()

  //////////////////////////////////////////////////////////////////////////////

  cv::Size get_size() const { return _bg.size(); }

  //////////////////////////////////////////////////////////////////////////////

  bool set_state(const StateName & state) {
    DEBUG_PRINT("Eye::set_state('%s')\n", state.c_str());
    // check state exists
    if (_states_data.count(state) == 0) {
      printf("Eye::set_state('%s'): set does not exist\n", state.c_str());
      return false;
    }
    if (!_curr_statedata) // there was no previous state
      return set_state_notransition(state);
    DEBUG_PRINT("Queued state '%s'\n", state.c_str());
    _next_state = state; // we will need to switch to SubState END
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! irisx, irisy in [-1, 1]
  inline void move_iris(double irisx, double irisy) {
    DEBUG_PRINT("Eye::move_iris(%g,%g)\n", irisx, irisy);
    _irisx = clamp(irisx, -1., 1.);
    _irisy = clamp(irisy, -1., 1.);
    _iris_trans.x = (1 + _irisx) * (_bg.cols / 2 -_iris.cols / 2);
    _iris_trans.y = (1 + _irisy) * (_bg.rows / 2 -_iris.rows / 2);
    _flip_iris_trans.x = (1 - _irisx) * (_bg.cols / 2 -_iris.cols / 2);
    _flip_iris_trans.y = _iris_trans.y;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool redraw(std::vector<cv::Mat3b> & out, const std::vector<bool> & eye_flips) {
    DEBUG_PRINT("Eye::redraw()\n");
    if (out.empty())
      return true;
    unsigned int nimgs = out.size();
    if (eye_flips.size() != nimgs) {
      printf("Eye::redraw(): eye flips size mismatch, %i rois, %i flips!\n",
             nimgs, eye_flips.size());
      return false;
    }

    //cv::imshow("iris", _iris); cv::waitKey(0);
    for (int i = 0; i < nimgs; ++i) {
      if (out[i].size() != _bg.size()) { // init images
        printf("Eye::redraw(): size mismatch, bg=%ix%i, out=%ix%i!\n",
               _bg.cols, _bg.rows, out[i].cols, out[i].rows);
        return false;
      }
    } // end loop i
    if (_curr_statedata == NULL) {
      printf("Eye::redraw(): _curr_statedata undefined!\n");
      return false;
    }
    // determine what eyelid needs to be drawn
    double time = _curr_substate_timer.getTimeSeconds();
    StateName curr_state = _curr_statedata->_name;
    unsigned int next_eyelid_idx = 1. * time / _curr_statedata->_eyelid_duration;
    cv::Mat4b* next_eyelid = _curr_eyelid;

    if (_curr_substate == BEGIN) {
      // check for transitions to OPEN
      if (next_eyelid_idx < 0 || next_eyelid_idx >= _curr_statedata->_neyelids_begin) {
        if (_next_state != curr_state) { // state queued -> end the current state
          DEBUG_PRINT("'%s':BEGIN -> END\n", curr_state.c_str());
          _curr_substate = END;
          _curr_substate_timer.reset();
          next_eyelid = &(_curr_statedata->_eyelids_begin.back());
        } else { // no state queued
          DEBUG_PRINT("'%s':BEGIN -> OPEN\n", curr_state.c_str());
          _curr_substate = OPEN;
          _curr_substate_timer.reset();
          next_eyelid = &(_curr_statedata->_eyelids_open.front());
        }
      }
      else { // default BEGIN animation
        next_eyelid = &(_curr_statedata->_eyelids_begin.at(next_eyelid_idx));
      }
    } // end BEGIN

    else if (_curr_substate == OPEN) {
      // check for transitions to blink
      if (_next_state != curr_state) { // state queued -> end the current state
        DEBUG_PRINT("'%s':OPEN -> END\n", curr_state.c_str());
        _curr_substate = END;
        _curr_substate_timer.reset();
        next_eyelid = &(_curr_statedata->_eyelids_begin.back());
      } else if (time > _curr_statedata->_next_blink_time) { // timeout for blink
        DEBUG_PRINT("'%s':OPEN -> BLINK\n", curr_state.c_str());
        _curr_substate = BLINK;
        _curr_substate_timer.reset();
        next_eyelid = &(_curr_statedata->_eyelids_blink.front());
      }
      else { // default OPEN eyelid
        next_eyelid = &(_curr_statedata->_eyelids_open.front());
      }
    } // end OPEN

    else if (_curr_substate == BLINK) {
      // check for transitions to OPEN
      if (next_eyelid_idx < 0 || next_eyelid_idx >= _curr_statedata->_neyelids_blink) {
        if (_next_state != curr_state) { // state queued -> end the current state
          DEBUG_PRINT("'%s':BLINK -> END\n", curr_state.c_str());
          _curr_substate = END;
          _curr_substate_timer.reset();
          next_eyelid = &(_curr_statedata->_eyelids_begin.back());
        } else {
          DEBUG_PRINT("'%s':BLINK -> OPEN\n", curr_state.c_str());
          _curr_substate = OPEN;
          _curr_substate_timer.reset();
          next_eyelid = &(_curr_statedata->_eyelids_open.front());
          // gaussian probability for next blink
          double blink = .3 * rand_gaussian() + _curr_statedata->_blink_period;
          blink = std::max(blink, 1.);
          _curr_statedata->_next_blink_time = blink;
        }
      }
      else { // default OPEN BLINK eyelid
        next_eyelid = &(_curr_statedata->_eyelids_blink.at(next_eyelid_idx));
      }
    } // end BLINK

    else if (_curr_substate == END) {
      if (next_eyelid_idx < 0 || next_eyelid_idx >= _curr_statedata->_neyelids_begin) {
        DEBUG_PRINT("'%s':END -> BEGIN\n", curr_state.c_str());
        set_state_notransition(_next_state);
      }
      else { // default END animation = reverse BEGIN
        unsigned int idx = _curr_statedata->_neyelids_begin - 1 - next_eyelid_idx;
        next_eyelid = &(_curr_statedata->_eyelids_begin.at(idx));
      }
    } // end END

    if (next_eyelid == NULL) {
      printf("next_eyelid = NULL, something went wrong...\n");
      for (int i = 0; i < nimgs; ++i)
        out[i].setTo(0);
      return false;
    }
    // force redraw if iris has moved a lot
    if (_force_redraw
        || cv::norm(_prev_iris_trans - _iris_trans) > 1E-2
        || next_eyelid != _curr_eyelid) { // pointers comparison
      for (int i = 0; i < nimgs; ++i) {
        overlayImage(out[i], _bg, out[i], cv::Point()); // use transparency
        overlayImage(out[i], _iris, out[i], (eye_flips[i] ? _flip_iris_trans : _iris_trans));
        overlayImage(out[i], *next_eyelid, out[i], cv::Point());
        if (eye_flips[i])
          cv::flip(out[i], out[i], 1);
      }
      _curr_eyelid = next_eyelid;
      _prev_iris_trans = _iris_trans;
      _force_redraw = false;
    }
    return true;
  } // end redraw()

  //////////////////////////////////////////////////////////////////////////////

protected:

  //////////////////////////////////////////////////////////////////////////////

  bool set_state_notransition(const StateName & state) {
    DEBUG_PRINT("Eye::set_state_notransition('%s')\n", state.c_str());
    std::map<StateName, StateData>::iterator it = _states_data.find(state);
    if (it == _states_data.end()) {
      printf("Eye::set_state_notransition('%s'): could not set state\n", state.c_str());
      return false;
    }
    _next_state = state;
    _curr_statedata = &(it->second);
    _curr_substate = BEGIN;
    _curr_substate_timer.reset();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! load the eyelids of a given state
  inline bool load_state(const StateName & state,
                         const std::string & folder) {
    DEBUG_PRINT("Eye::load_state('%s')\n", state.c_str());
    StateData data;
    data._name = state;
    if (imread_all_files_in_dir(folder, data._eyelids_begin, "begin_") <= 0
        || imread_all_files_in_dir(folder, data._eyelids_blink, "blink_") <= 0
        || imread_all_files_in_dir(folder, data._eyelids_open, "open_") <= 0) {
      printf("Eye::load_state('%s'): could not load one of the states!\n", state.c_str());
      return false;
    }
    data._neyelids_begin = data._eyelids_begin.size();
    data._neyelids_blink = data._eyelids_blink.size();
    data._neyelids_open = data._eyelids_open.size();
    _states_data.insert(std::pair<StateName, StateData>(state, data));
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  double _irisx, _irisy;
  cv::Point _iris_trans, _flip_iris_trans, _prev_iris_trans;
  cv::Mat4b _iris, _bg;
  bool _force_redraw;
  cv::Mat4b *_curr_eyelid; //!< pointer to the current eyelid frame
  Substate _curr_substate;
  Timer _curr_substate_timer;
  StateData* _curr_statedata;
  StateName _next_state;
  unsigned int _curr_substate_idx;
  std::map<StateName, StateData> _states_data;
}; // end class Eye

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

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

  cv::Size get_size() const { return _on_img.size(); }

  //////////////////////////////////////////////////////////////////////////////

  bool load_imgs(const std::string & led_folder) {
    DEBUG_PRINT("load_imgs('%s')\n", led_folder.c_str());
  } // end load_imgs();

  //////////////////////////////////////////////////////////////////////////////

  inline void set_auto_mode_value(double auto_mode_value) {
    // apply thres if needed
    if (_auto_mode)
      _state = (auto_mode_value > _auto_mode_thres ? ON : OFF);
  }

  //////////////////////////////////////////////////////////////////////////////

  bool redraw(cv::Mat3b & out) {
    DEBUG_PRINT("Led::redraw()\n");
    if (_state == _last_drawn_state) {
      DEBUG_PRINT("Led::redraw(): nothing to do!\n");
      return true; // nothing to do
    }
    cv::Mat4b* to_copy = (_state == OFF ? &_off_img : &_on_img);
    if (out.empty() || to_copy->size() != out.size()){
      printf("Led::redraw(): size mismatch!\n");
      return false;
    }
    overlayImage(out, *to_copy, out, cv::Point()); // use transparency
    _state = _last_drawn_state;
    return true;
  } // end redraw();

  //////////////////////////////////////////////////////////////////////////////

protected:
  void load_default_imgs() {
    _off_img.create(32, 32);
    _off_img.setTo(cv::Vec4b(0,0,0,0));
    _off_img.copyTo(_on_img);
    cv::circle(_off_img, cv::Point(16, 16), 16, cv::Scalar(0,0,0,255),  -1);
    cv::circle(_on_img,  cv::Point(16, 16), 16, cv::Scalar(255,0,0,255),-1);
  }

  State _state, _last_drawn_state;
  bool _auto_mode;
  cv::Mat4b _off_img, _on_img;
  double _auto_mode_thres; //! threshold for on/off
}; // end class Led

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class Avatar {
public:
  Avatar() {load_default_avatar();}

  //////////////////////////////////////////////////////////////////////////////

  inline bool add_eye(const Eye & eye, const cv::Point & center_pos,
                      bool eye_flip = false) {
    if (!add_roi(eye, center_pos, _eye_rois))
      return false;
    _eye_flips.push_back(eye_flip);
    //_eye = eye; // add it at the end so to preserve integrity if roi out of bounds
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline bool add_led(const Led & led, const cv::Point & center_pos) {
    if (!add_roi(led, center_pos, _led_rois))
      return false;
    _leds.push_back(led); // add it at the end so to preserve integrity if roi out of bounds
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  void load_default_avatar() {
    DEBUG_PRINT("Avatar::load_default_avatar()\n");
    // bg
    unsigned int mw = 640, mh = 480, nleds = 5;
    _bg.create(mh, mw);
    _bg.setTo(cv::Vec3b(100,100,100));
    _bg.copyTo(_avatar);
    // eyes
    _eye_rois.clear();
    _eye_flips.clear();
    _eye.load_default_imgs();
    add_eye(_eye, cv::Point(  mw/3, mh/4), false);
    add_eye(_eye, cv::Point(2*mw/3, mh/4), true);
    // leds
    _leds.clear();
    _led_rois.clear();
    for (int iled = 1; iled <= nleds; ++iled)
      add_led(Led(), cv::Point(iled*mw/(nleds+1), 3*mh/4));
    _avatar.release(); // so that we redraw it in redraw()
    redraw();
  } // end load_default_avatar();

  //////////////////////////////////////////////////////////////////////////////

  void configure(unsigned int w, unsigned int h) {
  }

  //////////////////////////////////////////////////////////////////////////////

  bool set_led_state(unsigned int led_idx, Led::State state){
    if (led_idx < 0 || led_idx >= _leds.size()) {
      printf("Avatar::set_led(): size mismatch!\n");
      return false;
    }
    _leds[led_idx].set_state(state);
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! irisx, irisy in [-1, 1]
  inline void move_iris(double irisx, double irisy) {
    _eye.move_iris(irisx, irisy);
  }

  //////////////////////////////////////////////////////////////////////////////

  bool redraw() {
    DEBUG_PRINT("Avatar::redraw()\n");
    if (_avatar.empty())
      _bg.copyTo(_avatar);
    // eye
    for (int i = 0; i < _eye_rois.size(); ++i) {
      std::vector<cv::Mat3b> avatar_cuts;
      for (int i = 0; i < _eye_rois.size(); ++i)
        avatar_cuts.push_back(_avatar(_eye_rois[i]));
      if (!_eye.redraw(avatar_cuts, _eye_flips)) {
        printf("Avatar::redraw(): Eye::redraw() went wrong!\n");
        return false;
      }
    }
    // leds
    if (!redraw_comps(_leds, _led_rois))
      return false;
    return true;
  } // end redraw()

  //////////////////////////////////////////////////////////////////////////////

  inline const cv::Mat3b & get_avatar() const { return _avatar; }
  inline const unsigned int neyes() const { return _eye_rois.size(); }
  inline const unsigned int nleds() const { return _leds.size(); }

protected:

  //////////////////////////////////////////////////////////////////////////////

  template<class _T>
  inline bool add_roi(const _T & comp, const cv::Point & center_pos,
                      std::vector<cv::Rect> & rois) {
    unsigned int lh = comp.get_size().height, lw = comp.get_size().width;
    if (center_pos.x < lw/2 || lw/2 + center_pos.x >= _avatar.cols
        || center_pos.y < lh/2 || lh/2 + center_pos.y >= _avatar.rows) {
      printf("Avatar::add_comp(): cannot add comp (%i,%i) at given position (%i,%i)!\n",
             lw, lh, center_pos.x, center_pos.y);
      return false;
    }
    rois.push_back(cv::Rect(center_pos.x - lw/2, center_pos.y - lh/2, lw, lh));
    return true;
  } // end add_comp()

  //////////////////////////////////////////////////////////////////////////////

  template<class _T>
  inline bool redraw_comps(std::vector<_T> & comps,
                           std::vector<cv::Rect> & rois) {
    DEBUG_PRINT("Avatar::redraw_comps()\n");
    unsigned int ncomps = comps.size();
    if (rois.size() != ncomps) {
      printf("Avatar::redraw_comps(): comp size mismatch!\n");
      return false;
    }
    for (int icomp = 0; icomp < ncomps; ++icomp) {
      cv::Mat3b avatar_cut = _avatar(rois[icomp]);
      if (!comps[icomp].redraw(avatar_cut)) {
        printf("Avatar::redraw_comps(): comp redraw() went wrong!\n");
        return false;
      }
    }
    return true;
  } // end redraw_comps()

  //////////////////////////////////////////////////////////////////////////////

  cv::Mat3b _bg, _avatar;
  std::vector<Led> _leds;
  Eye _eye;
  std::vector<cv::Rect> _led_rois, _eye_rois;
  std::vector<bool> _eye_flips;
}; // end class Avatar

#endif // EYES_BUILDER_H

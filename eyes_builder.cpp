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
    output.release();
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
    //printf("filename:'%s'\n", filenames[i].c_str());
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

inline void flip_vector(const cv::vector<cv::Mat4b> & in,
                        cv::vector<cv::Mat4b> & out) {
  unsigned int size = in.size();
  out.resize(size);
  for (int i = 0; i < size; ++i)
    cv::flip(in[i], out[i], 1);
} // end flip_vector();

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

class EyeBuilder {
public:
  typedef std::string StateName;
  enum Substate { OPEN = 0, BEGIN = 2, BLINK = 3, END = 4 };
  class StateData {
  public:
    StateData() : _blink_period(3), _next_blink_time(3), _eyelid_duration(.06) {}
    ~StateData() {/*foo.release();*/}
    StateName _name;
    double _blink_period, _eyelid_duration, _next_blink_time;
    std::vector<cv::Mat4b> _leyelids_blink,  _leyelids_begin,  _leyelids_open;
    std::vector<cv::Mat4b> _reyelids_blink,  _reyelids_begin,  _reyelids_open;
    unsigned int           _neyelids_blink,  _neyelids_begin,  _neyelids_open;
    cv::Mat4b foo;
  };

  //////////////////////////////////////////////////////////////////////////////

  EyeBuilder()                                { load_default_eyes();}
  ~EyeBuilder() {}
  EyeBuilder(const std::string & eyes_folder) { load_eyes(eyes_folder); }
  //////////////////////////////////////////////////////////////////////////////

  bool load_eyes(const std::string & eyes_folder) {
    _states_data.clear();
    if (!boost::filesystem::exists(eyes_folder)
        || !boost::filesystem::is_directory(eyes_folder)) {
      printf("Directory '%s' doesn't exist, loading default eyes!\n", eyes_folder.c_str());
      return load_default_eyes();
    }
    // load background RGB
    _bg = cv::imread(eyes_folder + "/bg.png", cv::IMREAD_COLOR);
    if (_bg.empty()) {
      printf("Could not load 3-channel bg image @ '%s/bg.png', loading default eyes!\n",
             eyes_folder.c_str());
      return load_default_eyes();
    }
    // load iris RGBA
    _iris = cv::imread(eyes_folder + "/iris.png", cv::IMREAD_UNCHANGED);
    if (_iris.empty() || _iris.channels() != 4) {
      printf("Could not load 4-channel iris image @ '%s/iris.png', loading default eyes!\n",
             eyes_folder.c_str());
      return load_default_eyes();
    }
    // load all states
    std::vector<std::string> states;
    all_subfolders(eyes_folder, states);
    for (int i = 0; i < states.size(); ++i) {
      printf("state:'%s'\n", states[i].c_str());
      load_state(states[i], eyes_folder + "/" + states[i]);
    }
    if (_states_data.empty()) {
      printf("Could not load any state, loading default eyes!\n");
      return load_default_eyes();
    }

    // safe default values
    _curr_statedata = NULL;
    StateName first_state = _states_data.begin()->second._name;
    if (!set_state_notransition("normal")
        && !set_state_notransition(first_state)) {
      printf("Could not set state 'normal' nor first state '%s', loading default eyes!\n",
             first_state.c_str());
      return load_default_eyes();
    }
    move_both_iris(0, 0);
    return redraw_eyes();
  } // end load_eyes();

  //////////////////////////////////////////////////////////////////////////////

  bool load_default_eyes() {
    _states_data.clear();
    int we = 100, wi = we / 2;
    _bg.create(we, we);
    _bg.setTo(cv::Vec3b(0,0,0));
    cv::circle(_bg, cv::Point(we/2,we/2), we/2, cv::Scalar(255,255,255),-1);
    _iris.create(wi, wi);
    _iris.setTo(cv::Vec4b(0,0,0,0));
    cv::circle(_iris, cv::Point(wi/2,wi/2), wi/2, cv::Scalar(50,0,00,255),-1); // blue iris
    cv::circle(_iris, cv::Point(.7*wi,.3*wi), wi/10, cv::Scalar(255,255,255,255),-1); // white point
    StateData data;
    data._name = "normal";
    cv::Mat4b eyelid(we, we, cv::Vec4b(0,0,0,0));
    // normal
    data._leyelids_begin.push_back(eyelid.clone());
    data._reyelids_begin.push_back(eyelid.clone());
    data._neyelids_begin = 1;
    data._leyelids_open.push_back(eyelid.clone());
    data._reyelids_open.push_back(eyelid.clone());
    data._neyelids_open = 1;
    for (int i = 1; i < 8; ++i) {
      eyelid.setTo(cv::Vec4b(0,0,0,0));
      cv::circle(eyelid, cv::Point(we/2,we/2), we/2, cv::Scalar(0,0,0,255),-1);
      cv::rectangle(eyelid, cv::Point(0, (i<4?i:8-i) *we/4),
                    cv::Point(we,we), cv::Scalar(0,0,0,0), -1);
      data._leyelids_blink.push_back(eyelid.clone());
      data._reyelids_blink.push_back(eyelid.clone());
    }
    data._neyelids_blink = data._leyelids_blink.size();
    _states_data.insert(std::pair<StateName, StateData>("normal", data));
    // default values
    _curr_statedata = NULL;
    set_state_notransition("normal");
    move_both_iris(0, 0);
    return redraw_eyes();
  }

  //////////////////////////////////////////////////////////////////////////////

  bool set_state(const StateName & state) {
    // check state exists
    if (_states_data.count(state) == 0)
      return false;
    if (!_curr_statedata) // there was no previous state
      return set_state_notransition(state);
    printf("Queued state '%s'\n", state.c_str());
    _next_state = state; // we will need to switch to SubState END
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! irisx, irisy in [-1, 1]
  inline void move_both_iris(double irisx, double irisy) {
    move_left_iris(irisx, irisy);
    move_right_iris(irisx, irisy);
  }
  //! irisx, irisy in [-1, 1]
  inline void move_left_iris(double irisx, double irisy) {
    _lirisx = clamp(irisx, -1., 1.);
    _lirisy = clamp(irisy, -1., 1.);
    _liris_trans.x = (1 + _lirisx) * (_bg.cols / 2 -_iris.cols / 2);
    _liris_trans.y = (1 + _lirisy) * (_bg.rows / 2 -_iris.rows / 2);
  }
  //! irisx, irisy in [-1, 1]
  inline void move_right_iris(double irisx, double irisy) {
    _ririsx = clamp(irisx, -1., 1.);
    _ririsy = clamp(irisy, -1., 1.);
    _riris_trans.x = (1 + _ririsx) * (_bg.cols / 2 -_iris.cols / 2);
    _riris_trans.y = (1 + _ririsy) * (_bg.rows / 2 -_iris.rows / 2);
  }

  //////////////////////////////////////////////////////////////////////////////

  bool redraw_eyes() {
    // determine what eyelid needs to be drawn
    double time = _curr_substate_timer.getTimeSeconds();
    StateName curr_state = _curr_statedata->_name;
    unsigned int next_eyelid_idx = 1. * time / _curr_statedata->_eyelid_duration;
    cv::Mat4b* next_leyelid = _curr_leyelid,
        *next_reyelid = _curr_reyelid;

    if (_curr_substate == BEGIN) {
      // check for transitions to OPEN
      if (next_eyelid_idx < 0 || next_eyelid_idx >= _curr_statedata->_neyelids_begin) {
        if (_next_state != curr_state) { // state queued -> end the current state
          printf("'%s':BEGIN -> END\n", curr_state.c_str());
          _curr_substate = END;
          _curr_substate_timer.reset();
        } else { // no state queued
          printf("'%s':BEGIN -> OPEN\n", curr_state.c_str());
          _curr_substate = OPEN;
          _curr_substate_timer.reset();
        }
      }
      else { // default BEGIN animation
        next_leyelid = &(_curr_statedata->_leyelids_begin.at(next_eyelid_idx));
        next_reyelid = &(_curr_statedata->_reyelids_begin.at(next_eyelid_idx));
      }
    } // end BEGIN

    else if (_curr_substate == OPEN) {
      // check for transitions to blink
      if (_next_state != curr_state) { // state queued -> end the current state
        printf("'%s':OPEN -> END\n", curr_state.c_str());
        _curr_substate = END;
        _curr_substate_timer.reset();
      } else if (time > _curr_statedata->_next_blink_time) { // timeout for blink
        printf("'%s':OPEN -> BLINK\n", curr_state.c_str());
        _curr_substate = BLINK;
        _curr_substate_timer.reset();
      }
      else { // default OPEN eyelid
        next_leyelid = &(_curr_statedata->_leyelids_open.front());
        next_reyelid = &(_curr_statedata->_reyelids_open.front());
      }
    } // end OPEN

    else if (_curr_substate == BLINK) {
      // check for transitions to OPEN
      if (next_eyelid_idx < 0 || next_eyelid_idx >= _curr_statedata->_neyelids_blink) {
        if (_next_state != curr_state) { // state queued -> end the current state
          printf("'%s':BLINK -> END\n", curr_state.c_str());
          _curr_substate = END;
          _curr_substate_timer.reset();
        } else {
          printf("'%s':BLINK -> OPEN\n", curr_state.c_str());
          _curr_substate = OPEN;
          _curr_substate_timer.reset();
          // gaussian probability for next blink
          double blink = .3 * rand_gaussian() + _curr_statedata->_blink_period;
          blink = std::max(blink, 1.);
          _curr_statedata->_next_blink_time = blink;
        }
      }
      else { // default OPEN BLINK eyelid
        next_leyelid = &(_curr_statedata->_leyelids_blink.at(next_eyelid_idx));
        next_reyelid = &(_curr_statedata->_reyelids_blink.at(next_eyelid_idx));
      }
    } // end BLINK

    else if (_curr_substate == END) {
      if (next_eyelid_idx < 0 || next_eyelid_idx >= _curr_statedata->_neyelids_begin) {
        printf("'%s':END -> BEGIN", curr_state.c_str());
        set_state_notransition(_next_state);
      }
      else { // default END animation = reverse BEGIN
        unsigned int idx = _curr_statedata->_neyelids_begin - 1 - next_eyelid_idx;
        next_leyelid = &(_curr_statedata->_leyelids_begin.at(idx));
        next_reyelid = &(_curr_statedata->_reyelids_begin.at(idx));
      }
    } // end END

    // force redraw if iris has moved a lot
    if (_leye.empty()
        || _curr_leyelid  == NULL
        || cv::norm(_prev_liris_trans - _liris_trans) > 1E-2
        || next_leyelid != _curr_leyelid) { // pointers comparison
      overlayImage(_bg, _iris, _leye, _liris_trans);
      overlayImage(_leye, *next_leyelid, _leye, cv::Point());
      _curr_leyelid = next_leyelid;
      _prev_liris_trans = _liris_trans;
    }
    if (_reye.empty()
        || _curr_reyelid  == NULL
        || cv::norm(_prev_riris_trans - _riris_trans) > 1E-2
        || next_reyelid != _curr_reyelid) { // pointers comparison
      overlayImage(_bg, _iris, _reye, _riris_trans);
      overlayImage(_reye, *next_reyelid, _reye, cv::Point());
      _curr_reyelid = next_reyelid;
      _prev_riris_trans = _riris_trans;
    }
    return true;
  } // end redraw_eyes()

  //////////////////////////////////////////////////////////////////////////////

  inline const cv::Mat3b & get_leye()  const { return _leye; }
  inline const cv::Mat3b & get_reye() const { return _reye; }

  //////////////////////////////////////////////////////////////////////////////

protected:

  //////////////////////////////////////////////////////////////////////////////

  bool set_state_notransition(const StateName & state) {
    std::map<StateName, StateData>::iterator it = _states_data.find(state);
    if (it == _states_data.end())
      return false;
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
    StateData data;
    data._name = state;
    if (imread_all_files_in_dir(folder, data._leyelids_begin, "begin_") <= 0
        || imread_all_files_in_dir(folder, data._leyelids_blink, "blink_") <= 0
        || imread_all_files_in_dir(folder, data._leyelids_open, "open_") <= 0)
      return false;
    data._neyelids_begin = data._leyelids_begin.size();
    data._neyelids_blink = data._leyelids_blink.size();
    data._neyelids_open = data._leyelids_open.size();
    // flip for right eye
    flip_vector(data._leyelids_begin, data._reyelids_begin);
    flip_vector(data._leyelids_blink, data._reyelids_blink);
    flip_vector(data._leyelids_open, data._reyelids_open);
    _states_data.insert(std::pair<StateName, StateData>(state, data));
    return true;
  }

  double _lirisx, _lirisy, _ririsx, _ririsy;
  cv::Point _liris_trans, _prev_liris_trans, _riris_trans, _prev_riris_trans;
  cv::Mat4b _iris;
  cv::Mat3b _bg, _leye, _reye;
  cv::Mat4b *_curr_leyelid, *_curr_reyelid; //!< pointer to the current eyelid frame
  Substate _curr_substate;
  Timer _curr_substate_timer;
  StateData* _curr_statedata;
  StateName _next_state;
  unsigned int _curr_substate_idx;
  std::map<StateName, StateData> _states_data;
}; // end class EyeBuilder

////////////////////////////////////////////////////////////////////////////////

int main(int, char**) {
  std::string default_path = "/home/arnaud/Dropbox/work/eyes_builder/data/mini_eyes";
  EyeBuilder b(default_path);
  for (int t = 0; t < 10000; ++t) {
    b.move_both_iris(cos(t/10.), sin(t/10.));
    b.redraw_eyes();
    cv::imshow("leye", b.get_leye());
    cv::imshow("reye", b.get_reye());
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

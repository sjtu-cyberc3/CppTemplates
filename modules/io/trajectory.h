#pragma once
#include "basic_io.h"

#include "modules/sensordata/key_frame.h"

namespace io {
class Trajectory : public BasicIOType {
 private:
  size_t index = 0;

 public:
  Trajectory(std::string path) : BasicIOType(path) {}

  void save_invre(const std::deque<KeyFrame>& deque, int format = 2) {
    for (size_t i = 0; i < deque.size(); i++) {
      if (i < index)
        continue;

      auto pose = deque[i];
      save(pose);

      index = i + 1;
    }
  }

  virtual void save(const KeyFrame& keyframe_)       = 0;
  virtual void read(std::deque<KeyFrame>& keyframe_) = 0;
};

class TraKITTI : public Trajectory {
 public:
  TraKITTI(std::string path) : Trajectory(path) {}
  void save(const KeyFrame& pose) override {
    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        auto posetemp = pose;
        trajectory_ofs_ << posetemp.matrix()(i, j);
        if (i == 2 && j == 3) {
          trajectory_ofs_ << std::endl;
        } else {
          trajectory_ofs_ << " ";
        }
      }
    }
  }

  void read(std::deque<KeyFrame>& keyframes) override {
    std::ifstream infile;
    infile.open(path_);
    assert(infile.is_open());

    std::string temp;

    double num[12];

    while (getline(infile, temp)) {
      KeyFrame          newkey;
      Eigen::Matrix4d   pose;
      int               i = 0;
      std::stringstream word(temp);
      std::string       s;
      while (word >> s) {
        double a = atof(s.c_str());
        num[i]   = a;
        i++;
      }

      pose(0, 0) = num[0];
      pose(0, 1) = num[1];
      pose(0, 2) = num[2];
      pose(0, 3) = num[3];
      pose(1, 0) = num[4];
      pose(1, 1) = num[5];
      pose(1, 2) = num[6];
      pose(1, 3) = num[7];
      pose(2, 0) = num[8];
      pose(2, 1) = num[9];
      pose(2, 2) = num[10];
      pose(2, 3) = num[11];
      newkey.set(pose);

      keyframes.push_back(newkey);
    }
    infile.close();
    std::cout << keyframes.size() << std::endl;
  }
};
class TraTUM : public Trajectory {
 public:
  TraTUM(std::string path) : Trajectory(path) {}
  void save(const KeyFrame& keyframe_) override {
    auto posetemp = keyframe_;
    auto wxyz     = posetemp.rot.quat();

    trajectory_ofs_ << std::fixed << posetemp.time << " " << posetemp.trans.x() << " " << posetemp.trans.y() << " " << posetemp.trans.z() << " "
                    << wxyz.x() << " " << wxyz.y() << " " << wxyz.z() << " " << wxyz.w() << std::endl;
  }

  // timestamp x y z q_x q_y q_z q_w
  void read(std::deque<KeyFrame>& keyframes) override {
    std::ifstream infile;
    infile.open(path_);
    assert(infile.is_open());

    std::string temp;

    double num[8];

    int line = 0;
    while (getline(infile, temp)) {
      int               i = 0;
      std::stringstream word(temp);
      std::string       s;
      while (word >> s) {
        double a = atof(s.c_str());
        num[i]   = a;
        i++;
      }

      KeyFrame newkey;
      newkey.index = line;
      newkey.time  = num[0];
      newkey.trans.set(num[1], num[2], num[3]);
      newkey.rot.set(num[7], num[4], num[5], num[6]);

      keyframes.push_back(newkey);
      line++;
    }
    infile.close();
    std::cout << keyframes.size() << std::endl;
  }
};

class Buildin : public Trajectory {
 public:
  Buildin(std::string path) : Trajectory(path) {}
  void save(const KeyFrame& keyframe_) override {
    trajectory_ofs_ << keyframe_;
  }

  void read(std::deque<KeyFrame>& deque) override {
    std::ifstream infile(path_);
    assert(infile.is_open());

    deque.clear();

    std::string thisline;
    while (getline(infile, thisline)) {
      std::stringstream word(thisline);

      KeyFrame this_data;
      word >> this_data;

      deque.push_back(this_data);
    }
    infile.close();
    std::cout << deque.size() << std::endl;
  }
};
}  // namespace io
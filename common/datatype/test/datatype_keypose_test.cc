// datatype
#include "common/datatype/keypose.h"
#include "common/datatype/io.h"
// gtest
#include <gtest/gtest.h>
// standard
#include <cstdio>
#include <random>

// param
constexpr int NUM  = 1000;
constexpr int SEED = 0;

// namespace
using namespace datatype;

// tests
TEST(datatype, keypose) {
  std::vector<KeyPose>             data_save(NUM);
  std::vector<KeyPose>             data_load;
  std::string                      filename = "keypose_test.txt";
  SensorIO<KeyPose>                io;
  std::default_random_engine       rng(SEED);
  std::normal_distribution<double> norm;
  std::uniform_int_distribution    uniform;
  // generate random data
  for (int i = 0; i < NUM; i++) {
    double       time      = norm(rng);
    unsigned int index     = norm(rng);
    data_save[i].trans.x() = norm(rng);
    data_save[i].trans.y() = norm(rng);
    data_save[i].trans.z() = norm(rng);
    data_save[i].rot.x()   = norm(rng);
    data_save[i].rot.y()   = norm(rng);
    data_save[i].rot.z()   = norm(rng);
    data_save[i].rot.w()   = norm(rng);
  }
  // save all
  ASSERT_TRUE(io.open(filename, std::ios_base::out));
  ASSERT_TRUE(io.saveAll(data_save.begin(), data_save.end()));
  io.close();
  // load all
  ASSERT_TRUE(io.open(filename, std::ios_base::in));
  ASSERT_TRUE(io.loadAll(std::back_inserter(data_load)));
  io.close();
  // check all
  ASSERT_EQ(data_save.size(), data_load.size());
  for (int i = 0; i < NUM; i++) {
    ASSERT_NEAR(data_save[i].time, data_load[i].time, 1e-7);
    ASSERT_NEAR(data_save[i].index, data_load[i].index, 1e-7);
    ASSERT_NEAR(data_save[i].trans.x(), data_load[i].trans.x(), 1e-7);
    ASSERT_NEAR(data_save[i].trans.y(), data_load[i].trans.y(), 1e-7);
    ASSERT_NEAR(data_save[i].trans.z(), data_load[i].trans.z(), 1e-7);
    ASSERT_NEAR(data_save[i].rot.x(), data_load[i].rot.x(), 1e-7);
    ASSERT_NEAR(data_save[i].rot.y(), data_load[i].rot.y(), 1e-7);
    ASSERT_NEAR(data_save[i].rot.z(), data_load[i].rot.z(), 1e-7);
    ASSERT_NEAR(data_save[i].rot.w(), data_load[i].rot.w(), 1e-7);
  }
  // save once
  ASSERT_TRUE(io.open(filename, std::ios_base::out));
  ASSERT_TRUE(io.saveOnce(data_save[0]));
  io.close();
  // load once
  ASSERT_TRUE(io.open(filename, std::ios_base::in));
  ASSERT_TRUE(io.loadOnce(data_load[0]));
  io.close();
  // check once
  ASSERT_NEAR(data_save[0].time, data_load[0].time, 1e-7);
  ASSERT_NEAR(data_save[0].index, data_load[0].index, 1e-7);
  ASSERT_NEAR(data_save[0].trans.x(), data_load[0].trans.x(), 1e-7);
  ASSERT_NEAR(data_save[0].trans.z(), data_load[0].trans.z(), 1e-7);
  ASSERT_NEAR(data_save[0].trans.y(), data_load[0].trans.y(), 1e-7);
  ASSERT_NEAR(data_save[0].rot.x(), data_load[0].rot.x(), 1e-7);
  ASSERT_NEAR(data_save[0].rot.y(), data_load[0].rot.y(), 1e-7);
  ASSERT_NEAR(data_save[0].rot.z(), data_load[0].rot.z(), 1e-7);
  ASSERT_NEAR(data_save[0].rot.w(), data_load[0].rot.w(), 1e-7);
}
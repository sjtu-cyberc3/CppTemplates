// datatype
#include "common/datatype/gnss.h"
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
TEST(datatype, gnss) {
  std::vector<GNSS>                data_save(NUM);
  std::vector<GNSS>                data_load;
  std::string                      filename = "gnss_test.txt";
  SensorIO<GNSS>                   io;
  std::default_random_engine       rng(SEED);
  std::normal_distribution<double> norm;
  std::uniform_int_distribution    uniform;
  // generate random data
  for (int i = 0; i < NUM; i++) {
    data_save[i].time      = i;
    data_save[i].longitude = norm(rng);
    data_save[i].latitude  = norm(rng);
    data_save[i].altitude  = norm(rng);
    data_save[i].status    = uniform(rng);
    data_save[i].service   = uniform(rng);
    data_save[i].local_E   = norm(rng);
    data_save[i].local_N   = norm(rng);
    data_save[i].local_U   = norm(rng);
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
    ASSERT_NEAR(data_save[i].longitude, data_load[i].longitude, 1e-7);
    ASSERT_NEAR(data_save[i].latitude, data_load[i].latitude, 1e-7);
    ASSERT_NEAR(data_save[i].altitude, data_load[i].altitude, 1e-7);
    ASSERT_EQ(data_save[i].status, data_load[i].status);
    ASSERT_EQ(data_save[i].service, data_load[i].service);
    ASSERT_NEAR(data_save[i].local_E, data_load[i].local_E, 1e-7);
    ASSERT_NEAR(data_save[i].local_N, data_load[i].local_N, 1e-7);
    ASSERT_NEAR(data_save[i].local_U, data_load[i].local_U, 1e-7);
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
  ASSERT_NEAR(data_save[0].longitude, data_load[0].longitude, 1e-7);
  ASSERT_NEAR(data_save[0].latitude, data_load[0].latitude, 1e-7);
  ASSERT_NEAR(data_save[0].altitude, data_load[0].altitude, 1e-7);
  ASSERT_EQ(data_save[0].status, data_load[0].status);
  ASSERT_EQ(data_save[0].service, data_load[0].service);
  ASSERT_NEAR(data_save[0].local_E, data_load[0].local_E, 1e-7);
  ASSERT_NEAR(data_save[0].local_N, data_load[0].local_N, 1e-7);
  ASSERT_NEAR(data_save[0].local_U, data_load[0].local_U, 1e-7);
}

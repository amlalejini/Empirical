#define CATCH_CONFIG_MAIN

#ifndef EMP_TRACK_MEM
#define EMP_TRACK_MEM
#endif

#include "third-party/Catch/single_include/catch.hpp"

#include "data/DataNode.h"
#include "data/DataManager.h"
#include "data/DataInterface.h"
#include "data/DataFile.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

// This function will tell us if the file generated by DataFile is identitcal
// to the expected file.
bool compareFiles(const std::string& p1, const std::string& p2) {
    // From mtrw's answer to https://stackoverflow.com/questions/6163611/compare-two-files
    std::ifstream f1(p1, std::ifstream::binary|std::ifstream::ate);
    std::ifstream f2(p2, std::ifstream::binary|std::ifstream::ate);

    if (f1.fail() || f2.fail()) {
        return false; //file problem
    }

    if (f1.tellg() != f2.tellg()) {
        return false; //size mismatch
    }

    //seek back to beginning and use std::equal to compare contents
    f1.seekg(0, std::ifstream::beg);
    f2.seekg(0, std::ifstream::beg);
    return std::equal(std::istreambuf_iterator<char>(f1.rdbuf()),
                        std::istreambuf_iterator<char>(),
                        std::istreambuf_iterator<char>(f2.rdbuf()));
}

TEST_CASE("Test DataNode", "[data]") {
    // Create a new empty DataNode
    emp::DataNode<int, emp::data::Current, emp::data::Range, emp::data::Pull, emp::data::Log, emp::data::Info> data;
    // Test GetCount function before and after adding to the node
    REQUIRE(data.GetCount()==0);
    data.Add(27, 28, 29);
    REQUIRE(data.GetCount()==3);
    data.Reset();
    REQUIRE(data.GetCount()==0);
    // ResetCount is untracked so should always be 0
    REQUIRE(data.GetResetCount()==0);

    const std::string info = "test data node";
    data.SetDescription(info);
    data.SetName(info);
    data.SetKeyword(info);
    REQUIRE(data.GetDescription()==info);
    REQUIRE(data.GetName()==info);
    REQUIRE(data.GetKeyword()==info);
}

TEST_CASE("Test DataRange", "[data]") {
    emp::DataNode<int, emp::data::Current, emp::data::Range, emp::data::Pull, emp::data::Log> data;

    data.Add(27);
    REQUIRE(data.GetCurrent() == 27);
    REQUIRE(data.GetTotal() == 27);
    REQUIRE(data.GetMean() == 27);
    REQUIRE(data.GetMin() == 27);
    REQUIRE(data.GetMax() == 27);
    REQUIRE(data.GetMedian() == 27);
    REQUIRE(data.GetPercentile(0) == 27);
    REQUIRE(data.GetPercentile(25) == 27);
    REQUIRE(data.GetPercentile(100) == 27);

    data.Add(29,28);

    // std::cout << "=> Added 27, 28, and 29" << std::endl;
    // std::cout << "Current = " << data.GetCurrent() << std::endl;
    // std::cout << "Total   = " << data.GetTotal() << std::endl;
    // std::cout << "Mean    = " << data.GetMean() << std::endl;
    // std::cout << "Min     = " << data.GetMin() << std::endl;
    // std::cout << "Max     = " << data.GetMax() << std::endl;

    REQUIRE(data.GetCurrent() == 28);
    REQUIRE(data.GetTotal() == 84);
    REQUIRE(data.GetMean() == 28);
    REQUIRE(data.GetMin() == 27);
    REQUIRE(data.GetMax() == 29);
    REQUIRE(data.GetMedian() == 28);
    REQUIRE(data.GetPercentile(0) == 27);
    REQUIRE(data.GetPercentile(25) == 27.5);
    REQUIRE(data.GetPercentile(100) == 29);

    data.Add(32);
    // std::cout << "\n=> Added 32" << std::endl;
    // std::cout << "Current = " << data.GetCurrent() << std::endl;
    // std::cout << "Total   = " << data.GetTotal() << std::endl;
    // std::cout << "Mean    = " << data.GetMean() << std::endl;
    // std::cout << "Min     = " << data.GetMin() << std::endl;
    // std::cout << "Max     = " << data.GetMax() << std::endl;

    REQUIRE(data.GetCurrent() == 32);
    REQUIRE(data.GetTotal() == 116);
    REQUIRE(data.GetMean() == 29);
    REQUIRE(data.GetMin() == 27);
    REQUIRE(data.GetMax() == 32);
    REQUIRE(data.GetMedian() == 28.5);
    REQUIRE(data.GetPercentile(0) == 27);
    REQUIRE(data.GetPercentile(25) == 27.75);
    REQUIRE(data.GetPercentile(100) == 32);


    data.Reset();
    // std::cout << "\n=> Reset!" << std::endl;
    // std::cout << "Current = " << data.GetCurrent() << std::endl;
    // std::cout << "Total   = " << data.GetTotal() << std::endl;
    // std::cout << "Mean    = " << data.GetMean() << std::endl;
    // std::cout << "Min     = " << data.GetMin() << std::endl;
    // std::cout << "Max     = " << data.GetMax() << std::endl;

    REQUIRE(data.GetCurrent() == 32);
    REQUIRE(data.GetTotal() == 0);
    REQUIRE(std::isnan(data.GetMean()));
    REQUIRE(data.GetMin() == 0);
    REQUIRE(data.GetMax() == 0);
    REQUIRE(std::isnan(data.GetMedian()));
    REQUIRE(std::isnan(data.GetPercentile(0)));
    REQUIRE(std::isnan(data.GetPercentile(25)));
    REQUIRE(std::isnan(data.GetPercentile(100)));

    data.Add(100,200,300,400,500);
    // std::cout << "\nAdded 100,200,300,400,500" << std::endl;
    // std::cout << "Current = " << data.GetCurrent() << std::endl;
    // std::cout << "Total   = " << data.GetTotal() << std::endl;
    // std::cout << "Mean    = " << data.GetMean() << std::endl;
    // std::cout << "Min     = " << data.GetMin() << std::endl;
    // std::cout << "Max     = " << data.GetMax() << std::endl;

    REQUIRE(data.GetCurrent() == 500);
    REQUIRE(data.GetTotal() == 1500);
    REQUIRE(data.GetMean() == 300);
    REQUIRE(data.GetMin() == 100);
    REQUIRE(data.GetMax() == 500);
    REQUIRE(data.GetMedian() == 300);
    REQUIRE(data.GetPercentile(0) == 100);
    REQUIRE(data.GetPercentile(25) == 200);
    REQUIRE(data.GetPercentile(100) == 500);

    data.AddPull([](){return -800;});
    data.PullData();
    // std::cout << "\nAdded -800 via PullData()" << std::endl;
    // std::cout << "Current = " << data.GetCurrent() << std::endl;
    // std::cout << "Total   = " << data.GetTotal() << std::endl;
    // std::cout << "Mean    = " << data.GetMean() << std::endl;
    // std::cout << "Min     = " << data.GetMin() << std::endl;
    // std::cout << "Max     = " << data.GetMax() << std::endl;

    REQUIRE(data.GetCurrent() == -800);
    REQUIRE(data.GetTotal() == 700);
    REQUIRE(data.GetMean() == Approx(116.6667));
    REQUIRE(data.GetMin() == -800);
    REQUIRE(data.GetMax() == 500);
    REQUIRE(data.GetMedian() == 250);
    REQUIRE(data.GetPercentile(0) == -800);
    REQUIRE(data.GetPercentile(25) == 125);
    REQUIRE(data.GetPercentile(100) == 500);

    data.AddPullSet([](){return emp::vector<int>({1600,0,0});});
    data.PullData(); // Remember that this also runs the function that returns -800
    // std::cout << "\nAdded another -800, a 1600 and two 0's via PullData()" << std::endl;
    // std::cout << "Current = " << data.GetCurrent() << std::endl;
    // std::cout << "Total   = " << data.GetTotal() << std::endl;
    // std::cout << "Mean    = " << data.GetMean() << std::endl;
    // std::cout << "Min     = " << data.GetMin() << std::endl;
    // std::cout << "Max     = " << data.GetMax() << std::endl;

    REQUIRE(data.GetCurrent() == 0);
    REQUIRE(data.GetTotal() == 1500);
    REQUIRE(data.GetMean() == 150);
    REQUIRE(data.GetMin() == -800);
    REQUIRE(data.GetMax() == 1600);
    REQUIRE(data.GetMedian() == 150);
    REQUIRE(data.GetPercentile(0) == -800);
    REQUIRE(data.GetPercentile(25) == 0);
    REQUIRE(data.GetPercentile(100) == 1600);

    // std::cout << std::endl;
    // data.PrintDebug();

    // std::cout << std::endl;

}

TEST_CASE("Test DataArchive", "[data]") {
    emp::DataNode<int, emp::data::Archive, emp::data::Current, emp::data::FullRange, emp::data::Info> data2;

    // data2.PrintDebug();

    data2.Add(1,2,3,4,5,6,7,9,8);
    // std::cout << "\nSetup data2 with values 1 through 9" << std::endl;
    // std::cout << "Current = " << data2.GetCurrent() << std::endl;
    // std::cout << "Total   = " << data2.GetTotal() << std::endl;
    // std::cout << "Mean    = " << data2.GetMean() << std::endl;
    // std::cout << "Min     = " << data2.GetMin() << std::endl;
    // std::cout << "Max     = " << data2.GetMax() << std::endl;

    REQUIRE(data2.GetCurrent() == 8);
    REQUIRE(data2.GetTotal() == 45);
    REQUIRE(data2.GetMean() == 5);
    REQUIRE(data2.GetMin() == 1);
    REQUIRE(data2.GetMax() == 9);
    REQUIRE(data2.GetData() == emp::vector<int>({1,2,3,4,5,6,7,9,8}));

    data2.SetInfo("Best Data", "This is the best of all possible data.", "best_data");

    // Test reset
    data2.Reset();

    // Generate what the archive should look like now that we Reset
    emp::vector<emp::vector<int>> arch_comp;
    arch_comp.push_back(emp::vector<int>({1,2,3,4,5,6,7,9,8}));

    // Compare archives as strings for easy reading of errors.
    REQUIRE(emp::to_string(data2.GetArchive()) == emp::to_string(arch_comp));

    data2.Add(4);
    arch_comp.push_back(emp::vector<int>({4}));

    REQUIRE(data2.GetCurrent() == 4);
    REQUIRE(data2.GetTotal() == 4);
    REQUIRE(data2.GetMean() == 4);
    REQUIRE(data2.GetMin() == 4);
    REQUIRE(data2.GetMax() == 4);
    REQUIRE(data2.GetData() == emp::vector<int>({4}));
    REQUIRE(data2.GetData(0) == emp::vector<int>({1,2,3,4,5,6,7,9,8}));
    REQUIRE(data2.GetResetCount() == 1);


    data2.Reset();
    REQUIRE(data2.GetArchive() == arch_comp);

    // Test that setting info worked
    REQUIRE(data2.GetName() == "Best Data");
    REQUIRE(data2.GetDescription() == "This is the best of all possible data.");
    REQUIRE(data2.GetKeyword() == "best_data");

}

TEST_CASE("Test DataStats", "[data]") {
    emp::DataNode<double, emp::data::Current, emp::data::Range, emp::data::Stats, emp::data::Log> data3;

    // std::cout << "\nSetup data3 with values 1 through 8 with an extra 8" << std::endl;

    data3.Add(1,2,3,4,5,6,7,8,8);
    // std::cout << "Current = " << data3.GetCurrent() << std::endl;
    // std::cout << "Total   = " << data3.GetTotal() << std::endl;
    // std::cout << "Mean    = " << data3.GetMean() << std::endl;
    // std::cout << "Min     = " << data3.GetMin() << std::endl;
    // std::cout << "Max     = " << data3.GetMax() << std::endl;
    // std::cout << "Variance= " << data3.GetVariance() << std::endl;
    // std::cout << "Std Dev = " << data3.GetStandardDeviation() << std::endl;
    // std::cout << "Skew    = " << data3.GetSkew() << std::endl;
    // std::cout << "Kurtosis= " << data3.GetKurtosis() << std::endl;

    REQUIRE(data3.GetCurrent() == 8);
    REQUIRE(data3.GetTotal() == 44);
    REQUIRE(data3.GetMean() == Approx(4.8888888888889));
    REQUIRE(data3.GetMin() == 1);
    REQUIRE(data3.GetMax() == 8);
    REQUIRE(data3.GetVariance() == Approx(5.87654));
    REQUIRE(data3.GetStandardDeviation() == Approx(2.42416));
    REQUIRE(data3.GetSkew() == Approx(-0.151045));
    REQUIRE(data3.GetKurtosis() == Approx(-1.3253830944));
}

TEST_CASE("Test DataEmpty", "[data]") {
    emp::DataNode<double> data_empty; // Build an empty DataNode to make sure no modules are required.
}

TEST_CASE("Test DataManager", "[data]") {
    emp::DataManager<double, emp::data::Current, emp::data::Range, emp::data::Pull, emp::data::Log> dataM;

    dataM.New("fitness");
    dataM.New("merit");
    dataM.New("fidelity");

    dataM.AddData("fitness", 3.0);
    dataM.Get("fitness").Add(6.5);
    auto & fit_node = dataM.Get("fitness");
    fit_node.Add(10.0);

    auto & merit_node = dataM.Get("merit");
    merit_node.Add(300, 650, 500);

    dataM.AddData("fidelity", 1.0, 0.8, 0.9);

    // std::cout << std::endl;

    // for (auto & x : dataM.GetNodes()) {
    //     auto & node = *(x.second);
    //     std::cout << x.first
    //             << " cur=" << node.GetCurrent()
    //             << " ave=" << node.GetMean()
    //             << " min=" << node.GetMin()
    //             << " max=" << node.GetMax()
    //             << " tot=" << node.GetTotal()
    //             << std::endl;
    // }

    REQUIRE(dataM.Get("fitness").GetCurrent() == 10);
    REQUIRE(dataM.Get("fitness").GetMean() == 6.5);
    REQUIRE(dataM.Get("fitness").GetMin() == 3);
    REQUIRE(dataM.Get("fitness").GetMax() == 10);
    REQUIRE(dataM.Get("fitness").GetTotal() == 19.5);

    REQUIRE(dataM.Get("fidelity").GetCurrent() == .9);
    REQUIRE(dataM.Get("fidelity").GetMean() == .9);
    REQUIRE(dataM.Get("fidelity").GetMin() == .8);
    REQUIRE(dataM.Get("fidelity").GetMax() == 1);
    REQUIRE(dataM.Get("fidelity").GetTotal() == 2.7);

}

TEST_CASE("Test DataInterface", "[data]") {
    auto * di = emp::MakeDataInterface<double, emp::data::Current, emp::data::Range, emp::data::Pull, emp::data::Log>();
    delete di;

    emp::DataNode<double, emp::data::Current, emp::data::Range> node;
    node.Add(5.5, .6); // Put in some test data, since we can't add through the interface

    auto * di2(&node);
    REQUIRE(di2->GetTotal() == 6.1);
    REQUIRE(di2->GetMin() == .6);
    REQUIRE(di2->GetMax() == 5.5);
    REQUIRE(di2->GetMean() == 3.05);

    emp::DataNode<double, emp::data::Current, emp::data::Range, emp::data::Stats> node2;
    node2.Add(5.5, .6); // Put in some test data, since we can't add through the interface

    auto * di3(&node2);

    REQUIRE(di3->GetTotal() == 6.1);
    REQUIRE(di3->GetMin() == .6);
    REQUIRE(di3->GetMax() == 5.5);
    REQUIRE(di3->GetMean() == 3.05);
    REQUIRE(di3->GetVariance() == Approx(6.0025));
    REQUIRE(di3->GetStandardDeviation() == Approx(2.45));
    REQUIRE(di3->GetSkew() == 0);
    REQUIRE(di3->GetKurtosis() == -2);
}

int test_fun() {
  static int val = 10;
  return val += 3;
}

TEST_CASE("Test DataFile", "[data]") {
    int test_int = 5;

    emp::DataFile dfile("test_file.dat");

    REQUIRE(dfile.GetFilename() == "test_file.dat");
    
    emp::DataMonitor<double> data_fracs;
    emp::DataMonitor<int> data_squares;
    emp::DataMonitor<uint64_t> data_cubes;

    dfile.AddCurrent(data_fracs);
    dfile.AddCurrent(data_squares);
    dfile.AddCurrent(data_cubes);
    dfile.AddMean(data_cubes);
    dfile.AddTotal(data_cubes);
    dfile.AddMin(data_cubes);
    dfile.AddMax(data_cubes);
    dfile.AddFun<int>(test_fun);
    dfile.AddVar<int>(test_int);

    double frac = 0.0;
    for (size_t i = 0; i < 10; i++) {
        test_int += i;
        data_fracs.Add(frac += 0.01);
        data_squares.Add((int)(i*i));
        data_cubes.Add(i*i*i);
        dfile.Update();

        // std::cout << i << std::endl;
    }

    dfile.SetupLine("[[",":", "]]\n");
    for (size_t i = 10; i < 20; i++) {
        data_fracs.Add(frac += 0.01);
        data_squares.Add((int)(i*i));
        data_cubes.Add(i*i*i);
        dfile.Update();

        // std::cout << i << std::endl;
    }

    REQUIRE(compareFiles("test_file.dat", "data/test_file.dat"));
}

TEST_CASE("Test histogram", "[data]") {
    emp::DataNode<double, emp::data::Current, emp::data::Range, emp::data::Histogram, emp::data::Pull, emp::data::Log> data;
    data.SetupBins(1,21,10);
    data.Add(1,2,1,19);

    REQUIRE(data.GetHistMin() == 1);
    REQUIRE(data.GetHistWidth(5) == 2);

    REQUIRE(data.GetBinMins() == emp::vector<double>({1,3,5,7,9,11,13,15,17,19}));

    REQUIRE(data.GetHistCount(9) == 1);
    REQUIRE(data.GetHistCounts() == emp::vector<size_t>({3,0,0,0,0,0,0,0,0,1}));

    data.Reset();
    REQUIRE(data.GetHistCounts() == emp::vector<size_t>({0,0,0,0,0,0,0,0,0,0}));
}

TEST_CASE("Test Container DataFile", "[data]") {

    emp::vector<int> cool_data({1,2,3});
    std::function<emp::vector<int>(void)> get_data = [&cool_data](){return cool_data;};
    emp::ContainerDataFile<emp::vector<int>> dfile("test_container_file.dat");

    dfile.SetUpdateContainerFun(get_data);

    std::function<int(int)> return_val = [](int i){return i;};
    std::function<int(int)> square_val = [](int i){return i*i;};

    dfile.AddContainerFun(return_val, "value", "value");
    dfile.AddContainerFun(square_val, "squared", "value squared");

    dfile.PrintHeaderKeys();
    dfile.Update();
    cool_data.push_back(5);
    dfile.Update();

    // Since update is virtual, this should work on a pointer to a base datafile
    emp::DataFile * data_ptr = & dfile;

    cool_data.push_back(6);
    data_ptr->Update();

    dfile.SetTimingRepeat(2);
    cool_data.clear();
    cool_data.push_back(7);
    cool_data.push_back(3);
    dfile.Update(2);
    dfile.Update(3);
    cool_data.push_back(2);
    data_ptr->Update(4);
    data_ptr->Update(5);

    REQUIRE(compareFiles("test_container_file.dat", "data/test_container_file.dat"));

    auto dfile2 = emp::MakeContainerDataFile(get_data, "test_make_container_file.dat");
    dfile2.AddContainerFun(return_val, "value", "value");
    dfile2.AddContainerFun(square_val, "squared", "value squared");

    dfile2.PrintHeaderKeys();
    dfile2.Update();

    REQUIRE(compareFiles("test_make_container_file.dat", "data/test_make_container_file.dat"));
}

TEST_CASE("Test timing", "[data]") {
    int test_int = 5;

    emp::DataFile dfile("test_timing_file.dat");

    emp::DataMonitor<double> data_fracs;
    emp::DataMonitor<int> data_squares;
    emp::DataMonitor<uint64_t> data_cubes;

    dfile.AddVar<int>(test_int);
    dfile.AddCurrent(data_fracs);
    dfile.AddCurrent(data_squares);
    dfile.AddCurrent(data_cubes);
    dfile.AddMean(data_cubes);
    dfile.AddTotal(data_cubes);
    dfile.AddMin(data_cubes);
    dfile.AddMax(data_cubes);
    dfile.AddFun<int>(test_fun);

    double frac = 0.0;

    dfile.SetTimingRepeat(2);

    for (size_t i = 0; i < 10; i++) {
        test_int = i;
        data_fracs.Add(frac += 0.01);
        data_squares.Add((int)(i*i));
        data_cubes.Add(i*i*i);
        dfile.Update(i);

        // std::cout << i << std::endl;
    }

    dfile.SetTimingOnce(5);

    for (size_t i = 0; i < 10; i++) {
        test_int = i;
        data_fracs.Add(frac += 0.01);
        data_squares.Add((int)(i*i));
        data_cubes.Add(i*i*i);
        dfile.Update(i);
        // std::cout << i << std::endl;
    }

    dfile.SetTimingRange(2, 3, 9);

    for (size_t i = 0; i < 10; i++) {
        test_int = i;
        data_fracs.Add(frac += 0.01);
        data_squares.Add((int)(i*i));
        data_cubes.Add(i*i*i);
        dfile.Update(i);

        // std::cout << i << std::endl;
    }

    dfile.SetTiming([](size_t ud){return (bool)floor(sqrt((double)ud) == ceil(sqrt((double)ud)));});

    for (size_t i = 0; i < 10; i++) {
        test_int = i;
        data_fracs.Add(frac += 0.01);
        data_squares.Add((int)(i*i));
        data_cubes.Add(i*i*i);
        dfile.Update(i);

        // std::cout << i << std::endl;
    }

    REQUIRE(compareFiles("test_timing_file.dat", "data/test_timing_file.dat"));
}

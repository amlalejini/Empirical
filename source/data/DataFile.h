/**
 *  @note This file is part of Empirical, https://github.com/devosoft/Empirical
 *  @copyright Copyright (C) Michigan State University, MIT Software license; see doc/LICENSE.md
 *  @date 2016-2018
 *
 *  @file  DataFile.h
 *  @brief DataFile objects use DataNode objects to dynamically fill out file contents.
 */

#ifndef EMP_DATA_FILE_H
#define EMP_DATA_FILE_H

#include <fstream>
#include <functional>
#include <iostream>
#include <string>

#include "../base/assert.h"
#include "../base/vector.h"
#include "../meta/type_traits.h"
#include "../tools/FunctionSet.h"
#include "../tools/string_utils.h"

#include "DataNode.h"

namespace emp {

  /** This class keeps track of everything associated with periodically printing data to a file.
   * It maintains a set of functions for calculating the desired measurements at each point in
   * time that they are required. It also handles the formating of the file.
   */
  class DataFile {
  protected:
    using fun_t = void(std::ostream &);

    std::ostream * os;
    FunctionSet<fun_t> funs;
    emp::vector<std::string> keys;
    emp::vector<std::string> descs;

    std::string line_begin;   ///< What should we print at the start of each line?
    std::string line_spacer;  ///< What should we print between entries?
    std::string line_end;     ///< What should we print at the end of each line?

  public:
    DataFile(const std::string & filename,
             const std::string & b="", const std::string & s=", ", const std::string & e="\n")
      : os(new std::ofstream(filename)), funs(), keys(), descs()
      , line_begin(b), line_spacer(s), line_end(e) { ; }
    DataFile(std::ostream & in_os,
             const std::string & b="", const std::string & s=", ", const std::string & e="\n")
      : os(&in_os), funs(), keys(), descs(), line_begin(b), line_spacer(s), line_end(e) { ; }
    DataFile(const DataFile &) = default;
    DataFile(DataFile &&) = default;
    ~DataFile() { os->flush(); }

    DataFile & operator=(const DataFile &) = default;
    DataFile & operator=(DataFile &&) = default;

    /// Returns the string that is printed at the beginning of each line.
    const std::string & GetLineBegin() const { return line_begin; }
    /// Returns the string that is printed between elements on each line (i.e. the delimeter).
    const std::string & GetSpacer() const { return line_spacer; }
    /// Returns the string that is printed at the end of each line.
    const std::string & GetLineEnd() const { return line_end; }

    /// Print @param _in at the beginning of each line.
    void SetLineBegin(const std::string & _in) { line_begin = _in; }
    /// Print @param _in between elements (i.e. make @param _in the delimeter).
    void SetSpacer(const std::string & _in) { line_spacer = _in; }
    /// Print @param _in at the end of each line.
    void SetLineEnd(const std::string & _in) { line_end = _in; }
    /// Set line begin character (@param b), column delimeter (@param s), and line end character (@param e)
    void SetupLine(const std::string & b, const std::string & s, const std::string & e) {
      line_begin = b;
      line_spacer = s;
      line_end = e;
    }

    /// Print a header containing the name of each column
    void PrintHeaderKeys() {
      *os << line_begin;
      for (size_t i = 0; i < keys.size(); i++) {
        if (i > 0) *os << line_spacer;
        *os << keys[i];
      }
      *os << line_end;
      os->flush();
    }

    /// Print a header containing comments describing all of the columns
    void PrintHeaderComment(const std::string & cstart = "# ") {
      for (size_t i = 0; i < keys.size(); i++) {
        *os << cstart << i << ": " << descs[i] << " (" << keys[i] << ")" << std::endl;
      }
      os->flush();
    }

    /// Run all of the functions and print the results as a new line in the file
    void Update() {
      *os << line_begin;
      for (size_t i = 0; i < funs.size(); i++) {
        if (i > 0) *os << line_spacer;
        funs[i](*os);
      }
      *os << line_end;
      os->flush();
    }

    /// If a function takes an ostream, pass in the correct one.
    /// Generic function for adding a column to the DataFile. In practice, you probably
    /// want to call one of the more specific ones.
    size_t Add(const std::function<void(std::ostream &)> & fun, const std::string & key, const std::string & desc) {
      size_t id = funs.GetSize();
      funs.Add(fun);
      keys.emplace_back(key);
      descs.emplace_back(desc);
      return id;
    }

    /// Add a function that returns a value to be printed to the file.
    template <typename T>
    size_t AddFun(const std::function<T()> & fun, const std::string & key="", const std::string & desc="") {
      std::function<fun_t> in_fun = [fun](std::ostream & os){ os << fun(); };
      return Add(in_fun, key, desc);
    }

    /// Add a function that always prints the current value of @param var.
    template <typename T>
    size_t AddVar(const T & var, const std::string & key="", const std::string & desc="") {
      std::function<fun_t> in_fun = [&var](std::ostream & os){ os << var; };
      return Add(in_fun, key, desc);
    }

    /// Add a function that always pulls the current value from the DataNode @param node.
    /// Requires that @param node have the data::Current modifier.
    template <typename VAL_TYPE, emp::data... MODS>
    size_t AddCurrent(DataNode<VAL_TYPE, MODS...> & node, const std::string & key="", const std::string & desc="") {
      std::function<fun_t> in_fun = [&node](std::ostream & os){ os << node.GetCurrent(); };
      return Add(in_fun, key, desc);
    }

    /// Add a function that always pulls the mean value from the DataNode @param node.
    /// Requires that @param node have the data::Range or data::FullRange modifier.
    template <typename VAL_TYPE, emp::data... MODS>
    size_t AddMean(DataNode<VAL_TYPE, MODS...> & node, const std::string & key="", const std::string & desc="") {
      std::function<fun_t> in_fun = [&node](std::ostream & os){ os << node.GetMean(); };
      return Add(in_fun, key, desc);
    }

    /// Add a function that always pulls the total value from the DataNode @param node.
    /// Requires that @param node have the data::Range or data::FullRange modifier.
    template <typename VAL_TYPE, emp::data... MODS>
    size_t AddTotal(DataNode<VAL_TYPE, MODS...> & node, const std::string & key="", const std::string & desc="") {
      std::function<fun_t> in_fun = [&node](std::ostream & os){ os << node.GetTotal(); };
      return Add(in_fun, key, desc);
    }

    /// Add a function that always pulls the minimum value from the DataNode @param node
    /// Requires that @param node have the data::Range or data::FullRange modifier.
    template <typename VAL_TYPE, emp::data... MODS>
    size_t AddMin(DataNode<VAL_TYPE, MODS...> & node, const std::string & key="", const std::string & desc="") {
      std::function<fun_t> in_fun = [&node](std::ostream & os){ os << node.GetMin(); };
      return Add(in_fun, key, desc);
    }

    /// Add a function that always pulls the maximum value from the DataNode @param node
    /// Requires that @param node have the data::Range or data::FullRange modifier.
    template <typename VAL_TYPE, emp::data... MODS>
    size_t AddMax(DataNode<VAL_TYPE, MODS...> & node, const std::string & key="", const std::string & desc="") {
      std::function<fun_t> in_fun = [&node](std::ostream & os){ os << node.GetMax(); };
      return Add(in_fun, key, desc);
    }

    /// Add a function that always pulls the variance from the DataNode @param node
    /// Requires that @param node have the data::Stats or data::FullStats modifier.
    template <typename VAL_TYPE, emp::data... MODS>
    size_t AddVariance(DataNode<VAL_TYPE, MODS...> & node, const std::string & key="", const std::string & desc="") {
      std::function<fun_t> in_fun = [&node](std::ostream & os){ os << node.GetVariance(); };
      return Add(in_fun, key, desc);
    }

    /// Add a function that always pulls the standard deviation from the DataNode @param node
    /// Requires that @param node have the data::Stats or data::FullStats modifier.
    template <typename VAL_TYPE, emp::data... MODS>
    size_t AddStandardDeviation(DataNode<VAL_TYPE, MODS...> & node, const std::string & key="", const std::string & desc="") {
      std::function<fun_t> in_fun = [&node](std::ostream & os){ os << node.GetStandardDeviation(); };
      return Add(in_fun, key, desc);
    }

    /// Add a function that always pulls the skewness from the DataNode @param node
    /// Requires that @param node have the data::Stats or data::FullStats modifier.
    template <typename VAL_TYPE, emp::data... MODS>
    size_t AddSkew(DataNode<VAL_TYPE, MODS...> & node, const std::string & key="", const std::string & desc="") {
      std::function<fun_t> in_fun = [&node](std::ostream & os){ os << node.GetSkew(); };
      return Add(in_fun, key, desc);
    }

    /// Add a function that always pulls the kurtosis from the DataNode @param node
    /// Requires that @param node have the data::Stats or data::FullStats modifier.
    template <typename VAL_TYPE, emp::data... MODS>
    size_t AddKurtosis(DataNode<VAL_TYPE, MODS...> & node, const std::string & key="", const std::string & desc="") {
      std::function<fun_t> in_fun = [&node](std::ostream & os){ os << node.GetKurtosis(); };
      return Add(in_fun, key, desc);
    }

    /// Add multiple functions that pull all stats measurements from the DataNode @param node
    /// Requires that @param node have the data::Stats or data::FullStats modifier.
    /// @param key and @param desc will have the name of the stat appended to the beginning.
    /// Note: excludes standard deviation, because it is easily calculated from variance
    template <typename VAL_TYPE, emp::data... MODS>
    void AddStats(DataNode<VAL_TYPE, MODS...> & node, const std::string & key="", const std::string & desc="") {
      AddMean(node, "mean_" + key, "mean of " + desc);
      AddMin(node, "min_" + key, "min of " + desc);
      AddMax(node, "max_" + key, "max of " + desc);
      AddVariance(node, "variance_" + key, "variance of " + desc);
      AddSkew(node, "skew_" + key, "skew of " + desc);
      AddKurtosis(node, "kurtosis_" + key, "kurtosis of " + desc);
    }

    /// Add a function that always pulls the count of the @param bin_id 'th bin of the histogram
    /// from @param node. Requires that @param node have the data::Histogram modifier and at least
    /// @bin_id bins.
    template <typename VAL_TYPE, emp::data... MODS>
    size_t AddHistBin(DataNode<VAL_TYPE, MODS...> & node, size_t bin_id, const std::string & key="", const std::string & desc="") {
      std::function<fun_t> in_fun =
        [&node,bin_id](std::ostream & os){ os << node.GetHistCount(bin_id); };
      return Add(in_fun, key, desc);
    }

    /// Add a function that always pulls the inferiority (mean divided by max) from the DataNode @param node.
    /// Requires that @param node have the data::Range or data::FullRange modifier.
    template <typename VAL_TYPE, emp::data... MODS>
    size_t AddInferiority(DataNode<VAL_TYPE, MODS...> & node, const std::string & key="", const std::string & desc="") {
      std::function<fun_t> in_fun = [&node](std::ostream & os){
        VAL_TYPE inf = (node.GetMax() == 0) ? 0 : (node.GetMean() / node.GetMax());
        os << inf;
      };
      return Add(in_fun, key, desc);
    }
  };


  template <typename container_t>
  class CollectionDataFile;

  namespace internal {

    template <typename container_t>
    struct update_impl {
      void Update(CollectionDataFile<container_t> * df) {
        using data_t = typename container_t::value_type;
        for (const data_t & d : df->GetCurrentRows()) {
          df->OutputLine(d);
        }
      }
    };

    template <typename container_t>
    struct update_impl<Ptr<container_t>> {
      void Update(CollectionDataFile<Ptr<container_t>> * df) {
        using data_t = typename remove_ptr_type<container_t>::type::value_type;

        for (const data_t & d : *(df->GetCurrentRows())) {
          df->OutputLine(d);
        }
      }
    };

    template <typename container_t>
    struct update_impl<container_t*> {
      void Update(CollectionDataFile<container_t*> * df) {
        using data_t = typename remove_ptr_type<container_t>::type::value_type;

        for (const data_t & d : *(df->GetCurrentRows())) {
          df->OutputLine(d);
        }
      }
    };

  }


  template <typename CONTAINER>
  class CollectionDataFile : public DataFile {
  private:
    using container_t = typename std::remove_reference<CONTAINER>::type;
    using raw_container_t = typename remove_ptr_type<container_t>::type;
    // using non_const_container_t = typename std::remove_const<raw_container_t>::type;
    // using data_t = typename non_const_container_t::value_type;
    using data_t = typename raw_container_t::value_type;
    using coll_fun_t = void(std::ostream &, data_t);
    using fun_update_container_t = std::function<container_t(void)>;

    // std::cout << typeid(container_t).name() << " " << typeid(raw_container_t).name() << " "

    fun_update_container_t update_container_fun;

    container_t current_rows;
    FunctionSet<coll_fun_t> collection_funs;
    emp::vector<std::string> collection_keys;
    emp::vector<std::string> collection_descs;

  public:

    CollectionDataFile(const std::string & filename,
             const std::string & b="", const std::string & s=", ", const std::string & e="\n")
             : DataFile(filename, b, s, e), update_container_fun(), current_rows() {;}

    void SetUpdateContainerFun(const fun_update_container_t fun) {
      update_container_fun = fun;
    }

    /// Print a header containing the name of each column
    void PrintHeaderKeys() {
      *os << line_begin;
      for (size_t i = 0; i < keys.size(); i++) {
        if (i > 0) *os << line_spacer;
        *os << keys[i];
      }
      for (size_t i = 0; i < collection_keys.size(); i++) {
        if (i > 0 || keys.size() > 0) *os << line_spacer;
        *os << collection_keys[i];
      }
      *os << line_end;
      os->flush();
    }

    /// Print a header containing comments describing all of the columns
    void PrintHeaderComment(const std::string & cstart = "# ") {
      for (size_t i = 0; i < keys.size(); i++) {
        *os << cstart << i << ": " << descs[i] << " (" << keys[i] << ")" << std::endl;
      }
      for (size_t i = 0; i < collection_keys.size(); i++) {
        *os << cstart << i+keys.size() << ": " << collection_descs[i] << " (" << collection_keys[i] << ")" << std::endl;
      }

      os->flush();
    }

    const container_t GetCurrentRows() const {return current_rows;}

    void OutputLine(const data_t d) {
      *os << line_begin;
        for (size_t i = 0; i < funs.size(); i++) {
          if (i > 0) *os << line_spacer;
          funs[i](*os);
        }

        for (size_t i = 0; i < collection_funs.size(); i++) {
          if (i > 0 || keys.size() > 0) *os << line_spacer;
          collection_funs[i](*os, d);
        }
      *os << line_end;
    }

    /// Run all of the functions and print the results as a new line in the file
    void Update(){
      emp_assert(update_container_fun);
      current_rows = update_container_fun();
    // std::cout << "curr: " << to_string(current_rows) << std::endl;
      // if (emp::is_ptr_type<container_t>::value) {
      //   for (const data_t & d : *current_rows) {
      //     *os << line_begin;
      //     for (size_t i = 0; i < funs.size(); i++) {
      //       if (i > 0) *os << line_spacer;
      //       funs[i](*os);
      //     }

      //     for (size_t i = 0; i < collection_funs.size(); i++) {
      //       if (i > 0 || keys.size() > 0) *os << line_spacer;
      //       collection_funs[i](*os, d);
      //     }
      //   *os << line_end;
      //   }
      // } else {
      //   for (const data_t & d : current_rows) {
      //     *os << line_begin;
      //     for (size_t i = 0; i < funs.size(); i++) {
      //       if (i > 0) *os << line_spacer;
      //       funs[i](*os);
      //     }

      //     for (size_t i = 0; i < collection_funs.size(); i++) {
      //       if (i > 0 || keys.size() > 0) *os << line_spacer;
      //       collection_funs[i](*os, d);
      //     }
      //   *os << line_end;
      //   }
      // }
      // os->flush();

      internal::update_impl<container_t>().Update(this);
      os->flush();
    }

    /// If a function takes an ostream, pass in the correct one.
    /// Generic function for adding a column to the DataFile. In practice, you probably
    /// want to call one of the more specific ones.
    size_t AddCollection(const std::function<void(std::ostream &, data_t)> & fun, const std::string & key, const std::string & desc) {
      size_t id = collection_funs.GetSize();
      collection_funs.Add(fun);
      collection_keys.emplace_back(key);
      collection_descs.emplace_back(desc);
      return id;
    }

    /// Add a function that returns a value to be printed to the file.
    template <typename T>
    size_t AddCollectionFun(const std::function<T(const data_t)> & fun, const std::string & key="", const std::string & desc="") {
      std::function<coll_fun_t> in_fun = [fun](std::ostream & os, const data_t data){ os << fun(data); };
      return AddCollection(in_fun, key, desc);
    }


  };


}

#endif

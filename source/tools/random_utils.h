/**
 *  @note This file is part of Empirical, https://github.com/devosoft/Empirical
 *  @copyright Copyright (C) Michigan State University, MIT Software license; see doc/LICENSE.md
 *  @date 2016-2017
 *
 *  @file  random_utils.h
 *  @brief Helper functions for emp::Random for common random tasks.
 *  @note Status: RELEASE
 */

#ifndef EMP_RANDOM_UTILS_H
#define EMP_RANDOM_UTILS_H

#include <unordered_set>
#include "../base/vector.h"
#include "BitVector.h"
#include "Random.h"
#include "math.h"
#include "set_utils.h"

namespace emp {

  /// Randomly reorder all of the elements in a vector.
  /// If max_count is provided, just make sure that the first max_count entries are randomly
  /// drawn from entire vector.

  template <typename T>
  inline void Shuffle(Random & random, emp::vector<T> & v, size_t max_count)
  {
    for (size_t i = 0; i < max_count; i++) {
      const size_t pos = random.GetUInt(i, v.size());
      if (pos == i) continue;
      std::swap(v[i], v[pos]);
    }
  }

  template <typename T>
  inline void Shuffle(Random & random, emp::vector<T> & v) { Shuffle(random, v, v.size()); }


  /// Return an emp::vector<int> numbered 0 through size-1 in a random order.

  inline emp::vector<size_t> GetPermutation(Random & random, size_t size) {
    emp::vector<size_t> seq(size);
    seq[0] = 0;
    for (size_t i = 1; i < size; i++) {
      size_t val_pos = random.GetUInt(i+1);
      seq[i] = seq[val_pos];
      seq[val_pos] = i;
    }
    return seq;
  }

  /// Choose K positions from N possibilities.

  inline void Choose(Random & random, size_t N, size_t K, std::vector<size_t> & choices) {
    emp_assert (N >= K);

    choices.resize(K);
    while (K) {
      if (N==K || random.P(((double) K)/((double) N))) { choices[--K] = --N; }
      else --N;
    }
  }

  inline std::vector<size_t> Choose(Random & random, size_t N, size_t K) {
    std::vector<size_t> choices;
    Choose(random,N,K,choices);
    return choices;
  }


  /// Generate a random BitVector of the specified size.
  inline BitVector RandomBitVector(Random & random, size_t size, double p=0.5)
  {
    emp_assert(p >= 0.0 && p <= 1.0);
    BitVector bits(size);
    for (size_t i = 0; i < size; i++) bits[i] = random.P(p);
    return bits;
  }

  /// Generate a random double vector in the specified range.
  inline emp::vector<double> RandomDoubleVector(Random & random, size_t size, double min, double max) {
    emp::vector<double> vals(size);
    for (double & v : vals) v = random.GetDouble(min, max);
    return vals;
  }

  /// Generate a random size_t vector in the specified range.
  inline emp::vector<size_t> RandomUIntVector(Random & random, size_t size, size_t min, size_t max) {
    emp::vector<size_t> vals(size);
    for (size_t & v : vals) v = random.GetUInt(min, max);
    return vals;
  }

  /// Generate a random vector in the specified type and range.
  template <typename T>
  inline emp::vector<T> RandomVector(Random & random, size_t size, T min, T max) {
    emp::vector<T> vals(size);
    for (T & v : vals) v = (T) random.GetDouble((double) min, (double) max);
    return vals;
  }

  /// Generate a random BitVector of the specified size.
  inline void RandomizeBitVector(BitVector & bits, Random & random, double p=0.5)
  {
    emp_assert(p >= 0.0 && p <= 1.0);
    for (size_t i = 0; i < bits.size(); i++) bits[i] = random.P(p);
  }

  /// Generate a random vector in the specified type and range.
  template <typename T>
  inline void RandomizeVector(emp::vector<T> & vals, Random & random, T min, T max) {
    for (T & v : vals) v = (T) random.GetDouble((double) min, (double) max);
  }

  /// Generate one random BitSet<W>.
  /// Given a vector of other BitSets (unique_from), this function will guarantee
  /// the generated BitSet is unique with respect to those BitSets.
  /// @param rnd - Random number generator to use.
  /// @param unique_from - Other BitSets that the generated BitSet should be unique from.
  template<size_t W>
  BitSet<W> RandomBitSet(emp::Random & rnd, const emp::vector<BitSet<W>> & unique_from=emp::vector<BitSet<W>>()) {
    std::unordered_set<BitSet<W>> unique_from_set(unique_from.begin(), unique_from.end());
    emp_assert(unique_from_set.size() < emp::Pow2(W), "BitSet<W> is not large enough to be able to guarantee requested number of unique tags");
    BitSet<W> new_bitset(rnd, 0.5);
    while (unique_from_set.size()) {
      if (emp::Has(unique_from_set, new_bitset)) {
        new_bitset.Randomize(rnd);
        continue;
      }
      break;
    }
    return new_bitset;
  }

  /// Generate 'count' number of random BitSet<W>.
  /// Given a vector of other bitsets (unique_from), this function will guarantee the bitsets generated
  /// and returned are unique with respect to unique_from.
  /// @param rnd - Random number generator to use when generating a random bitset.
  /// @param count - How many bitsets should be generated?
  /// @param guarantee_unique - Should generated bitsets be guaranteed to be unique from each other?
  /// @param unique_from - Other bitsets that the bitsets being generated should be unique with respect to. Only used if 'guarantee_unique' is true.
  template<size_t W>
  emp::vector<BitSet<W>> RandomBitSets(emp::Random & rnd, size_t count, bool guarantee_unique=false,
                                       const emp::vector<BitSet<W>> & unique_from=emp::vector<BitSet<W>>())
  {
    // If we don't have to make any promises, run quickly!
    emp::vector<BitSet<W>> new_bitsets(count);
    if (!guarantee_unique && unique_from.size() == 0) {
      for (auto & bs : new_bitsets) { bs.Randomize(rnd); }
    } else {
      std::unordered_set<BitSet<W>> unique_from_set(unique_from.begin(), unique_from.end());
      emp_assert(unique_from_set.size()+count <= emp::Pow2(W), "Not possible to generate requested number of BitSets");
      for (auto & bs : new_bitsets) {
        bs.Randomize(rnd);
        while (unique_from_set.size()) {
          if (emp::Has(unique_from_set, bs)) {
            bs.Randomize(rnd);
            continue;
          }
          break;
        }
        unique_from_set.emplace(bs);
      }
    }
    return new_bitsets;
  }
}

#endif

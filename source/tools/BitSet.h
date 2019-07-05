/**
 *  @note This file is part of Empirical, https://github.com/devosoft/Empirical
 *  @copyright Copyright (C) Michigan State University, MIT Software license; see doc/LICENSE.md
 *  @date 2016-2018
 *
 *  @file  BitSet.h
 *  @brief A drop-in replacement for std::bitset, with additional bit magic features.
 *  @note Status: RELEASE
 *
 *  @note Like std::bitset, bit zero is on the right side.  Unlike std::bitset, emp::BitSet
 *       gives access to bit fields for easy access to different sized chucnk of bits and
 *       implementation new bit-magic tricks.
 */


#ifndef EMP_BIT_SET_H
#define EMP_BIT_SET_H

#include <iostream>
#include <limits.h>   // for CHAR_BIT

#include "../base/assert.h"
#include "../base/vector.h"
#include "../base/Ptr.h"

#include "bitset_utils.h"
#include "functions.h"
#include "math.h"
#include "Random.h"

namespace emp {

  ///  A fixed-sized (but arbitrarily large) array of bits, and optimizes operations on those bits
  ///  to be as fast as possible.
  template <size_t NUM_BITS> class BitSet {
  private:

#ifdef EMSCRIPTEN
    using field_t = uint32_t;  ///< Field sizes are 32 bits in Emscripten (max directly handled)
#else
    using field_t = uint64_t;  ///< Field sizes are 64 bits in native.
#endif

    static constexpr field_t FIELD_BITS = sizeof(field_t)*8; ///< How many bits are in a field?

    static constexpr field_t FIELD_LOG2 = emp::Log2(FIELD_BITS);

    /// Fields hold bits in groups of 32 or 64 (as uint32_t or uint64_t);
    /// how many fields do we need?
    static constexpr field_t NUM_FIELDS = (1 + ((NUM_BITS - 1) / FIELD_BITS));

    /// End position of the stored bits in the last field; 0 if perfect fit.
    static constexpr field_t LAST_BIT = NUM_BITS & (FIELD_BITS - 1);

    /// How many total bytes are needed to represent these bits? (rounded up to full bytes)
    static const field_t NUM_BYTES = 1 + ((NUM_BITS - 1) >> 3);

    field_t bit_set[NUM_FIELDS];  ///< Fields to hold the actual bits for this BitSet.

    /// BitProxy lets us use operator[] on with BitSet as an lvalue.
    class BitProxy {
    private:
      BitSet<NUM_BITS> & bit_set;  ///< BitSet object that this proxy refers to.
      size_t index;                ///< Position in BitSet the this proxy refers to.
    public:
      BitProxy(BitSet<NUM_BITS> & _set, size_t _idx) : bit_set(_set), index(_idx) {
        emp_assert(_idx < bit_set.size());
      }

      /// Set the bit value that this proxy refers to.
      BitProxy & operator=(bool b) {    // lvalue handling...
        bit_set.Set(index, b);
        return *this;
      }

      /// Convert BitProxy to a regular boolean value.
      operator bool() const {            // rvalue handling...
        return bit_set.Get(index);
      }

      /// Flip this bit.
      BitProxy & Toggle() { bit_set.Toggle(index); return *this; }
    };
    friend class BitProxy;

    inline static size_t FieldID(const size_t index) {
      emp_assert((index >> FIELD_LOG2) < NUM_FIELDS);
      return index >> FIELD_LOG2;
    }
    inline static size_t FieldPos(const size_t index) { return index & (FIELD_BITS - 1); }

    inline static size_t Byte2Field(const size_t index) { return index/4; }
    inline static size_t Byte2FieldPos(const size_t index) { return (index & 3) << 3; }

    inline void Copy(const field_t in_set[NUM_FIELDS]) {
      for (size_t i = 0; i < NUM_FIELDS; i++) bit_set[i] = in_set[i];
    }

    /// Helper: call SHIFT with positive number instead
    void ShiftLeft(const field_t shift_size) {
      if (shift_size > NUM_BITS) {
        Clear();
        return;
      }
      const int field_shift = shift_size / FIELD_BITS;
      const int bit_shift = shift_size % FIELD_BITS;
      const int bit_overflow = FIELD_BITS - bit_shift;

      // Loop through each field, from L to R, and update it.
      if (field_shift) {
        for (int i = NUM_FIELDS - 1; i >= field_shift; --i) {
          bit_set[i] = bit_set[i - field_shift];
        }
        for (int i = field_shift - 1; i >= 0; i--) bit_set[i] = 0;
      }

      // account for bit_shift
      if (bit_shift) {
        for (int i = NUM_FIELDS - 1; i > field_shift; --i) {
          bit_set[i] <<= bit_shift;
          bit_set[i] |= (bit_set[i-1] >> bit_overflow);
        }
        // Handle final field (field_shift position)
        bit_set[field_shift] <<= bit_shift;
      }

      // Mask out any bits that have left-shifted away
      if (LAST_BIT) { bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT); }
    }


    /// Helper for calling SHIFT with negative number
    void ShiftRight(const field_t shift_size) {
      if (!shift_size) return;
      if (shift_size > NUM_BITS) {
        Clear();
        return;
      }
      const field_t field_shift = shift_size / FIELD_BITS;
      const field_t bit_shift = shift_size % FIELD_BITS;
      const field_t bit_overflow = FIELD_BITS - bit_shift;

      // account for field_shift
      if (field_shift) {
        for (size_t i = 0; i < (NUM_FIELDS - field_shift); ++i) {
          bit_set[i] = bit_set[i + field_shift];
        }
        for (size_t i = NUM_FIELDS - field_shift; i < NUM_FIELDS; i++) bit_set[i] = 0;
      }

      // account for bit_shift
      if (bit_shift) {
        for (size_t i = 0; i < (NUM_FIELDS - 1 - field_shift); ++i) {
          bit_set[i] >>= bit_shift;
          bit_set[i] |= (bit_set[i+1] << bit_overflow);
        }
        bit_set[NUM_FIELDS - 1 - field_shift] >>= bit_shift;
      }
    }

    /// Helper: call ROTATE with negative number instead
    void RotateLeft(const field_t shift_size_raw) {
      const field_t shift_size = shift_size_raw % NUM_BITS;

      // special case: for exactly one field_T, try to go low level
      // adapted from https://stackoverflow.com/questions/776508/best-practices-for-circular-shift-rotate-operations-in-c
      if constexpr (NUM_FIELDS == 1) {
        field_t & n = bit_set[0];
        field_t c = shift_size;

        // assumes width is a power of 2.
        const field_t mask = MaskLow<field_t>(FIELD_LOG2);

        c &= mask;
        n = (n<<c) | (n>>( (-(c+FIELD_BITS-NUM_BITS))&mask ));

      } else {

        // note that we already modded shift_size by NUM_BITS
        // so there's no need to mod by FIELD_SIZE here
        const int field_shift = LAST_BIT ? (
          (shift_size + FIELD_BITS - LAST_BIT) / FIELD_BITS
        ) : (
          shift_size / FIELD_BITS
        );
        // if we field shift, we need to shift bits by (FIELD_BITS - LAST_BIT)
        // more to account for the filler that gets pulled out of the middle
        const int bit_shift = LAST_BIT && field_shift ? (
          (shift_size + FIELD_BITS - LAST_BIT) % FIELD_BITS
        ) : (
          shift_size % FIELD_BITS
        );
        const int bit_overflow = FIELD_BITS - bit_shift;

        // if rotating more than field capacity, we need to rotate fields
        std::rotate(
          std::rbegin(bit_set),
          std::rbegin(bit_set)+field_shift,
          std::rend(bit_set)
        );

        // if necessary, shift filler bits out of the middle
        if constexpr ((bool)LAST_BIT) {
          const int filler_idx = (NUM_FIELDS - 1 + field_shift) % NUM_FIELDS;
          for (int i = filler_idx + 1; i < (int)NUM_FIELDS; ++i) {
            bit_set[i-1] |= bit_set[i] << LAST_BIT;
            bit_set[i] >>= (FIELD_BITS - LAST_BIT);
          }
        }

        // account for bit_shift
        if (bit_shift) {

          const field_t keystone = LAST_BIT ? (
            (bit_set[NUM_FIELDS - 1] << (FIELD_BITS - LAST_BIT))
            | (bit_set[NUM_FIELDS - 2] >> LAST_BIT)
          ) : (
            bit_set[NUM_FIELDS - 1]
          );

          for (int i = NUM_FIELDS - 1; i > 0; --i) {
            bit_set[i] <<= bit_shift;
            bit_set[i] |= (bit_set[i-1] >> bit_overflow);
          }
          // Handle final field
          bit_set[0] <<= bit_shift;
          bit_set[0] |= keystone >> bit_overflow;

        }

      }

      // Mask out any bits that have left-shifted away
      if constexpr ((bool)LAST_BIT) {
        bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT);
      }

    }


    /// Helper for calling ROTATE with positive number
    void RotateRight(const field_t shift_size_raw) {

      const field_t shift_size = shift_size_raw % NUM_BITS;
      const int bit_shift = shift_size % FIELD_BITS;

      // special case: for exactly one field_t, try to go low level
      // adapted from https://stackoverflow.com/questions/776508/best-practices-for-circular-shift-rotate-operations-in-c
      if constexpr (NUM_FIELDS == 1) {
        field_t & n = bit_set[0];
        field_t c = shift_size;


        const unsigned int mask = (CHAR_BIT*sizeof(n) - 1);

        // assert ( (c<=mask) &&"rotate by type width or more");
        c &= mask;
        n = (n>>c) | (n<<( (NUM_BITS-c)&mask ));

      } else {

        const field_t field_shift = (shift_size / FIELD_BITS) % NUM_FIELDS;
        const field_t bit_overflow = FIELD_BITS - bit_shift;

        // if rotating more than field capacity, we need to rotate fields
        std::rotate(
          std::begin(bit_set),
          std::begin(bit_set)+field_shift,
          std::end(bit_set)
        );

        // if necessary, shift filler bits out of the middle
        if constexpr ((bool)LAST_BIT) {
          const int filler_idx = NUM_FIELDS - 1 - field_shift;
          for (int i = filler_idx + 1; i < (int)NUM_FIELDS; ++i) {
            bit_set[i-1] |= bit_set[i] << LAST_BIT;
            bit_set[i] >>= (FIELD_BITS - LAST_BIT);
          }
        }

        // account for bit_shift
        if (bit_shift) {

          const field_t keystone = LAST_BIT ? (
            bit_set[0] >> (FIELD_BITS - LAST_BIT)
          ) : (
            bit_set[0]
          );

          if constexpr ((bool)LAST_BIT) {
            bit_set[NUM_FIELDS-1] |= bit_set[0] << LAST_BIT;
          }

          for (size_t i = 0; i < NUM_FIELDS - 1; ++i) {
            bit_set[i] >>= bit_shift;
            bit_set[i] |= (bit_set[i+1] << bit_overflow);
          }
          bit_set[NUM_FIELDS - 1] >>= bit_shift;
          bit_set[NUM_FIELDS - 1] |= keystone << bit_overflow;
        }
      }

      // Mask out any bits that have left-shifted away
      if constexpr ((bool)LAST_BIT) {
        bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT);
      }

    }

  public:
    /// Constructor: Assume all zeroes in set
    BitSet() { Clear(); }

    /// Copy constructor from another BitSet
    BitSet(const BitSet & in_set) { Copy(in_set.bit_set); }

    /// Constructor to generate a random BitSet (with equal prob of 0 or 1).
    BitSet(Random & random) { Randomize(random); }

    /// Constructor to generate a random BitSet with provided prob of 1's.
    BitSet(Random & random, const double p1) { Clear(); Randomize(random, p1); }

    /// Destructor.
    ~BitSet() = default;

    /// Assignment operator.
    BitSet & operator=(const BitSet<NUM_BITS> & in_set) {
      Copy(in_set.bit_set);
      return *this;
    }

    /// Set all bits randomly, with a 50% probability of being a 0 or 1.
    void Randomize(Random & random) {
      // Randomize all fields, then mask off bits in the last field if not complete.
      for (size_t i = 0; i < NUM_FIELDS; i++) bit_set[i] = random.GetUInt();
      if (LAST_BIT > 0) bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT);
    }

    /// Set all bits randomly, with a given probability of being a 1.
    void Randomize(Random & random, const double p1) {
      if (p1 == 0.5) return Randomize(random); // If 0.5 probability, generate by field!
      for (size_t i = 0; i < NUM_BITS; i++) Set(i, random.P(p1));
    }

    /// Assign from a BitSet of a different size.
    template <size_t NUM_BITS2>
    BitSet & Import(const BitSet<NUM_BITS2> & in_set) {
      static const size_t NUM_FIELDS2 = 1 + ((NUM_BITS2 - 1) >> FIELD_LOG2);
      static const size_t MIN_FIELDS = (NUM_FIELDS < NUM_FIELDS2) ? NUM_FIELDS : NUM_FIELDS2;
      for (size_t i = 0; i < MIN_FIELDS; i++) bit_set[i] = in_set.GetUInt(i);  // Copy avail fields
      for (size_t i = MIN_FIELDS; i < NUM_FIELDS; i++) bit_set[i] = 0;         // Zero extra fields
      return *this;
    }

    /// Convert to a Bitset of a different size.
    template <size_t NUM_BITS2>
    BitSet<NUM_BITS2> Export() const {
      static const size_t NUM_FIELDS2 = 1 + ((NUM_BITS2 - 1) >> FIELD_LOG2);
      static const size_t MIN_FIELDS = (NUM_FIELDS < NUM_FIELDS2) ? NUM_FIELDS : NUM_FIELDS2;
      BitSet<NUM_BITS2> out_bits;
      for (size_t i = 0; i < MIN_FIELDS; i++) out_bits.SetUInt(i, bit_set[i]);  // Copy avail fields
      for (size_t i = MIN_FIELDS; i < NUM_FIELDS; i++) out_bits.SetUInt(i, 0);  // Zero extra fields
      return out_bits;
    }

    /// Test if two BitSet objects are identical.
    bool operator==(const BitSet & in_set) const {
      for (size_t i = 0; i < NUM_FIELDS; ++i) {
        if (bit_set[i] != in_set.bit_set[i]) return false;
      }
      return true;
    }

    /// Compare two BitSet objects, based on the associated binary value.
    bool operator<(const BitSet & in_set) const {
      for (int i = NUM_FIELDS-1; i >= 0; --i) {         // Start loop at the largest field.
        if (bit_set[i] == in_set.bit_set[i]) continue;  // If same, keep looking!
        return (bit_set[i] < in_set.bit_set[i]);        // Otherwise, do comparison
      }
      return false;
    }

    /// Compare two BitSet objects, based on the associated binary value.
    bool operator<=(const BitSet & in_set) const {
      for (int i = NUM_FIELDS-1; i >= 0; --i) {         // Start loop at the largest field.
        if (bit_set[i] == in_set.bit_set[i]) continue;  // If same, keep looking!
        return (bit_set[i] < in_set.bit_set[i]);        // Otherwise, do comparison
      }
      return true;
    }

    /// Test if two BitSet objects are different.
    bool operator!=(const BitSet & in_set) const { return !operator==(in_set); }

    /// Compare two BitSet objects, based on the associated binary value.
    bool operator>(const BitSet & in_set) const { return !operator<=(in_set); }

    /// Compare two BitSet objects, based on the associated binary value.
    bool operator>=(const BitSet & in_set) const { return !operator<(in_set); }

    /// How many bits are in this BitSet?
    constexpr static size_t GetSize() { return NUM_BITS; }

    /// Retrieve the bit as a specified index.
    bool Get(size_t index) const {
      emp_assert(index >= 0 && index < NUM_BITS);
      const size_t field_id = FieldID(index);
      const size_t pos_id = FieldPos(index);
      return (bit_set[field_id] & (((field_t)1U) << pos_id)) != 0;
    }

    /// Set the bit at a specified index.
    void Set(size_t index, bool value=true) {
      emp_assert(index < NUM_BITS);
      const size_t field_id = FieldID(index);
      const size_t pos_id = FieldPos(index);
      const field_t pos_mask = ((field_t)1U) << pos_id;

      if (value) bit_set[field_id] |= pos_mask;
      else       bit_set[field_id] &= ~pos_mask;
    }

    /// Flip all bits in this BitSet
    BitSet & Toggle() { return NOT_SELF(); }

    /// Flip a single bit
    BitSet & Toggle(size_t index) {
      emp_assert(index >= 0 && index < NUM_BITS);
      const size_t field_id = FieldID(index);
      const size_t pos_id = FieldPos(index);
      (bit_set[field_id] ^= (((field_t)1U) << pos_id));
      return *this;
    }

    /// Flips all the bits in a range [start, end)
    BitSet & Toggle(size_t start, size_t end) {
      emp_assert(start <= end && end <= NUM_BITS);
      for(size_t index = start; index < end; index++) {
        Toggle(index);
      }
      return *this;
    }

    /// Get the full byte starting from the bit at a specified index.
    uint8_t GetByte(size_t index) const {
      emp_assert(index < NUM_BYTES);
      const size_t field_id = Byte2Field(index);
      const size_t pos_id = Byte2FieldPos(index);
      return (bit_set[field_id] >> pos_id) & 255;
    }

    /// Set the full byte starting at the bit at the specified index.
    void SetByte(size_t index, uint8_t value) {
      emp_assert(index < NUM_BYTES);
      const size_t field_id = Byte2Field(index);
      const size_t pos_id = Byte2FieldPos(index);
      const field_t val_uint = value;
      bit_set[field_id] = (bit_set[field_id] & ~(((uint64_t)255U) << pos_id)) | (val_uint << pos_id);
    }

    /// Get the 32-bit unsigned int; index in in 32-bit jumps (i.e., this is a field ID not bit id)
    field_t GetUInt(size_t index) const {
      emp_assert(index < NUM_FIELDS);
      return bit_set[index];
    }

    /// Set the 32-bit unsigned int; index in in 32-bit jumps (i.e., this is a field ID not bit id)
    void SetUInt(size_t index, field_t value) {
      emp_assert(index < NUM_FIELDS);
      bit_set[index] = value;
    }

    /// Get the full 32-bit unsigned int starting from the bit at a specified index.
    field_t GetUIntAtBit(size_t index) {
      emp_assert(index < NUM_BITS);
      const size_t field_id = FieldID(index);
      const size_t pos_id = FieldPos(index);
      if (pos_id == 0) return bit_set[field_id];
      return (bit_set[field_id] >> pos_id) |
        ((field_id+1 < NUM_FIELDS) ? bit_set[field_id+1] << (FIELD_BITS-pos_id) : 0);
    }

    /// Get OUT_BITS bits starting from the bit at a specified index (max 32)
    template <size_t OUT_BITS>
    uint32_t GetValueAtBit(size_t index) {
      static_assert(OUT_BITS <= FIELD_BITS, "Requesting too many bits to fit in a UInt");
      return GetUIntAtBit(index) & MaskLow<uint32_t>(OUT_BITS);
    }

    /// Return true if ANY bits in the BitSet are one, else return false.
    bool Any() const { for (auto i : bit_set) if (i) return true; return false; }

    /// Return true if NO bits in the BitSet are one, else return false.
    bool None() const { return !Any(); }

    /// Return true if ALL bits in the BitSet are one, else return false.
    bool All() const { return (~(*this)).None(); }

    /// Index into a const BitSet (i.e., cannot be set this way.)
    bool operator[](size_t index) const { return Get(index); }

    /// Index into a BitSet, returning a proxy that will allow bit assignment to work.
    BitProxy operator[](size_t index) { return BitProxy(*this, index); }

    /// Set all bits to zero.
    void Clear() { for (auto & i : bit_set) i = (field_t)0U; }

    /// Set all bits to one.
    void SetAll() {
      for (auto & i : bit_set) i = ~((field_t)0U);
      if (LAST_BIT) {
        bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT);
      }
    }

    /// Print all bits to the provided output stream.
    void Print(std::ostream & out=std::cout) const {
      for (size_t i = NUM_BITS; i > 0; i--) { out << Get(i-1); }
    }

    /// Print all bits from smallest to largest, as if this were an array, not a bit representation.
    void PrintArray(std::ostream & out=std::cout) const {
      for (size_t i = 0; i < NUM_BITS; i++) out << Get(i);
    }

    /// Print the locations of all one bits, using the provided spacer (default is a single space)
    void PrintOneIDs(std::ostream & out=std::cout, char spacer=' ') const {
      for (size_t i = 0; i < NUM_BITS; i++) { if (Get(i)) out << i << spacer; }
    }

    /// Count 1's by looping through once for each bit equal to 1
    size_t CountOnes_Sparse() const {
      size_t bit_count = 0;
      for (auto i : bit_set) {
        while (i) {
          i &= (i-1);       // Peel off a single 1.
          bit_count++;      // And increment the counter
        }
      }
      return bit_count;
    }

    /// Count 1's in semi-parallel; fastest for even 0's & 1's
    size_t CountOnes_Mixed() const {
      const uint32_t *uint_bit_set = (const uint32_t*) bit_set;
      size_t bit_count = 0;
      for (size_t i = 0; i < NUM_FIELDS * sizeof(field_t)/4; ++i) {
        const uint32_t v = uint_bit_set[i];
        const uint32_t t1 = v - ((v >> 1) & 0x55555555);
        const uint32_t t2 = (t1 & 0x33333333) + ((t1 >> 2) & 0x33333333);
        bit_count += (((t2 + (t2 >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
      }
      return bit_count;
    }

    /// Count the number of ones in the BitSet using bit tricks for a speedup.
    size_t CountOnes() const { return CountOnes_Mixed(); }

    /// Return the index of the first one in the sequence; return -1 if no ones are available.
    int FindBit() const {
      size_t field_id = 0;
      while (field_id < NUM_FIELDS && bit_set[field_id]==0) field_id++;
      return (field_id < NUM_FIELDS) ? (int) (find_bit(bit_set[field_id]) + (field_id << FIELD_LOG2)) : -1;
    }

    /// Return index of first one in sequence (or -1 if no ones); change this position to zero.
    int PopBit() {
      size_t field_id = 0;
      while (field_id < NUM_FIELDS && bit_set[field_id]==0) field_id++;
      if (field_id == NUM_FIELDS) return -1;  // Failed to find bit!

      const int pos_found = (int) find_bit(bit_set[field_id]);
      bit_set[field_id] &= ~(1U << pos_found);
      return pos_found + (int)(field_id << FIELD_LOG2);
    }

    /// Return index of first one in sequence AFTER start_pos (or -1 if no ones)
    int FindBit(const size_t start_pos) const {
      // @CAO -- There are better ways to do this with bit tricks
      //         (but start_pos is tricky...)
      for (size_t i = start_pos; i < NUM_BITS; i++) {
        if (Get(i)) return (int) i;
      }
      return -1;
    }

    /// Return a vector indicating the posistions of all ones in the BitSet.
    emp::vector<size_t> GetOnes() const {
      // @CAO -- There are better ways to do this with bit tricks.
      emp::vector<size_t> out_set(CountOnes());
      size_t cur_pos = 0;
      for (size_t i = 0; i < NUM_BITS; i++) {
        if (Get(i)) out_set[cur_pos++] = i;
      }
      return out_set;
    }

    /// Perform a Boolean NOT on this BitSet and return the result.
    BitSet NOT() const {
      BitSet out_set(*this);
      for (size_t i = 0; i < NUM_FIELDS; i++) out_set.bit_set[i] = ~bit_set[i];
      if (LAST_BIT > 0) out_set.bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT);
      return out_set;
    }

    /// Perform a Boolean AND with a second BitSet and return the result.
    BitSet AND(const BitSet & set2) const {
      BitSet out_set(*this);
      for (size_t i = 0; i < NUM_FIELDS; i++) out_set.bit_set[i] = bit_set[i] & set2.bit_set[i];
      return out_set;
    }

    /// Perform a Boolean OR with a second BitSet and return the result.
    BitSet OR(const BitSet & set2) const {
      BitSet out_set(*this);
      for (size_t i = 0; i < NUM_FIELDS; i++) out_set.bit_set[i] = bit_set[i] | set2.bit_set[i];
      return out_set;
    }

    /// Perform a Boolean NAND with a second BitSet and return the result.
    BitSet NAND(const BitSet & set2) const {
      BitSet out_set(*this);
      for (size_t i = 0; i < NUM_FIELDS; i++) out_set.bit_set[i] = ~(bit_set[i] & set2.bit_set[i]);
      if (LAST_BIT > 0) out_set.bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT);
      return out_set;
    }

    /// Perform a Boolean NOR with a second BitSet and return the result.
    BitSet NOR(const BitSet & set2) const {
      BitSet out_set(*this);
      for (size_t i = 0; i < NUM_FIELDS; i++) out_set.bit_set[i] = ~(bit_set[i] | set2.bit_set[i]);
      if (LAST_BIT > 0) out_set.bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT);
      return out_set;
    }

    /// Perform a Boolean XOR with a second BitSet and return the result.
    BitSet XOR(const BitSet & set2) const {
      BitSet out_set(*this);
      for (size_t i = 0; i < NUM_FIELDS; i++) out_set.bit_set[i] = bit_set[i] ^ set2.bit_set[i];
      return out_set;
    }

    /// Perform a Boolean EQU with a second BitSet and return the result.
    BitSet EQU(const BitSet & set2) const {
      BitSet out_set(*this);
      for (size_t i = 0; i < NUM_FIELDS; i++) out_set.bit_set[i] = ~(bit_set[i] ^ set2.bit_set[i]);
      if (LAST_BIT > 0) out_set.bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT);
      return out_set;
    }


    /// Perform a Boolean NOT on this BitSet, store result here, and return this object.
    BitSet & NOT_SELF() {
      for (size_t i = 0; i < NUM_FIELDS; i++) bit_set[i] = ~bit_set[i];
      if (LAST_BIT > 0) bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT);
      return *this;
    }

    /// Perform a Boolean AND with a second BitSet, store result here, and return this object.
    BitSet & AND_SELF(const BitSet & set2) {
      for (size_t i = 0; i < NUM_FIELDS; i++) bit_set[i] = bit_set[i] & set2.bit_set[i];
      return *this;
    }

    /// Perform a Boolean OR with a second BitSet, store result here, and return this object.
    BitSet & OR_SELF(const BitSet & set2) {
      for (size_t i = 0; i < NUM_FIELDS; i++) bit_set[i] = bit_set[i] | set2.bit_set[i];
      return *this;
    }

    /// Perform a Boolean NAND with a second BitSet, store result here, and return this object.
    BitSet & NAND_SELF(const BitSet & set2) {
      for (size_t i = 0; i < NUM_FIELDS; i++) bit_set[i] = ~(bit_set[i] & set2.bit_set[i]);
      if (LAST_BIT > 0) bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT);
      return *this;
    }

    /// Perform a Boolean NOR with a second BitSet, store result here, and return this object.
    BitSet & NOR_SELF(const BitSet & set2) {
      for (size_t i = 0; i < NUM_FIELDS; i++) bit_set[i] = ~(bit_set[i] | set2.bit_set[i]);
      if (LAST_BIT > 0) bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT);
      return *this;
    }

    /// Perform a Boolean XOR with a second BitSet, store result here, and return this object.
    BitSet & XOR_SELF(const BitSet & set2) {
      for (size_t i = 0; i < NUM_FIELDS; i++) bit_set[i] = bit_set[i] ^ set2.bit_set[i];
      return *this;
    }

    /// Perform a Boolean EQU with a second BitSet, store result here, and return this object.
    BitSet & EQU_SELF(const BitSet & set2) {
      for (size_t i = 0; i < NUM_FIELDS; i++) bit_set[i] = ~(bit_set[i] ^ set2.bit_set[i]);
      if (LAST_BIT > 0) bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT);
      return *this;
    }

    /// Positive shifts go right and negative shifts go left (0 does nothing);
    /// return result.
    BitSet SHIFT(const int shift_size) const {
      BitSet out_set(*this);
      if (shift_size > 0) out_set.ShiftRight((field_t) shift_size);
      else if (shift_size < 0) out_set.ShiftLeft((field_t) (-shift_size));
      return out_set;
    }

    /// Positive shifts go right and negative shifts go left (0 does nothing);
    /// store result here, and return this object.
    BitSet & SHIFT_SELF(const int shift_size) {
      if (shift_size > 0) ShiftRight((field_t) shift_size);
      else if (shift_size < 0) ShiftLeft((field_t) -shift_size);
      return *this;
    }

    /// Positive rotates go left and negative rotates go left (0 does nothing);
    /// return result.
    BitSet ROTATE(const int rotate_size) const {
      BitSet out_set(*this);
      if (rotate_size > 0) out_set.RotateRight((field_t) rotate_size);
      else if (rotate_size < 0) out_set.RotateLeft((field_t) (-rotate_size));
      return out_set;
    }

    /// Positive rotates go right and negative rotates go left (0 does nothing);
    /// store result here, and return this object.
    BitSet & ROTATE_SELF(const int rotate_size) {
      if (rotate_size > 0) RotateRight((field_t) rotate_size);
      else if (rotate_size < 0) RotateLeft((field_t) -rotate_size);
      return *this;
    }

    /// Specialized implementation to rotate left by one bit
    BitSet & L_ROTATE_SELF() {

      if constexpr (NUM_BITS == 1) return *this;

      const int bit_shift = 1;

      // special case: for exactly one field_t, try to go low level
      // adapted from https://stackoverflow.com/questions/776508/best-practices-for-circular-shift-rotate-operations-in-c
      if constexpr (NUM_FIELDS == 1) {
        field_t & n = bit_set[0];
        field_t c = bit_shift;

        // assumes width is a power of 2.
        const unsigned int mask = (CHAR_BIT*sizeof(n) - 1);

        c &= mask;
        n = (n<<c) | (n>>( (-(c+FIELD_BITS-NUM_BITS))&mask ));

      } else {

        const int bit_overflow = FIELD_BITS - bit_shift;


        const field_t keystone = LAST_BIT ? (
          (bit_set[NUM_FIELDS - 1] << (FIELD_BITS - LAST_BIT))
          | (bit_set[NUM_FIELDS - 2] >> LAST_BIT)
        ) : (
          bit_set[NUM_FIELDS - 1]
        );

        for (int i = NUM_FIELDS - 1; i > 0; --i) {
          bit_set[i] <<= bit_shift;
          bit_set[i] |= (bit_set[i-1] >> bit_overflow);
        }
        // Handle final field
        bit_set[0] <<= bit_shift;
        bit_set[0] |= keystone >> bit_overflow;

      }

      // mask out filler bits
      if constexpr ((bool)LAST_BIT) {
        bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT);
      }

      return *this;
    }


    /// Specialized implementation to rotate right by one bit
    BitSet & R_ROTATE_SELF() {

      if constexpr (NUM_BITS == 1) return *this;

      const int bit_shift = 1;

      // special case: for exactly one field_t, try to go low level
      // adapted from https://stackoverflow.com/questions/776508/best-practices-for-circular-shift-rotate-operations-in-c
      if constexpr (NUM_FIELDS == 1) {
        field_t & n = bit_set[0];
        field_t c = bit_shift;

        //bitset_utils mask tools

        const unsigned int mask = (CHAR_BIT*sizeof(n) - 1);

        // assert ( (c<=mask) &&"rotate by type width or more");
        c &= mask;
        n = (n>>c) | (n<<( (NUM_BITS-c)&mask ));

      } else {

        const field_t bit_overflow = FIELD_BITS - bit_shift;

        const field_t keystone = LAST_BIT ? (
          bit_set[0] >> (FIELD_BITS - LAST_BIT)
        ) : (
          bit_set[0]
        );

        if constexpr ((bool)LAST_BIT) {
          bit_set[NUM_FIELDS-1] |= bit_set[0] << LAST_BIT;
        }

        for (size_t i = 0; i < NUM_FIELDS - 1; ++i) {
          bit_set[i] >>= bit_shift;
          bit_set[i] |= (bit_set[i+1] << bit_overflow);
        }
        bit_set[NUM_FIELDS - 1] >>= bit_shift;
        bit_set[NUM_FIELDS - 1] |= keystone << bit_overflow;
      }

      // mask out filler bits
      if constexpr ((bool)LAST_BIT) {
        bit_set[NUM_FIELDS - 1] &= MaskLow<field_t>(LAST_BIT);
      }

      return *this;

    }

    /// Operator bitwise NOT...
    BitSet operator~() const { return NOT(); }

    /// Operator bitwise AND...
    BitSet operator&(const BitSet & ar2) const { return AND(ar2); }

    /// Operator bitwise OR...
    BitSet operator|(const BitSet & ar2) const { return OR(ar2); }

    /// Operator bitwise XOR...
    BitSet operator^(const BitSet & ar2) const { return XOR(ar2); }

    /// Operator shift left...
    BitSet operator<<(const size_t shift_size) const { return SHIFT(-(int)shift_size); }

    /// Operator shift right...
    BitSet operator>>(const size_t shift_size) const { return SHIFT((int)shift_size); }

    /// Compound operator bitwise AND...
    const BitSet & operator&=(const BitSet & ar2) { return AND_SELF(ar2); }

    /// Compound operator bitwise OR...
    const BitSet & operator|=(const BitSet & ar2) { return OR_SELF(ar2); }

    /// Compound operator bitwise XOR...
    const BitSet & operator^=(const BitSet & ar2) { return XOR_SELF(ar2); }

    /// Compound operator shift left...
    const BitSet & operator<<=(const size_t shift_size) { return SHIFT_SELF(-(int)shift_size); }

    /// Compound operator shift right...
    const BitSet & operator>>=(const size_t shift_size) { return SHIFT_SELF((int)shift_size); }

    /// Function to allow drop-in replacement with std::bitset.
    constexpr static size_t size() { return NUM_BITS; }

    /// Function to allow drop-in replacement with std::bitset.
    inline bool all() const { return All(); }

    /// Function to allow drop-in replacement with std::bitset.
    inline bool any() const { return Any(); }

    /// Function to allow drop-in replacement with std::bitset.
    inline bool none() const { return !Any(); }

    /// Function to allow drop-in replacement with std::bitset.
    inline size_t count() const { return CountOnes_Mixed(); }

    /// Function to allow drop-in replacement with std::bitset.
    inline BitSet & flip() { return Toggle(); }

    /// Function to allow drop-in replacement with std::bitset.
    inline BitSet & flip(size_t pos) { return Toggle(pos); }

    /// Function to allow drop-in replacement with std::bitset.
    inline BitSet & flip(size_t start, size_t end) { return Toggle(start, end); }
  };

  template <size_t NUM_BITS1, size_t NUM_BITS2>
  BitSet<NUM_BITS1+NUM_BITS2> join(const BitSet<NUM_BITS1> & in1, const BitSet<NUM_BITS2> & in2) {
    BitSet<NUM_BITS1+NUM_BITS2> out_bits;
    out_bits.Import(in2);
    out_bits <<= NUM_BITS1;
    out_bits |= in2.template Export<NUM_BITS1+NUM_BITS2>();
  }

  /// Computes simple matching coefficient (https://en.wikipedia.org/wiki/Simple_matching_coefficient).
  template <size_t NUM_BITS>
  double SimpleMatchCoeff(const BitSet<NUM_BITS> & in1, const BitSet<NUM_BITS> & in2) {
    emp_assert(NUM_BITS > 0); // TODO: can be done with XOR
    return (double)((in1 & in2).CountOnes() + (~in1 & ~in2).CountOnes()) / (double)NUM_BITS;
  }

}

template <size_t NUM_BITS> std::ostream & operator<<(std::ostream & out, const emp::BitSet<NUM_BITS> & _bit_set) {
  _bit_set.Print(out);
  return out;
}



#endif

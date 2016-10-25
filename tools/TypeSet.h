//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2016.
//  Released under the MIT Software license; see doc/LICENSE
//
//  TypeSet represents a collection of types for manipulation (typically for metaprogramming)

#ifndef EMP_TYPE_SET_H
#define EMP_TYPE_SET_H

#include "meta.h"

namespace emp {

  // Pre-declaration of TypeSet
  template <typename... Ts> struct TypeSet;

  // Create a null type for padding.
  struct null_t {};

  // Anonymous helpers for TypeSet
  namespace {
    // Create add N copies of the same type to the end of a typeset.
    template <typename START, typename T, int N>
    struct ts_pad { using type = typename ts_pad<START,T,N-1>::type::template add_t<T>; };
    template <typename START, typename T>
    struct ts_pad<START, T,0> { using type = START; };

    // Helper for shifting a specified number of types to TypeSet T1 from TypeSet T2.
    // Example: to crop, move a specified number of types to empty TypeSet and return it.
    //          to merge, move all of one typeset over to the other and return it.
    template <int S, typename T1, typename T2>
    struct ts_shift {
      using move_t = typename T2::first_t;
      using inc_t = typename T1::template add_t<move_t>;
      using dec_t = typename T2::pop_t;
      using type1 = typename ts_shift<S-1, inc_t, dec_t>::type1;
      using type2 = typename ts_shift<S-1, inc_t, dec_t>::type2;
    };
    template <typename T1, typename T2>
    struct ts_shift<0, T1, T2> {
      using type1 = T1;
      using type2 = T2;
    };

  }

  template <typename T, int N> using TypeSetFill = typename ts_pad<TypeSet<>,T,N>::type;

  // Specialized TypeSet with at least one type.
  template <typename T1, typename... Ts>
  struct TypeSet<T1, Ts...> {
    template <typename T> constexpr static bool Has() { return has_type<T,T1,Ts...>(); }
    template <typename T> constexpr static int Count() { return count_type<T,T1,Ts...>(); }

    // Type ID's can be retrieved with
    //   GetID<my_type>() to get the ID associated with specific type my_type
    //   GetID(owner) to get the ID associated with the type of 'owner'
    template <typename T> constexpr static int GetID() { return get_type_index<T,T1,Ts...>(); }
    template <typename T> constexpr static int GetID(const T &) { return get_type_index<T,T1,Ts...>(); }

    constexpr static int SIZE = 1+sizeof...(Ts);
    constexpr static int GetSize() { return SIZE; }

    constexpr static bool IsEmpty() { return false; }
    constexpr static bool IsUnique() { return has_unique_types<T1,Ts...>(); }

    // Get the type associated with a specified position in the pack.
    template <int POS> using type = pack_id<POS, T1, Ts...>;

    // Other type lookups
    using this_t = TypeSet<T1, Ts...>;
    using first_t = T1;
    using last_t = last_type<T1,Ts...>;

    // Modifications
    template <typename... T> using push_front_t = TypeSet<T...,T1,Ts...>;
    template <typename... T> using push_back_t = TypeSet<T1,Ts...,T...>;
    template <typename... T> using add_t = TypeSet<T1,Ts...,T...>;    // Same as push_back_t...

    using pop_t = TypeSet<Ts...>;
    template <int N> using popN_t = typename ts_shift<N, this_t, TypeSet<>>::type2;
    template <int N> using crop_t = typename ts_shift<N, TypeSet<>, this_t>::type1;

    template <typename T, int N=1> using pad_t = typename ts_pad<this_t,T,N>::type;
    template <int N, typename DEFAULT=null_t>
      using resize_t = typename pad_t<DEFAULT,(N>SIZE)?(N-SIZE):0>::template crop_t<N>;
    template <typename IN> using merge_t = typename ts_shift<IN::SIZE, this_t, IN>::type1;
    using reverse_t = typename pop_t::reverse_t::template push_back_t<T1>;
    using rotate_t = typename pop_t::template push_back_t<T1>;

    // Conversions
    template <typename RETURN_T> using to_function_t = RETURN_T(T1,Ts...);
    template <template <typename...> class TEMPLATE>
    using apply_t = TEMPLATE<T1, Ts...>;

  };

  // Specialized TypeSet with no types.
  template <>
  struct TypeSet<> {
    template <typename T> constexpr static bool Has() { return false; }
    template <typename T> constexpr static int Count() { return 0; }

    // GetID() NOT IMPLEMENTED since no ID's are available.
    constexpr static int SIZE = 0;
    constexpr static int GetSize() { return 0; }
    constexpr static bool IsEmpty() { return true; }
    constexpr static bool IsUnique() { return true; }

    // first_t, last_t, and pop_t not implemented, since no types are available.
    // (@CAO: could implement with null_t)

    using this_t = TypeSet<>;

    template <typename... T> using push_front_t = TypeSet<T...>;
    template <typename... T> using push_back_t = TypeSet<T...>;
    template <typename... T> using add_t = TypeSet<T...>;

    template <typename T, int N=1> using pad_t = typename ts_pad<this_t,T,N>::type;
    template <int N, typename DEFAULT=null_t> using resize_t = pad_t<DEFAULT,N>;

    template <typename IN> using merge_t = IN;
    using reverse_t = this_t;
    using rotate_t = this_t;

    template <typename RETURN_T> using to_function_t = RETURN_T();
  };
}

#endif

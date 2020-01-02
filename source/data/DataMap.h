/**
 *  @note This file is part of Empirical, https://github.com/devosoft/Empirical
 *  @copyright Copyright (C) Michigan State University, MIT Software license; see doc/LICENSE.md
 *  @date 2018-2019.
 *
 *  @file  DataMap.h
 *  @brief A DataMap links names to arbitrary object types.
 *  @note Status: ALPHA
 *
 *  A DataMap links data names to arbitrary object types.  Each data map is composed of a
 *  MemoryImage that holds a set of values and a DataLayout that maps names and other information
 *  to those values.
 * 
 *  Use the Add() method to include a new data entry into the DataMap.
 * 
 *  Use the Get() method to retrieve reference to a value in the DataMap.
 * 
 *  Use the Set() method to change a value in the DataMap
 *  (you may also use Get() followed by an assignment.)
 * 
 *  New data entries can be added to a DataMap, but never removed (for efficiency purposes).
 *  When a DataMap is copied, all data entries are also copied (relatively fast).
 *  As long as a DataMaps layout doesn't change, all copied maps will share the same layout (fast). 
 * 
 * 
 *  DEVELOPER NOTES:
 *  - Layouts should be freezable to ensure that no new maps change the Layout.
 *  - AddLog() instead of Add() if you want to keep a set of values.  This should take flags to
 *    indicate how values should be retrieved by default, such as First, Last, Average, etc.
 *  - Settings for all entries should have more information on how they are dealt with, such as if
 *    they should be included in output an how.
 * 
 *  - After everything else is working, build a LocalDataMap<size_t> that locks in the size at
 *    compiletime, providing more localized memory.  Otherwise DataMap as a whole can be built
 *    on a templated class that takes an IMAGE_T as an argument.
 */

#ifndef EMP_DATA_MAP_H
#define EMP_DATA_MAP_H

#include <string>
#include <unordered_map>
#include <cstring>        // For std::memcpy

#include "../base/assert.h"
#include "../base/Ptr.h"
#include "../meta/TypeID.h"
#include "../tools/map_utils.h"
#include "../tools/string_utils.h"

#include "MemoryImage.h"
#include "DataLayout.h"

namespace emp {

  class DataMap {
  protected:
    MemoryImage memory;              ///< Memory status for this Map.
    emp::Ptr<DataLayout> layout_ptr; ///< Which layout are we using?

    DataMap(emp::Ptr<DataLayout> in_layout_ptr, size_t in_size)
      : memory(in_size), layout_ptr(in_layout_ptr) { ; }

    void CopyImage(const DataMap & from_map, DataMap & to_map) {
      emp_assert(from_map.layout_ptr == to_map.layout_ptr);
      layout_ptr->CopyImage(from_map.memory, to_map.memory);
    }

  public:
    DataMap() : layout_ptr(emp::NewPtr<DataLayout>()) { ; }
    DataMap(const DataMap & in_map) : layout_ptr(in_map.layout_ptr) {
      CopyImage(in_map, *this);
      layout_ptr->IncMaps();
    }
    DataMap(DataMap && in_map) : memory(std::move(in_map.memory)), layout_ptr(in_map.layout_ptr) {
      in_map.memory.RawResize(0);
    }

    ~DataMap() {
      layout_ptr->DecMaps();
      if (layout_ptr->GetNumMaps() == 0) layout_ptr.Delete();
    }

    /// Retrieve the DataLayout associated with this image.
    DataLayout & GetMapLayout() { return *layout_ptr; }
    const DataLayout & GetMapLayout() const { return *layout_ptr; }

    /// Determine how many Bytes large this image is.
    size_t GetSize() const { return memory.GetSize(); }

    /// Translate a name into an ID.
    size_t GetID(const std::string & name) const { return layout_ptr->GetID(name); }

    /// Is this image using the most current version of the DataLayout?
    bool IsCurrent() const { return GetSize() == layout_ptr->GetImageSize(); }

    /// Test if this map has a setting ID.
    bool HasID(size_t id) const { return layout_ptr->HasID(id); }

    /// Test is this map has a variable by a given name.
    bool HasName(const std::string & name) const { return layout_ptr->HasName(name); }

    /// Test if a variable is of a given type.
    template <typename T> bool IsType(size_t id) const { return layout_ptr->IsType<T>(id); }
    template <typename T> bool IsType(const std::string & name) const {
      return layout_ptr->IsType<T>(GetID(name));
    }

    /// Retrieve a variable by its type and position.
    template <typename T>
    T & Get(size_t id) {
      emp_assert(HasID(id), id, GetSize());
      emp_assert(IsType<T>(id));
      return memory.Get<T>(id);
    }

    /// Retrieve a const variable by its type and position.
    template <typename T>
    const T & Get(size_t id) const {
      emp_assert(HasID(id), id, GetSize());
      emp_assert(IsType<T>(id));
      return memory.Get<T>(id);
    }


    /// Retrieve a variable by its type and name. (Slower!)
    template <typename T>
    T & Get(const std::string & name) {
      emp_assert(HasName(name));
      emp_assert(IsType<T>(name), name, GetType(name), emp::GetTypeID<T>());
      return memory.Get<T>(GetID(name));
    }

    /// Retrieve a const variable by its type and name. (Slower!)
    template <typename T>
    const T & Get(const std::string & name) const {
      emp_assert(HasName(name));
      emp_assert(IsType<T>(name), name, GetType(name), emp::GetTypeID<T>());
      return memory.Get<T>(GetID(name));
    }

    /// Set a variable by ID.
    template <typename T> T & Set(size_t id, const T & value) {
      return (Get<T>(id) = value);
    }

    /// Set a variable by name.
    template <typename T> T & Set(const std::string & name, const T & value) {
      return (Get<T>(name) = value);
    }

    // Type-specific Getters and Setters
    double & GetValue(size_t id) { return Get<double>(id); }
    double GetValue(size_t id) const { return Get<double>(id); }
    double & GetValue(const std::string & name) { return Get<double>(name); }
    double GetValue(const std::string & name) const { return Get<double>(name); }
    double & SetValue(size_t id, double value) { return Set<double>(id, value); }
    double & SetValue(const std::string & name, double value) { return Set<double>(name, value); }

    std::string & GetString(const size_t id) { return Get<std::string>(id); }
    const std::string & GetString(const size_t id) const { return Get<std::string>(id); }
    std::string & GetString(const std::string & name) { return Get<std::string>(name); }
    const std::string & GetString(const std::string & name) const { return Get<std::string>(name); }
    std::string & SetString(const size_t id, const std::string & value) { return Set<std::string>(id, value); }
    std::string & SetString(const std::string & name, const std::string & value) { return Set<std::string>(name, value); }

    /// Look up the type of a variable by ID.
    emp::TypeID GetType(size_t id) const { return layout_ptr->GetType(id); }

    /// Look up the type of a variable by name.
    emp::TypeID GetType(const std::string & name) const { return layout_ptr->GetType(GetID(name)); }


    /// Add a new variable with a specified type, name and value.
    template <typename T>
    size_t Add(const std::string & name,
               const T & default_value,
               const std::string & desc="",
               const std::string & notes="") {
      // If the current layout is shared, first make a copy of it.
      if (layout_ptr->GetNumMaps() > 1) {
        layout_ptr->DecMaps();
        layout_ptr.New(*layout_ptr);
      }

      return layout_ptr->Add<T>(memory, name, default_value, desc, notes);
    }

    // Add type-specific variables.
    template <typename... Ts> size_t AddString(Ts &&... args) { return Add<std::string>(args...); }
    template <typename... Ts> size_t AddValue(Ts &&... args) { return Add<double>(args...); }
  };

}

#endif

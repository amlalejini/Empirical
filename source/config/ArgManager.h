//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2016.
//  Released under the MIT Software license; see doc/LICENSE
//
//  A simple ArgManager tool for sythesizing command-line arguments and config files.


#ifndef EMP_CL_ARG_MANAGER_H
#define EMP_CL_ARG_MANAGER_H

#include <algorithm>
#include <string>
#include <vector>
#include <map>
#include <iterator>
#include <set>
#include <limits>
#include <numeric>

#include "base/Ptr.h"
#include "base/vector.h"
#include "command_line.h"
#include "config.h"

namespace emp {

  /// TODO: add functors and value or
  struct ArgSpec {

    const size_t quota;
    const std::string description;
    const std::unordered_set<std::string> aliases;
    const bool enforce_quota;
    const bool gobble_flags;
    const bool flatten;

    ArgSpec(
      const size_t quota_=0,
      const std::string description_="No description provided.",
      const std::unordered_set<std::string> aliases_=std::unordered_set<std::string>(),
      const bool enforce_quota_=true,
      const bool gobble_flags_=false,
      const bool flatten_=false
    ) : quota(quota_),
        description(description_),
        aliases(aliases_),
        enforce_quota(enforce_quota_),
        gobble_flags(gobble_flags_),
        flatten(flatten_)
    { ; }

  };

  /// TODO
  class ArgManager {

  private:
    std::multimap<std::string, emp::vector<std::string>> packs;
    const std::unordered_map<std::string, ArgSpec> specs;

  public:
    // Convert input arguments to a vector of strings for easier processing.
    static emp::vector<std::string> args_to_strings(int argc, char* argv[]) {
      emp::vector<std::string> args;
      for (size_t i = 0; i < (size_t) argc; i++) {
        args.push_back(argv[i]);
      }
      return args;
    }

    static std::multimap<std::string, emp::vector<std::string>> parse(
      const emp::vector<std::string> args,
      const std::unordered_map<std::string, ArgSpec> & specs = std::unordered_map<std::string, ArgSpec>()
    ) {

      auto res = std::multimap<std::string, emp::vector<std::string>>();

      const auto alias_map = std::accumulate(
        std::begin(specs),
        std::end(specs),
        std::unordered_map<std::string, std::string>(),
        [](
          std::unordered_map<std::string, std::string> l,
          const std::pair<std::string, ArgSpec> & r
        ){
          l.insert({r.first, r.first});
          for(const auto & p : r.second.aliases) l.insert({p, r.first});
          return l;
        }
      );

      // check for duplicate aliases
      const bool check = alias_map.size() == std::accumulate(
        std::begin(specs),
        std::end(specs),
        specs.size(),
        [](
          const size_t l,
          const std::pair<std::string, ArgSpec> & r
        ){
          return l + r.second.aliases.size();
        }
      );

      emp_assert(check, "duplicate aliases detected");

      // lookup table with leading dashes stripped
      const emp::vector<std::string> deflagged = [args](){
        auto res = args;
        for (auto & val : res) {
          val.erase(0, val.find_first_not_of('-'));
        }
        return res;
      }();

      // if word is a valid command or alias for a command,
      // return the deflagged, dealiased command
      // otherwise, it's a positional command
      auto parse_alias = [deflagged, args, alias_map](size_t i) {
        return alias_map.count(deflagged[i]) ?
          alias_map.find(deflagged[i])->second : "_positional";
      };

      for(size_t i = 0; i < args.size(); ++i) {

        // e.g., if "_positional" hasn't been registered in spec
        if ( !specs.count(parse_alias(i)) ) {
          res.insert({
              "_unknown",
              { args[i] }
          });
          continue;
        }

        const std::string & command = specs.find(parse_alias(i))->first;
        const ArgSpec & spec = specs.find(parse_alias(i))->second;

        // handle unknown flags
        if (
          command == "_positional"
          && deflagged[i] != args[i]
          && !spec.gobble_flags
        ) {
          res.insert({
              "_unknown",
              { args[i] }
          });
          continue;
        }


        // fast forward to grab all the args for this argpack
        size_t j;
        for (
          j = i;
          j < args.size()
          && j - i < spec.quota
          && (
            spec.gobble_flags
            || ! (j + 1 < args.size())
            || deflagged[j+1] == args[j+1]
          );
          ++j
        );

        res.insert(
          {
            command,
            emp::vector<std::string>(
              std::next(std::begin(args), command == "_positional" ? i : i+1),
              j+1 < args.size() ? std::next(std::begin(args), j+1) : std::end(args)
            )
          }
        );

        i = j;

      }

      return res;

    }

    /// Make specs for builtin commands
    static std::unordered_map<std::string, ArgSpec> make_builtin_specs(
      const emp::Ptr<const Config> config=nullptr
    ) {

      std::unordered_map<std::string, ArgSpec> res({
        {"_positional", ArgSpec(
          std::numeric_limits<size_t>::max(),
          "Positional arguments.",
          {},
          false,
          false,
          true
        )},
        {"help", ArgSpec(0, "Print help information.", {"h"})},
        {"gen", ArgSpec(1, "Generate configuration file.")},
        {"make-const", ArgSpec(1, "Generate const version of macros file.")}
      });

      for (const auto & e : *config) {
        const auto & entry = e.second;
        res.insert({
          entry->GetName(),
          ArgSpec(
            1,
            emp::to_string(
              entry->GetDescription(),
              " (type=", entry->GetType(),
              "; default=", entry->GetDefault(), ')'
            )
          )
        });
      }

      return res;
    }

    // create best-effort specifications for unspecified Args
    static std::unordered_map<std::string, ArgSpec> retrofit_specs(
      const std::multimap<std::string, emp::vector<std::string>> & packs
    ) {

      std::unordered_map<std::string, ArgSpec> res;

      for (const auto & [n, p] : packs) {
        if (!res.count(n)) {
          res.insert({
            n,
            ArgSpec(
              0,
              "Retrofitted.",
              {},
              false
            )
          });
        }
      }

      return res;

    }

    ArgManager(
      int argc,
      char* argv[],
      const std::unordered_map<std::string, ArgSpec> & specs_ = std::unordered_map<std::string, ArgSpec>()
    ) : ArgManager(
      ArgManager::args_to_strings(argc, argv),
      specs_
    ) { ; }

    ArgManager(
      const emp::vector<std::string> args,
      const std::unordered_map<std::string, ArgSpec> & specs_ = std::unordered_map<std::string, ArgSpec>()
    ) : ArgManager(
      ArgManager::parse(args, specs_),
      specs_
    ) { ; }

    ArgManager(
      const std::multimap<std::string, emp::vector<std::string>> & packs_,
      const std::unordered_map<std::string, ArgSpec> & specs_ = std::unordered_map<std::string, ArgSpec>()
    ) : packs(packs_), specs(specs_.size() ? specs_ : retrofit_specs(packs_)) {

      // flatten args that should be flattened
      for (auto & [n, s] : specs) {
        if (s.flatten && packs.count(n)) {
          emp::vector<std::string> flat = std::accumulate(
            packs.equal_range(n).first,
            packs.equal_range(n).second,
            emp::vector<std::string>(),
            [](
              emp::vector<std::string> l,
              const std::pair<std::string, emp::vector<std::string>> & r
            ){
              l.insert(std::end(l), std::begin(r.second), std::end(r.second));
              return l;
            }
          );
          packs.erase(packs.equal_range(n).first, packs.equal_range(n).second);
          packs.insert({n, flat});
        }
      }
    }

    ~ArgManager() { ; }

    /// UseArg consumes an argument pack accessed by a certain name.
    std::optional<emp::vector<std::string>> UseArg(const std::string & name) {

      if (!specs.count(name) || !packs.count(name)) return std::nullopt;

      const auto & cur_spec = specs.find(name)->second;
      const auto & cur_pack = packs.lower_bound(name)->second;

      const auto res = (
        !cur_spec.enforce_quota || cur_spec.quota == cur_pack.size()
      ) ? std::make_optional(cur_pack) : std::nullopt;

      if (res) packs.erase(packs.lower_bound(name));

      return res;

    }

    /// ViewArg returns all argument packs under a certain name.
    emp::vector<emp::vector<std::string>> ViewArg(const std::string & name) {

      emp::vector<emp::vector<std::string>> res;

      const auto range = packs.equal_range(name);
      for (auto it = range.first; it != range.second; ++it) {
        res.push_back(it->second);
      }

      return res;

    }

    // Process builtin commands.
    /// Return bool for "should program proceed" (i.e., true=continue, false=exit).
    bool ProcessBuiltin(
      const emp::Ptr<const Config> config=nullptr,
      std::ostream & os=std::cout
    ) {

      if (UseArg("help")) {
        PrintHelp(os);
        return false;
      }

      bool proceed = true;

      if (const auto res = UseArg("gen"); res && config) {
        const std::string cfg_file = res->front();
        os << "Generating new config file: " << cfg_file << std::endl;
        config->Write(cfg_file);
        proceed = false;
      }

      if (const auto res = UseArg("make-const"); res && config)  {
        const std::string macro_file = res->front();
        os << "Generating new macros file: " << macro_file << std::endl;
        config->WriteMacros(macro_file, true);
        proceed = false;
      }

      return proceed;

    }

    /// Print the current state of the ArgManager.
    void Print(std::ostream & os=std::cout) const {

      for(const auto & it : packs ) {
        os << it.first << ":";
        for(const auto & v : it.second ) {
          os << " " << v;
        }
        os << std::endl;
      }

    }

    /// Print information about all known argument types and what they're for;
    /// make pretty.
    void PrintHelp(std::ostream & os=std::cerr) const {

      for (const auto & [n, s] : specs) {
        os << "-"
           << n;
        for (const auto & a : s.aliases) os << " -" << a;
        os << " [" << ( (!s.enforce_quota) ? "<=" : "=" ) << s.quota << "]";
        os << std::endl
           << "   | "
           << s.description
           << std::endl;

      }

    }

    /// Test if there are any unused arguments, and if so, output an error.
    bool HasUnused(std::ostream & os=std::cerr) const {
      if (packs.size() > 1) {
        Print(os);
        PrintHelp(os);
        return true;
      }
      return false;
    }

    /// Convert settings from a configure object to command-line arguments.
    /// Return bool for "should program proceed" (i.e., true=continue, false=exit).
    void ApplyConfigOptions(Config & config) {

      // Scan through the config object to generate command line flags for each setting.
      for (auto e : config) {

        const auto entry = e.second;

        const auto res = UseArg(entry->GetName());

        if (res) config.Set(entry->GetName(), (*res)[0]);

      }

    }

  };

  namespace cl {

    /// A simple class to manage command-line arguments that were passed in.
    /// Derived from emp::vector<std::string>, but with added functionality for argument handling.
    class ArgManager : public emp::vector<std::string> {
    private:
      using parent_t = emp::vector<std::string>;
      emp::vector<std::string> arg_names;
      emp::vector<std::string> arg_descs;

    public:
      ArgManager() : parent_t(), arg_names(), arg_descs() { ; }
      ArgManager(int argc, char* argv[])
       : parent_t(args_to_strings(argc, argv)), arg_names(), arg_descs() { ; }
      ~ArgManager() { ; }

      /// UseArg takes a name, a variable and an optional description.  If the name exists,
      /// it uses the next argument to change the value of the variable.
      /// Return 1 if found, 0 if not found, and -1 if error (no value provided)
      template <typename T>
      int UseArg(const std::string & name, T & var, const std::string & desc="") {
        arg_names.push_back(name);
        arg_descs.push_back(desc);
        return use_arg_value(*this, name, var);
      }

      /// UseArg can also take a config object and a name, and use the argument to set the
      /// config object.
      int UseArg(const std::string & name, Config & config, const std::string & cfg_name,
                 const std::string & desc="") {
        arg_names.push_back(name);
        arg_descs.push_back(desc);
        std::string var;
        bool rv = use_arg_value(*this, name, var);
        if (rv==1) config.Set(cfg_name, var);
        return rv;
      }

      /// UseFlag takes a name and an optional description.  If the name exists, return true,
      /// otherwise return false.
      bool UseFlag(const std::string & name, const std::string & desc="") {
        arg_names.push_back(name);
        arg_descs.push_back(desc);
        return use_arg(*this, name);
      }

      /// Print information about all known argument types and what they're for; make pretty.
      void PrintHelp(std::ostream & os) const {
        size_t max_name_size = 0;
        for (const auto & name : arg_names) {
          if (max_name_size < name.size()) max_name_size = name.size();
        }
        for (size_t i = 0; i < arg_names.size(); i++) {
          os << arg_names[i]
             << std::string(max_name_size + 1 - arg_names[i].size(), ' ')
             << arg_descs[i]
             << std::endl;
        }
      }

      /// Test if there are any unprocessed arguments, and if so, output an error.
      bool HasUnknown(std::ostream & os=std::cerr) const {
        if (size() > 1) {
          os << "Unknown args:";
          for (size_t i = 1; i < size(); i++) os << " " << (*this)[i];
          os << std::endl;
          PrintHelp(os);
          return true;
        }
        return false;
      }

      /// Leaving TestUnknown for backward compatability; returns opposite of HasUnknown().
      bool TestUnknown(std::ostream & os=std::cerr) const { return !HasUnknown(os); }

      /// Convert settings from a configure object to command-line arguments.
      /// Return bool for "should program proceed" (i.e., true=continue, false=exit).
      bool ProcessConfigOptions(Config & config, std::ostream & os,
                                const std::string & cfg_file="",
                                const std::string & macro_file="")
      {
        // Scan through the config object to generate command line flags for each setting.
        for (auto e : config) {
          auto entry = e.second;
          std::string desc = emp::to_string( entry->GetDescription(),
                                             " (type=", entry->GetType(),
                                             "; default=", entry->GetDefault(), ')' );
          UseArg(to_string('-', entry->GetName()), config, entry->GetName(), desc);
        }

        // Determine if we're using any special options for comman line flags.
        bool print_help    = UseFlag("--help", "Print help information.");
        bool create_config = cfg_file.size() && UseFlag("--gen", "Generate configuration file.");
        bool const_macros  = macro_file.size() && UseFlag("--make-const", "Generate const version of macros file.");

        if (print_help)    { PrintHelp(os); return false; }
        if (create_config) {
          os << "Generating new config file: " << cfg_file << std::endl;
          config.Write(cfg_file);
          return false;
        }
        if (const_macros)  {
          os << "Generating new macros file: " << macro_file << std::endl;
          config.WriteMacros(macro_file, true);
          return false;
        }

        return true;
      }

    };


  }
}

#endif

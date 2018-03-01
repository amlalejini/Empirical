/**
 *  @note This file is part of Empirical, https://github.com/devosoft/Empirical
 *  @copyright Copyright (C) Michigan State University, MIT Software license; see doc/LICENSE.md
 *  @date 2018
 * 
 * This file contains extra analysis tools to use with systematics managers that
 * have non-null DATA_TYPES.
**/

#ifndef EMP_EVO_SYSTEMATICS_ANALYSIS_H
#define EMP_EVO_SYSTEMATICS_ANALYSIS_H

#include "Systematics.h"


// Mutation info functions. Assumes each taxon has a struct containing an unordered map
// with keys that are strings indicating types of mutations and keys that are numbers
// indicating the number of that type of mutation that occurred to make this taxon from
// the parent.

namespace emp {

    /// Returns the total number of times a mutation of type @param type
    /// that along @param taxon 's lineage. (Different from CountMuts in
    /// that CountMuts sums them whereas CountMutSteps would count two
    /// simultaneous mutations of the same type as one event)
    template <typename taxon_t>
    int LineageLength(Ptr<taxon_t> taxon) {
        int count = 0;

        while (taxon) {
            count++;
            taxon = taxon->GetParent();            
        }

        return count;
    }

    /// Returns the total number of times a mutation of type @param type
    /// that along @param taxon 's lineage. (Different from CountMuts in
    /// that CountMuts sums them whereas CountMutSteps would count two
    /// simultaneous mutations of the same type as one event)
    template <typename taxon_t>
    int CountMutSteps(Ptr<taxon_t> taxon, std::string type="substitution") {
        int count = 0;

        while (taxon) {
            count += (int)(taxon->GetData().mut_counts[type] > 0);
            taxon = taxon->GetParent();            
        }

        return count;
    }

    /// Returns the total number of times a mutation of type @param type
    /// that along @param taxon 's lineage. (Different from CountMuts in
    /// that CountMuts sums them whereas CountMutSteps would count two
    /// simultaneous mutations of the same type as one event)
    template <typename taxon_t>
    int CountMutSteps(Ptr<taxon_t> taxon, emp::vector<std::string> types) {
        int count = 0;

        while (taxon) {
            for (std::string type : types) {
                count += (int)(taxon->GetData().mut_counts[type] > 0);
            }
            taxon = taxon->GetParent();            
        }

        return count;
    }

    /// Returns the total number of mutations of type @param type that occurred
    /// along @param taxon 's lineage.
    template <typename taxon_t>
    int CountMuts(Ptr<taxon_t> taxon, std::string type="substitution") {
        int count = 0;

        while (taxon) {
            count += taxon->GetData().mut_counts[type];
            taxon = taxon->GetParent();            
        }

        return count;
    }

    /// Returns the total number of mutations of type @param type that occurred
    /// along @param taxon 's lineage.
    template <typename taxon_t>
    int CountMuts(Ptr<taxon_t> taxon, emp::vector<std::string> types) {
        int count = 0;

        while (taxon) {
            for (std::string type : types) {
                count += taxon->GetData().mut_counts[type];
            }
            taxon = taxon->GetParent();            
        }

        return count;
    }

    /// Returns the total number of deleterious mutational steps that occurred
    /// along @param taxon 's lineage. (a change from parent to child taxon counts
    /// as a single step, regardless of the number of mutations that happened at
    /// that time point)
    template <typename taxon_t>
    int CountDeleteriousSteps(Ptr<taxon_t> taxon) {
        int count = 0;
        Ptr<taxon_t> parent = taxon->GetParent();

        while (parent) {
            if (taxon->GetData().fitness.GetMean() < parent->GetData().fitness.GetMean()) {
                count++;
            }
            taxon = parent;            
            parent = taxon->GetParent();               
        }

        return count;
    }

    /// Returns the total number of changes in phenotype that occurred
    /// along @param taxon 's lineage.
    template <typename taxon_t>
    int CountPhenotypeChanges(Ptr<taxon_t> taxon) {
        int count = 0; // Start with current phenotype
        Ptr<taxon_t> parent = taxon->GetParent();

        while (parent) {
            if (taxon->GetData().phenotype != parent->GetData().phenotype) {
                count++;
            }
            taxon = parent;            
            parent = taxon->GetParent();               
        }

        return count;
    }

    /// Returns the total number of unique phenotypes that occurred
    /// along @param taxon 's lineage.
    template <typename taxon_t>
    int CountUniquePhenotypes(Ptr<taxon_t> taxon) {
        int count = 0; 
        std::set<decltype(taxon->GetData().phenotype)> seen;

        while (taxon) {
            if (!Has(seen, taxon->GetData().phenotype)) {
                count++;
                seen.insert(taxon->GetData().phenotype);
            }
            taxon = taxon->GetParent();               
        }

        return count;
    }

};

#endif
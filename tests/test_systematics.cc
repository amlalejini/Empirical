#define CATCH_CONFIG_MAIN

#ifndef EMP_TRACK_MEM
#define EMP_TRACK_MEM
#endif

#undef NDEBUG
#define TDEBUG 1

#include "third-party/Catch/single_include/catch.hpp"

#include "Evo/Systematics.h"
#include <iostream>

TEST_CASE("Test Systematics", "[evo]")
{
  emp::Systematics<int> sys(true, true, true);

  std::cout << "\nAddOrg 25 (id1, no parent)\n";
  auto id1 = sys.AddOrg(25, nullptr, 0);
  std::cout << "\nAddOrg -10 (id2; parent id1)\n";
  auto id2 = sys.AddOrg(-10, id1, 6);
  std::cout << "\nAddOrg 26 (id3; parent id1)\n";
  auto id3 = sys.AddOrg(26, id1, 10);
  std::cout << "\nAddOrg 27 (id4; parent id2)\n";
  auto id4 = sys.AddOrg(27, id2, 25);
  std::cout << "\nAddOrg 28 (id5; parent id2)\n";
  auto id5 = sys.AddOrg(28, id2, 32);
  std::cout << "\nAddOrg 29 (id6; parent id5)\n";
  auto id6 = sys.AddOrg(29, id5, 39);
  

  std::cout << "\nRemoveOrg (id2)\n";
  sys.RemoveOrg(id1);
  sys.RemoveOrg(id2);

  double mpd = sys.GetMeanPairwiseDistance();
  std::cout << "MPD: " << mpd <<std::endl;
  REQUIRE(mpd == Approx(2.66666667));

  std::cout << "\nAddOrg 30 (id7; parent id1)\n";
  auto id7 = sys.AddOrg(30, id1, 6);
  std::cout << "\nAddOrg 31 (id8; parent id7)\n";
  auto id8 = sys.AddOrg(31, id7, 11);
  std::cout << "\nAddOrg 32 (id9; parent id8)\n";
  auto id9 = sys.AddOrg(32, id8, 19);
  sys.RemoveOrg(id7);
  sys.RemoveOrg(id8);

  REQUIRE(sys.GetEvolutionaryDistinctiveness(id3, 10) == 10);
  REQUIRE(sys.GetEvolutionaryDistinctiveness(id4, 25) == 21);
  REQUIRE(sys.GetEvolutionaryDistinctiveness(id5, 32) == 15);
  REQUIRE(sys.GetEvolutionaryDistinctiveness(id6, 39) == 22);
  REQUIRE(sys.GetEvolutionaryDistinctiveness(id6, 45) == 28);
  REQUIRE(sys.GetEvolutionaryDistinctiveness(id9, 19) == 19);

  std::cout << "\nAddOrg 33 (id10; parent id8)\n";
  auto id10 = sys.AddOrg(33, id8, 19);

  REQUIRE(sys.GetEvolutionaryDistinctiveness(id9, 19) == 13.5);
  REQUIRE(sys.GetEvolutionaryDistinctiveness(id10, 19) == 13.5);

  sys.RemoveOrg(id10);

  REQUIRE(sys.GetEvolutionaryDistinctiveness(id9, 19) == 19);

  id10 = sys.AddOrg(33, id8, 19);
  std::cout << "\nAddOrg 34 (id11; parent id9)\n";
  auto id11 = sys.AddOrg(34, id9, 22);
  std::cout << "\nAddOrg 35 (id12; parent id10)\n";
  auto id12 = sys.AddOrg(35, id10, 23);

  sys.RemoveOrg(id9);
  sys.RemoveOrg(id10);

  REQUIRE(sys.GetEvolutionaryDistinctiveness(id11, 26) == 20.5);
  REQUIRE(sys.GetEvolutionaryDistinctiveness(id12, 26) == 20.5);

  std::cout << "\nAddOrg 36 (id13; parent id12)\n";
  auto id13 = sys.AddOrg(36, id12, 27);
  std::cout << "\nAddOrg 37 (id14; parent id13)\n";
  auto id14 = sys.AddOrg(37, id13, 30);

  sys.RemoveOrg(id13);

  REQUIRE(sys.GetEvolutionaryDistinctiveness(id14, 33) == Approx(19.6667));

  std::cout << "\nAddOrg 38 (id15; parent id14)\n";
  auto id15 = sys.AddOrg(38, id14, 33);

  sys.RemoveOrg(id14);

  REQUIRE(sys.GetEvolutionaryDistinctiveness(id15, 33) == Approx(19.6667));

  std::cout << "\nAddOrg 39 (id16; parent id11)\n";
  auto id16 = sys.AddOrg(39, id11, 35);
  std::cout << "\nAddOrg 40 (id17; parent id11)\n";
  auto id17 = sys.AddOrg(40, id11, 35);

  REQUIRE(sys.GetEvolutionaryDistinctiveness(id16, 35) == Approx(20.7));
  REQUIRE(sys.GetEvolutionaryDistinctiveness(id17, 35) == Approx(20.7));

  std::cout << "\nAddOrg 41 (id18; parent id17)\n";
  auto id18 = sys.AddOrg(41, id17, 36);

  REQUIRE(sys.GetEvolutionaryDistinctiveness(id18, 37) == 14);

  REQUIRE(sys.GetTaxonDistinctiveness(id18) == Approx(1/6));
  REQUIRE(sys.GetBranchesToRoot(id18) == 2);
  REQUIRE(sys.GetDistanceToRoot(id18) == 6);

  REQUIRE(sys.GetTaxonDistinctiveness(id15) == Approx(1/7));
  REQUIRE(sys.GetBranchesToRoot(id15) == 1);
  REQUIRE(sys.GetDistanceToRoot(id15) == 7);
  REQUIRE(sys.GetPhylogeneticDiversity() == 17);
  REQUIRE(sys.GetAveDepth() == Approx(3.9));

  std::cout << "id1 = " << id1 << std::endl;
  std::cout << "id2 = " << id2 << std::endl;
  std::cout << "id3 = " << id3 << std::endl;
  std::cout << "id4 = " << id4 << std::endl;

  std::cout << "\nLineage:\n";
  sys.PrintLineage(id4);
  sys.PrintStatus();
}

template <typename PHEN_TYPE>
struct mut_landscape_info {
  std::unordered_map<std::string, int> mut_counts;
  double fitness;
  PHEN_TYPE phenotype;
};

TEST_CASE("Test Data Struct", "[evo]")
{

  emp::Systematics<int, mut_landscape_info<int> > sys(true, true, true);

}
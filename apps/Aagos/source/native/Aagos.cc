#include <iostream>

#include "base/vector.h"
#include "config/ArgManager.h"
#include "config/command_line.h"

#include "../AagosOrg.h"
#include "../AagosWorld.h"

int main(int argc, char* argv[])
{
  AagosConfig config;
  // Deal with loading config values via native interface (config file and command line args)
  config.Read("Aagos.cfg");
  auto args = emp::cl::ArgManager(argc, argv);
  if (args.ProcessConfigOptions(config, std::cout, "Aagos.cfg", "Aagos-macros.h") == false) exit(0);
  if (args.TestUnknown() == false) exit(0);  // If there are leftover args, throw an error.


  AagosWorld world(config);

  emp::Random & random = world.GetRandom();

  // Build a random initial population
  for (uint32_t i = 0; i < config.POP_SIZE(); i++) {
    AagosOrg next_org(config.NUM_BITS(), config.NUM_GENES(), config.GENE_SIZE());
    next_org.Randomize(random);
    std::cout << next_org.GetNumBits() << std::endl;
    world.Inject(next_org);
  }

  for (size_t gen = 0; gen < config.MAX_GENS(); gen++) {
    // Do mutations on the population.
    world.DoMutations(1);

    // Keep the best individual.
    emp::EliteSelect(world, 1, 1);

    // Run a tournament for the rest...
    emp::TournamentSelect(world, 5, config.POP_SIZE()-1);

    world.Update();

    if (gen % 100 == 0) {
      std::cout << gen << " : fitness=" << world.CalcFitnessID(0) << std::endl;
      world[0].Print();
    }
  }
}

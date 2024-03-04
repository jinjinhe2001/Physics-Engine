/*
AlloCore Example: Random

Description:
Examples of random number facilities in the rnd:: namespace.

Author:
Lance Putnam, 10/2012, putnam.lance@gmail.com
*/

#include "al/math/al_Random.hpp"
using namespace al;

int main() {
  // Lesson 1: Pseudorandom Integer Generators
  // =========================================================================
  /*
  Pretty much all "random" numbers generated by a computer are the result
  of very carefully crafted algorithms involving operations on integers.
  These algorithms generate a sequence of pseudo-random integer values.
  AlloCore provides three of these generators: MulLinCon, LinCon, and
  Tausworthe. Each involves a trade-off between execution speed and quality
  of "randomness" of result. MulLinCon is the fastest, but lowest quality.
  LinCon is slightly slower, but higher quality. Tausworthe is the slowest
  (but still relatively fast) and highest quality.
  */

  {
    // The default constructor "seeds" the generator with a random value
    rnd::MulLinCon rig;
    // rnd::LinCon rig;
    // rnd::Tausworthe rig;

    // We generate a random integer by using the function operator syntax:
    unsigned r = rig();

    // The result will be different each time we run the program
    printf("Random integer: %u\n", r);

    // We can also seed the generator so that it produces the exact same
    // pseudo-random sequence:
    rig.seed(17498);
    for (int i = 0; i < 6; ++i) printf("%u ", rig());
    printf("\n");

    // Here we repeat the same thing to prove it produces the same sequence
    rig.seed(17498);
    for (int i = 0; i < 6; ++i) printf("%u ", rig());
    printf("\n");
  }

  // Lesson 2: The Random class
  // =========================================================================
  /*
  While the preceeding generators do give us access to random numbers, in
  practice, we usually require real numbers rather than integers. This is the
  purpose of the Random class.
  */
  {
    // Random is a template class with its single parameter being one of
    // the random integer generators, MulLinCon, LinCon, or Tausworthe.
    rnd::Random<rnd::MulLinCon> rng1;
    rnd::Random<rnd::LinCon> rng2;
    rnd::Random<rnd::Tausworthe> rng3;

    // The default template parameter value is Tausworthe, giving the
    // highest quality random numbers, so we can declare it more simply:
    rnd::Random<> rng;

    rng.uniform();  // Returns random float in [0, 1)

    // We can also seed Random, like above, to repeat the same results
    rng.seed(48297);
    for (int i = 0; i < 6; ++i) printf("%g ", rng.uniform());
    printf("\n");
    rng.seed(48297);
    for (int i = 0; i < 6; ++i) printf("%g ", rng.uniform());
    printf("\n");

    // This is how we can randomly seed the generator
    rng.seed(rnd::seed());

    // Random numbers falling within a certain range can also be produced.
    // The return value has the same type as the argument(s).
    rng.uniform(10.);    // Returns random double in [0, 10)
    rng.uniform(10.f);   // Returns random float in [0, 10)
    rng.uniform(10);     // Returns random integer in [0, 10)
    rng.uniform(10, 1);  // Returns random integer in [1, 10)

    rng.normal();  // Returns standard normal (Gaussian) variate

    rng.prob();     // Returns true 50% of the time
    rng.prob(0.1);  // Returns true 10% of the time
    rng.prob(0.7);  // Returns true 70% fo the time

    float xy[2];
    rng.ball<2>(xy);  // Returns random point inside a unit circle

    float xyz[3];
    rng.ball<3>(xyz);  // Returns random point inside a unit sphere

    int arr[] = {0, 1, 2, 3, 4, 5, 6, 7};
    rng.shuffle(arr, 8);  // Shuffles elements of array

    printf("Shuffled array: ");
    for (int i = 0; i < 8; ++i) printf("%d ", arr[i]);
    printf("\n");
  }

  // Lesson 3: Easy Random Numbers
  // =========================================================================
  /*
  Sometime we just want a quick-n-easy way to do random things. For this
  reason, several global functions are available in the rnd:: namespace.
  Most of the methods in Random are available as global functions.
  */
  {
    rnd::uniform();     // Returns random float in [0, 1)
    rnd::uniform(10.);  // Returns random double in [0, 10)
    rnd::normal();      // Returns standard normal (Gaussian) variate
    rnd::prob(0.2);     // Returns true 20% of the time
  }
}

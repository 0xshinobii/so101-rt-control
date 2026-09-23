#include <gtest/gtest.h>
#include "arm_control/spsc_ring.hpp"

TEST(arm_control, queue)
{
  const int cap = 4;
  const int testing_number = 67;
  int out;

  arm_control::SpscRing<int> ring{cap};

  // assert push
  for (int i = 1; i <= cap; i++) {
    ASSERT_EQ(ring.push(i), true);
  }
  ASSERT_EQ(ring.push(testing_number), false);

  // pop 1 item and then push should be not drop
  ASSERT_EQ(ring.pop(out), true);
  ASSERT_EQ(out, 1); // FIFO
  ASSERT_EQ(ring.push(testing_number), true);

  // assert pop
  for (int i = 2; i <= cap; i++) {
    ASSERT_EQ(ring.pop(out), true);
    ASSERT_EQ(out, i); // FIFO
  }

  ASSERT_EQ(ring.pop(out), true);
  ASSERT_EQ(out, testing_number);
  ASSERT_EQ(ring.pop(out), false);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

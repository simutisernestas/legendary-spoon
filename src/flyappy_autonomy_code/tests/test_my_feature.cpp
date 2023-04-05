#include <gtest/gtest.h>

#include "flyappy_autonomy_code/flyappy.hpp"

TEST(MyFeature, Something)
{
    Flyappy flyappy;

    const int a = 1;
    EXPECT_EQ(a, 1);
}

TEST(Planner, CanPlanAStraithPathThroughEmptySpace)
{
    Flyappy flyappy{};
    // get position
    Vec pos;
    flyappy.getPos(pos);
    Vec goal{pos.x + 3, pos.y};
    flyappy.planPath(goal);
    std::vector<Node*> path;
    flyappy.getPlan(path);
    // check if plan is not empty
    EXPECT_FALSE(path.empty());
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

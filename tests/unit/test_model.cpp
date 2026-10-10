#include "support.h"
#include <cstdint>

TEST(Model, BorrowsTableAndInitializesFrames)
{
    manipulator_dh_joint joints[] = {joint(0), {MANIPULATOR_PRISMATIC,-1,-1,-1,-1}};
    manipulator_model m{};
    ASSERT_EQ(manipulator_model_init(&m,joints,2),MANIPULATOR_OK);
    EXPECT_EQ(m.joints,joints); EXPECT_EQ(m.joint_count,2u);
    matrix_near(m.world_base,identity(),0); matrix_near(m.last_tool,identity(),0);
    EXPECT_EQ(manipulator_model_validate(&m),MANIPULATOR_OK);
    joints[1].type=(manipulator_joint_type)99;
    EXPECT_EQ(manipulator_model_validate(&m),MANIPULATOR_INVALID_MODEL);
}

TEST(Model, InvalidSizesAndPointersPreserveModel)
{
    auto j=joint(); manipulator_model m{};
    ASSERT_EQ(manipulator_model_init(&m,&j,1),MANIPULATOR_OK);
    auto before=m;
    EXPECT_EQ(manipulator_model_init(nullptr,&j,1),MANIPULATOR_INVALID_ARGUMENT);
    EXPECT_EQ(manipulator_model_init(&m,nullptr,1),MANIPULATOR_INVALID_ARGUMENT);
    EXPECT_EQ(manipulator_model_init(&m,&j,0),MANIPULATOR_INVALID_MODEL);
    EXPECT_EQ(manipulator_model_init(&m,&j,SIZE_MAX),MANIPULATOR_INVALID_MODEL);
    EXPECT_EQ(manipulator_model_init(&m,&j,SIZE_MAX/sizeof(manipulator_transform)),MANIPULATOR_INVALID_MODEL);
    EXPECT_EQ(std::memcmp(&m,&before,sizeof(m)),0);
    EXPECT_EQ(manipulator_model_validate(nullptr),MANIPULATOR_INVALID_ARGUMENT);
    auto bad=m; bad.joints=nullptr;
    EXPECT_EQ(manipulator_model_validate(&bad),MANIPULATOR_INVALID_ARGUMENT);
}

TEST(Model, AllParametersMustBeFinite)
{
    for (auto field : {&manipulator_dh_joint::a, &manipulator_dh_joint::alpha,
                       &manipulator_dh_joint::d_offset, &manipulator_dh_joint::theta_offset}) {
        for (manipulator_real value : {std::numeric_limits<manipulator_real>::quiet_NaN(),
                             std::numeric_limits<manipulator_real>::infinity()}) {
            auto j=joint(); j.*field=value; manipulator_model m{};
            EXPECT_EQ(manipulator_model_init(&m,&j,1),MANIPULATOR_NONFINITE_INPUT);
        }
    }
}

TEST(Model, ValidatesBaseAndTool)
{
    auto j=joint(); manipulator_model m{};
    ASSERT_EQ(manipulator_model_init(&m,&j,1),MANIPULATOR_OK);
    m.world_base.m[0][0]=2;
    EXPECT_EQ(manipulator_model_validate(&m),MANIPULATOR_INVALID_TRANSFORM);
    m.world_base=identity(); m.last_tool.m[3][0]=(manipulator_real)1e-10;
    EXPECT_EQ(manipulator_model_validate(&m),MANIPULATOR_INVALID_TRANSFORM);
    m.last_tool=identity(); m.last_tool.m[1][3]=std::numeric_limits<manipulator_real>::quiet_NaN();
    EXPECT_EQ(manipulator_model_validate(&m),MANIPULATOR_NONFINITE_INPUT);
}

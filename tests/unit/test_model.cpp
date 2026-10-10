#include "support.h"
#include <cstdint>

TEST(Model, BorrowsTableAndInitializesFrames)
{
    k_dh_joint joints[] = {joint(0), {K_PRISMATIC,-1,-1,-1,-1}};
    k_model m{};
    ASSERT_EQ(k_model_init(&m,joints,2),K_OK);
    EXPECT_EQ(m.joints,joints); EXPECT_EQ(m.joint_count,2u);
    matrix_near(m.world_base,identity(),0); matrix_near(m.last_tool,identity(),0);
    EXPECT_EQ(k_model_validate(&m),K_OK);
    joints[1].type=(k_joint_type)99;
    EXPECT_EQ(k_model_validate(&m),K_INVALID_MODEL);
}

TEST(Model, InvalidSizesAndPointersPreserveModel)
{
    auto j=joint(); k_model m{};
    ASSERT_EQ(k_model_init(&m,&j,1),K_OK);
    auto before=m;
    EXPECT_EQ(k_model_init(nullptr,&j,1),K_INVALID_ARGUMENT);
    EXPECT_EQ(k_model_init(&m,nullptr,1),K_INVALID_ARGUMENT);
    EXPECT_EQ(k_model_init(&m,&j,0),K_INVALID_MODEL);
    EXPECT_EQ(k_model_init(&m,&j,SIZE_MAX),K_INVALID_MODEL);
    EXPECT_EQ(k_model_init(&m,&j,SIZE_MAX/sizeof(k_transform)),K_INVALID_MODEL);
    EXPECT_EQ(std::memcmp(&m,&before,sizeof(m)),0);
    EXPECT_EQ(k_model_validate(nullptr),K_INVALID_ARGUMENT);
    auto bad=m; bad.joints=nullptr;
    EXPECT_EQ(k_model_validate(&bad),K_INVALID_ARGUMENT);
}

TEST(Model, AllParametersMustBeFinite)
{
    for (auto field : {&k_dh_joint::a, &k_dh_joint::alpha,
                       &k_dh_joint::d_offset, &k_dh_joint::theta_offset}) {
        for (k_real value : {std::numeric_limits<k_real>::quiet_NaN(),
                             std::numeric_limits<k_real>::infinity()}) {
            auto j=joint(); j.*field=value; k_model m{};
            EXPECT_EQ(k_model_init(&m,&j,1),K_NONFINITE_INPUT);
        }
    }
}

TEST(Model, ValidatesBaseAndTool)
{
    auto j=joint(); k_model m{};
    ASSERT_EQ(k_model_init(&m,&j,1),K_OK);
    m.world_base.m[0][0]=2;
    EXPECT_EQ(k_model_validate(&m),K_INVALID_TRANSFORM);
    m.world_base=identity(); m.last_tool.m[3][0]=(k_real)1e-10;
    EXPECT_EQ(k_model_validate(&m),K_INVALID_TRANSFORM);
    m.last_tool=identity(); m.last_tool.m[1][3]=std::numeric_limits<k_real>::quiet_NaN();
    EXPECT_EQ(k_model_validate(&m),K_NONFINITE_INPUT);
}

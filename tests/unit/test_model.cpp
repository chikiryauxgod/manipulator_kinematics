#include "support.h"
#include <cstdint>

TEST(Model, BorrowsTableAndInitializesFrames)
{
    mk_dh_joint joints[] = {joint(0), {MK_PRISMATIC,-1,-1,-1,-1}};
    mk_model m{};
    ASSERT_EQ(mk_model_init(&m,joints,2),MK_OK);
    EXPECT_EQ(m.joints,joints); EXPECT_EQ(m.joint_count,2u);
    matrix_near(m.world_base,identity(),0); matrix_near(m.last_tool,identity(),0);
    EXPECT_EQ(mk_model_validate(&m),MK_OK);
    joints[1].type=(mk_joint_type)99;
    EXPECT_EQ(mk_model_validate(&m),MK_INVALID_MODEL);
}

TEST(Model, InvalidSizesAndPointersPreserveModel)
{
    auto j=joint(); mk_model m{};
    ASSERT_EQ(mk_model_init(&m,&j,1),MK_OK);
    auto before=m;
    EXPECT_EQ(mk_model_init(nullptr,&j,1),MK_INVALID_ARGUMENT);
    EXPECT_EQ(mk_model_init(&m,nullptr,1),MK_INVALID_ARGUMENT);
    EXPECT_EQ(mk_model_init(&m,&j,0),MK_INVALID_MODEL);
    EXPECT_EQ(mk_model_init(&m,&j,SIZE_MAX),MK_INVALID_MODEL);
    EXPECT_EQ(mk_model_init(&m,&j,SIZE_MAX/sizeof(mk_transform)),MK_INVALID_MODEL);
    EXPECT_EQ(std::memcmp(&m,&before,sizeof(m)),0);
    EXPECT_EQ(mk_model_validate(nullptr),MK_INVALID_ARGUMENT);
    auto bad=m; bad.joints=nullptr;
    EXPECT_EQ(mk_model_validate(&bad),MK_INVALID_ARGUMENT);
}

TEST(Model, AllParametersMustBeFinite)
{
    for (auto field : {&mk_dh_joint::a, &mk_dh_joint::alpha,
                       &mk_dh_joint::d_offset, &mk_dh_joint::theta_offset}) {
        for (mk_real value : {std::numeric_limits<mk_real>::quiet_NaN(),
                             std::numeric_limits<mk_real>::infinity()}) {
            auto j=joint(); j.*field=value; mk_model m{};
            EXPECT_EQ(mk_model_init(&m,&j,1),MK_NONFINITE_INPUT);
        }
    }
}

TEST(Model, ValidatesBaseAndTool)
{
    auto j=joint(); mk_model m{};
    ASSERT_EQ(mk_model_init(&m,&j,1),MK_OK);
    m.world_base.m[0][0]=2;
    EXPECT_EQ(mk_model_validate(&m),MK_INVALID_TRANSFORM);
    m.world_base=identity(); m.last_tool.m[3][0]=(mk_real)1e-10;
    EXPECT_EQ(mk_model_validate(&m),MK_INVALID_TRANSFORM);
    m.last_tool=identity(); m.last_tool.m[1][3]=std::numeric_limits<mk_real>::quiet_NaN();
    EXPECT_EQ(mk_model_validate(&m),MK_NONFINITE_INPUT);
}

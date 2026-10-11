#include "support.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <iomanip>
#include <sstream>

static std::string scientific(long double value)
{
    std::ostringstream text;
    text << std::scientific << std::setprecision(18) << value;
    return text.str();
}

class Family : public testing::TestWithParam<int> {};
TEST_P(Family, SpatialGeometryAgainstIndependentTrigonometry)
{
    const size_t n=(size_t)GetParam();
    std::vector<k_dh_joint> joints(n,joint());
    joints[0].alpha=(k_real)(pi/2); joints[0].d_offset=(k_real)0.1;
    for(size_t i=1;i<n;++i) joints[i].a=(k_real)(0.05L+0.003L*i);
    TestModel m{}; ASSERT_EQ(test_model_init(&m,joints.data(),n),K_OK);
    long double max_p_error=0, max_r_error=0;
    for(int sample=0;sample<32;++sample) {
        std::vector<k_real> q(n);
        for(size_t i=0;i<n;++i) q[i]=(k_real)(sample==0 ? 0 : std::sin((long double)(i+1)*(sample+1))*2);
        const auto q_before=q; const auto joints_before=joints;
        k_transform out{}; ASSERT_EQ(k_forward(&m,q.data(),n,&out),K_OK);
        long double phi=0,r=0,z=joints[0].d_offset,scale=std::fabs((long double)joints[0].d_offset);
        for(size_t i=1;i<n;++i) {
            phi+=q[i]; r+=(long double)joints[i].a*std::cos(phi);
            z+=(long double)joints[i].a*std::sin(phi); scale+=std::fabs((long double)joints[i].a);
        }
        long double c=std::cos((long double)q[0]),s=std::sin((long double)q[0]);
        long double p[]={c*r,s*r,z};
        long double rot[3][3]={{c*std::cos(phi),-c*std::sin(phi),s},
            {s*std::cos(phi),-s*std::sin(phi),-c},{std::sin(phi),std::cos(phi),0}};
        long double pmag=std::max({std::fabs(p[0]),std::fabs(p[1]),std::fabs(p[2])});
        long double p_tol=(K_USE_FLOAT?5e-5L:5e-12L)*std::max(1.0L,scale)+(K_USE_FLOAT?5e-6L:5e-13L)*pmag;
        for(int i=0;i<3;++i) {
            auto e=std::fabs((long double)out.m[i][3]-p[i]); max_p_error=std::max(max_p_error,e);
            EXPECT_LE(e,p_tol) << sample << ':' << i;
            for(int j=0;j<3;++j) {
                e=std::fabs((long double)out.m[i][j]-rot[i][j]); max_r_error=std::max(max_r_error,e);
                EXPECT_LE(e,rotation_tolerance) << sample << ':' << i << ',' << j;
            }
        }
        EXPECT_EQ(k_transform_validate(&out),K_OK);
        EXPECT_EQ(q,q_before);
        EXPECT_EQ(std::memcmp(joints.data(),joints_before.data(),n*sizeof(joints[0])),0);
    }
    RecordProperty("max_position_error",scientific(max_p_error));
    RecordProperty("max_rotation_error",scientific(max_r_error));
}
INSTANTIATE_TEST_SUITE_P(Sizes,Family,testing::Values(1,3,4,5,6,7,16,64));

using Vec = std::array<long double,3>;
static Vec combination(const Vec &a,long double x,const Vec &b,long double y)
{ return {a[0]*x+b[0]*y,a[1]*x+b[1]*y,a[2]*x+b[2]*y}; }
TEST_P(Family, GeneralMixedSpatialGeometryAndFrames)
{
    const size_t n=(size_t)GetParam();
    long double maximum_p=0,maximum_r=0;
    for(int sample=0;sample<24;++sample) {
        std::vector<k_dh_joint> js(n);
        std::vector<k_real> q(n);
        for(size_t i=0;i<n;++i) {
            long double k=(long double)((i+1)*(sample+1));
            js[i]={((i+sample)%3==0)?K_PRISMATIC:K_REVOLUTE,
                   (k_real)(2*std::sin(k)),(k_real)(pi*std::cos(k)),
                   (k_real)(3*std::cos(2*k)),(k_real)(pi*std::sin(3*k))};
            q[i]=(k_real)(2*std::cos(4*k));
        }
        TestModel m{}; ASSERT_EQ(test_model_init(&m,js.data(),n),K_OK);
        m.world_base={{{0,-1,0,(k_real)1.25},{1,0,0,-2},{0,0,1,3},{0,0,0,1}}};
        m.last_tool={{{0,0,1,2},{1,0,0,-1},{0,1,0,(k_real)0.5},{0,0,0,1}}};
        std::vector<k_transform> frames(n+2),work(n+2);
        ASSERT_EQ(k_forward_frames(&m,q.data(),n,frames.data(),frames.size(),work.data(),work.size()),K_OK);
        Vec x{0,1,0},y{-1,0,0},z{0,0,1},p{1.25L,-2,3};
        long double scale=1.25L+2+3+2+1+0.5L;
        for(size_t i=0;i<n;++i) {

            long double theta=js[i].theta_offset,d=js[i].d_offset;
            if(js[i].type==K_REVOLUTE) theta+=q[i]; else d+=q[i];
            const long double c=std::cos(theta),s=std::sin(theta);
            Vec u=combination(x,c,y,s),v=combination(x,-s,y,c);
            p=combination(p,1,z,d); p=combination(p,1,u,js[i].a);
            const long double ca=std::cos((long double)js[i].alpha),sa=std::sin((long double)js[i].alpha);
            x=u; y=combination(v,ca,z,sa); z=combination(v,-sa,z,ca);
            scale+=std::fabs((long double)js[i].a)+std::fabs((long double)d);
            Vec basis[3]={x,y,z};
            for(int row=0;row<3;++row) for(int col=0;col<3;++col) {
                auto error=std::fabs((long double)frames[i+1].m[row][col]-basis[col][row]);
                maximum_r=std::max(maximum_r,error);
                EXPECT_LE(error,rotation_tolerance);
            }
            long double pmag=std::max({std::fabs(p[0]),std::fabs(p[1]),std::fabs(p[2])});
            long double ptol=(K_USE_FLOAT?5e-5L:5e-12L)*std::max(1.0L,scale)+(K_USE_FLOAT?5e-6L:5e-13L)*pmag;
            for(int row=0;row<3;++row) {
                auto error=std::fabs((long double)frames[i+1].m[row][3]-p[row]);
                maximum_p=std::max(maximum_p,error); EXPECT_LE(error,ptol);
            }
        }
        p=combination(p,1,x,2);p=combination(p,1,y,-1);p=combination(p,1,z,0.5L);
        Vec tool_basis[3]={y,z,x};
        for(int row=0;row<3;++row) {
            for(int col=0;col<3;++col) EXPECT_NEAR(frames[n+1].m[row][col],tool_basis[col][row],rotation_tolerance);
            auto error=std::fabs((long double)frames[n+1].m[row][3]-p[row]);
            maximum_p=std::max(maximum_p,error);
            EXPECT_LE(error,(K_USE_FLOAT?5e-5L:5e-12L)*scale+(K_USE_FLOAT?5e-6L:5e-13L)*std::fabs(p[row]));
        }
    }
    RecordProperty("max_position_error",scientific(maximum_p));
    RecordProperty("max_rotation_error",scientific(maximum_r));
}

TEST(Forward, TwoNonplanarLinksHandCalculated)
{
    k_dh_joint js[]={{K_REVOLUTE,1,(k_real)(pi/2),3,0},
                     {K_REVOLUTE,2,(k_real)(pi/2),4,0}};
    k_real q[]={(k_real)(pi/2),(k_real)(pi/2)};
    TestModel m{}; ASSERT_EQ(test_model_init(&m,js,2),K_OK);
    k_transform out{};
    ASSERT_EQ(k_forward(&m,q,2,&out),K_OK);
    k_transform expected={{{0,1,0,4},{0,0,1,1},{1,0,0,5},{0,0,0,1}}};
    matrix_near(out,expected,rotation_tolerance);
}

TEST(Forward, MixedJointsAndOffsets)
{
    k_dh_joint js[]={{K_REVOLUTE,1,0,0,(k_real)(pi/4)},
                     {K_PRISMATIC,0,0,2,0}};
    k_real q[]={(k_real)(pi/4),3}; TestModel m{};
    ASSERT_EQ(test_model_init(&m,js,2),K_OK);
    k_transform out{}; ASSERT_EQ(k_forward(&m,q,2,&out),K_OK);
    k_transform expected={{{0,-1,0,0},{1,0,0,1},{0,0,1,5},{0,0,0,1}}};
    matrix_near(out,expected);
    js[0].theta_offset=0; q[0]=(k_real)(pi/2);
    ASSERT_EQ(test_model_init(&m,js,2),K_OK);
    k_transform shifted{}; ASSERT_EQ(k_forward(&m,q,2,&shifted),K_OK);
    matrix_near(out,shifted);
}

TEST(Forward, BaseToolAndFrameIndexing)
{
    k_dh_joint js[]={joint(1),joint(1),joint(1)};
    k_real q[]={(k_real)(pi/2),0,0}; TestModel m{};
    ASSERT_EQ(test_model_init(&m,js,3),K_OK);
    m.world_base={{{0,-1,0,1},{1,0,0,2},{0,0,1,3},{0,0,0,1}}};
    m.last_tool.m[0][3]=1;
    std::array<k_transform,6> frames,work;
    frames.fill(sentinel()); work.fill(sentinel());
    auto tail=frames[5]; auto worktail=work[5];
    ASSERT_EQ(k_forward_frames(&m,q,3,frames.data(),6,work.data(),6),K_OK);
    matrix_near(frames[0],m.world_base,0);
    for(int i=1;i<=3;++i) {
        EXPECT_NEAR(frames[i].m[0][3],1-i,rotation_tolerance);
        EXPECT_NEAR(frames[i].m[1][3],2,rotation_tolerance);
        EXPECT_NEAR(frames[i].m[2][3],3,rotation_tolerance);
    }
    EXPECT_NEAR(frames[4].m[0][3],-3,rotation_tolerance);
    k_transform tool{}; ASSERT_EQ(k_forward(&m,q,3,&tool),K_OK);
    matrix_near(tool,frames[4],0);
    same_bytes(frames[5],tail); same_bytes(work[5],worktail);
}

TEST(Forward, InputErrorsLeaveBothBuffersUntouched)
{
    auto j=joint(); k_real q=0; TestModel m{};
    ASSERT_EQ(test_model_init(&m,&j,1),K_OK);
    auto out=sentinel(),before=out;
    EXPECT_EQ(k_forward(nullptr,&q,1,&out),K_INVALID_ARGUMENT);
    EXPECT_EQ(k_forward(&m,nullptr,1,&out),K_INVALID_ARGUMENT);
    EXPECT_EQ(k_forward(&m,&q,1,nullptr),K_INVALID_ARGUMENT);
    EXPECT_EQ(k_forward(&m,&q,0,&out),K_SIZE_MISMATCH);
    EXPECT_EQ(k_forward(&m,&q,2,&out),K_SIZE_MISMATCH);
    q=std::numeric_limits<k_real>::infinity();
    EXPECT_EQ(k_forward(&m,&q,1,&out),K_NONFINITE_INPUT); same_bytes(out,before);
    std::array<k_transform,3> frames,work; frames.fill(sentinel()); work.fill(sentinel());
    auto fb=frames,wb=work;
    EXPECT_EQ(k_forward_frames(&m,&q,1,frames.data(),3,work.data(),3),K_NONFINITE_INPUT);
    q=0;
    EXPECT_EQ(k_forward_frames(&m,&q,1,frames.data(),2,work.data(),3),K_BUFFER_TOO_SMALL);
    EXPECT_EQ(k_forward_frames(&m,&q,1,frames.data(),3,work.data(),2),K_BUFFER_TOO_SMALL);
    EXPECT_EQ(k_forward_frames(&m,&q,1,nullptr,3,work.data(),3),K_INVALID_ARGUMENT);
    EXPECT_EQ(k_forward_frames(&m,&q,1,frames.data(),3,nullptr,3),K_INVALID_ARGUMENT);
    EXPECT_EQ(std::memcmp(frames.data(),fb.data(),sizeof(frames)),0);
    EXPECT_EQ(std::memcmp(work.data(),wb.data(),sizeof(work)),0);
    m.joint_count=SIZE_MAX;
    EXPECT_EQ(k_forward(&m,&q,SIZE_MAX,&out),K_INVALID_MODEL);
    same_bytes(out,before);
}

TEST(Forward, NumericErrorsAtEveryJointPreserveOutput)
{
    for(int bad=0;bad<3;++bad) for(auto type : {K_REVOLUTE,K_PRISMATIC}) {
        k_dh_joint js[]={joint(),joint(),joint()}; k_real q[]={0,0,0};
        js[bad].type=type; q[bad]=std::numeric_limits<k_real>::max();
        if(type==K_REVOLUTE) js[bad].theta_offset=q[bad]; else js[bad].d_offset=q[bad];
        TestModel m{}; ASSERT_EQ(test_model_init(&m,js,3),K_OK);
        auto out=sentinel(),before=out;
        EXPECT_EQ(k_forward(&m,q,3,&out),K_NUMERIC_ERROR); same_bytes(out,before);
        std::array<k_transform,5> frames,work; frames.fill(sentinel()); auto fb=frames;
        EXPECT_EQ(k_forward_frames(&m,q,3,frames.data(),5,work.data(),5),K_NUMERIC_ERROR);
        EXPECT_EQ(std::memcmp(frames.data(),fb.data(),sizeof(frames)),0);
    }
}

TEST(Forward, ToolOverflowPreservesCompleteOutput)
{
    auto j=joint(); TestModel m{}; k_real q=0;
    ASSERT_EQ(test_model_init(&m,&j,1),K_OK);
    m.world_base.m[0][3]=std::numeric_limits<k_real>::max();
    m.last_tool.m[0][3]=m.world_base.m[0][3];
    std::array<k_transform,3> frames,work; frames.fill(sentinel()); auto fb=frames;
    EXPECT_EQ(k_forward_frames(&m,&q,1,frames.data(),3,work.data(),3),K_NUMERIC_ERROR);
    EXPECT_EQ(std::memcmp(frames.data(),fb.data(),sizeof(frames)),0);
}

TEST(Forward, LongChainAndLargeAnglesControlled)
{
    std::vector<k_dh_joint> js(1024,joint((k_real)0.01));
    std::vector<k_real> q(1024,(k_real)0.03); TestModel m{};
    ASSERT_EQ(test_model_init(&m,js.data(),js.size()),K_OK);
    auto out=sentinel(),before=out;
    auto status=k_forward(&m,q.data(),q.size(),&out);
    ASSERT_TRUE(status==K_OK || status==K_NUMERIC_ERROR);
    if(status==K_OK) EXPECT_EQ(k_transform_validate(&out),K_OK); else same_bytes(out,before);
    js.resize(1); q.resize(1); q[0]=(k_real)1e6;
    ASSERT_EQ(test_model_init(&m,js.data(),1),K_OK);
    ASSERT_EQ(k_forward(&m,q.data(),1,&out),K_OK);
    EXPECT_NEAR(out.m[0][0],std::cos((double)q[0]),rotation_tolerance);
}

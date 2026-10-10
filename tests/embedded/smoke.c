#include <manipulator/forward.h>
#include <manipulator/transform.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static manipulator_dh_joint joints[64];
static manipulator_real q[64];
static manipulator_transform frames[66], work[66];
extern unsigned char _stack_bottom, _stack_top;

static void host_write(const char *text)
{
    register uintptr_t op __asm__("r0") = 4;
    register uintptr_t arg __asm__("r1") = (uintptr_t)text;
    __asm__ volatile("bkpt 0xab" : "+r"(op), "+r"(arg) : : "memory");
}
void smoke_exit(int code)
{
    const uint32_t args[2]={0x20026u,(uint32_t)code};
    register uintptr_t op __asm__("r0") = 0x20;
    register uintptr_t arg __asm__("r1") = (uintptr_t)args;
    __asm__ volatile("bkpt 0xab" : "+r"(op), "+r"(arg) : : "memory");
    for (;;) {}
}
static void write_number(unsigned value)
{
    char buffer[16],reverse[12]; unsigned n=0;
    do { reverse[n++]=(char)('0'+value%10); value/=10; } while(value);
    for(unsigned i=0;i<n;++i) buffer[i]=reverse[n-1-i];
    buffer[n]='\0'; host_write(buffer);
}
static void fill_stack(void)
{
    uintptr_t sp;
    __asm__ volatile("mrs %0, msp" : "=r"(sp));
    for (volatile unsigned char *p=&_stack_bottom; (uintptr_t)p<sp-256; ++p) *p=0xa5;
}
static unsigned stack_used(void)
{
    volatile unsigned char *p=&_stack_bottom;
    while (p<&_stack_top && *p==0xa5) ++p;
    return (unsigned)(&_stack_top-p);
}
static int near(manipulator_real a, manipulator_real b)
{
    manipulator_real error=a-b;
    return error<(manipulator_real)2e-5 && error>-(manipulator_real)2e-5;
}
int main(void)
{
    manipulator_model model;
    manipulator_transform out;
    const manipulator_real pi=(manipulator_real)3.14159265358979323846;
    joints[0]=(manipulator_dh_joint){MANIPULATOR_REVOLUTE,0,pi/2,(manipulator_real)0.1,0};
    joints[1]=(manipulator_dh_joint){MANIPULATOR_REVOLUTE,(manipulator_real)0.3,0,0,0};
    joints[2]=(manipulator_dh_joint){MANIPULATOR_REVOLUTE,(manipulator_real)0.2,0,0,0};
    q[2]=pi/2;
    if(manipulator_model_init(&model,joints,3)!=MANIPULATOR_OK || manipulator_forward(&model,q,3,&out)!=MANIPULATOR_OK) return 1;
    if(!near(out.m[0][3],(manipulator_real)0.3) || !near(out.m[1][3],0) || !near(out.m[2][3],(manipulator_real)0.3)) return 2;
    joints[0]=(manipulator_dh_joint){MANIPULATOR_REVOLUTE,1,0,0,0};
    joints[1]=(manipulator_dh_joint){MANIPULATOR_PRISMATIC,0,0,2,0}; q[0]=pi/2; q[1]=3;
    if(manipulator_model_init(&model,joints,2)!=MANIPULATOR_OK) return 3;
    model.world_base.m[2][3]=10; model.last_tool.m[0][3]=2;
    if(manipulator_forward_frames(&model,q,2,frames,66,work,66)!=MANIPULATOR_OK) return 4;
    if(!near(frames[3].m[0][3],0) || !near(frames[3].m[1][3],3) || !near(frames[3].m[2][3],15)) return 5;
    manipulator_transform before=frames[3];
    q[0]=(manipulator_real)NAN;
    if(manipulator_forward(&model,q,2,&frames[3])!=MANIPULATOR_NONFINITE_INPUT || memcmp(&before,&frames[3],sizeof(before))) return 6;
    q[0]=pi/2; model.last_tool.m[3][0]=(manipulator_real)1e-10;
    if(manipulator_forward(&model,q,2,&frames[3])!=MANIPULATOR_INVALID_TRANSFORM) return 7;
    model.last_tool.m[3][0]=0;
#if MANIPULATOR_USE_FLOAT
    volatile manipulator_real sub=FLT_TRUE_MIN;
    manipulator_real largest=FLT_MAX;
#else
    volatile manipulator_real sub=DBL_TRUE_MIN;
    manipulator_real largest=DBL_MAX;
#endif
    joints[0]=(manipulator_dh_joint){MANIPULATOR_PRISMATIC,0,0,0,0}; q[0]=sub;
    if(manipulator_model_init(&model,joints,1)!=MANIPULATOR_OK || manipulator_forward(&model,q,1,&out)!=MANIPULATOR_OK || out.m[2][3]!=sub) return 8;
    joints[0].d_offset=largest; q[0]=largest;
    if(manipulator_forward(&model,q,1,&out)!=MANIPULATOR_NUMERIC_ERROR) return 9;
    for(unsigned i=0;i<64;++i) { joints[i]=(manipulator_dh_joint){MANIPULATOR_REVOLUTE,(manipulator_real)0.1,0,0,0}; q[i]=0; }
    const unsigned sizes[]={3,6,64};
    for(unsigned k=0;k<3;++k) {
        unsigned n=sizes[k];
        if(manipulator_model_init(&model,joints,n)!=MANIPULATOR_OK) return 10;
        fill_stack();
        if(manipulator_forward_frames(&model,q,n,frames,66,work,66)!=MANIPULATOR_OK) return 11;
        unsigned used=stack_used();
        if(!near(frames[n+1].m[0][3],(manipulator_real)n*(manipulator_real)0.1)) return 12;
        host_write("STACK "); write_number(n); host_write(" "); write_number(used); host_write("\n");
    }
    host_write("SMOKE_OK\n");
    return 0;
}

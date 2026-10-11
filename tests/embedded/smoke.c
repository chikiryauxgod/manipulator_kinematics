#include "../support/model_setup.h"
#include <manipulator/inverse.h>
#include <manipulator/transform.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static k_dh_joint joints[64];
static k_real q[64];
static k_joint_config storage[64];
static k_transform unit;
static k_real ik_workspace[384], ik_output[64], scales[64], steps[64];
static k_transform frames[66], work[66];
extern unsigned char _stack_bottom, _stack_top;

static void host_write(const char *text)
{
    register uintptr_t op __asm__("r0") = 4;
    register uintptr_t arg __asm__("r1") = (uintptr_t)text;
    __asm__ volatile("bkpt 0xab" : "+r"(op), "+r"(arg) : : "memory");
}
void smoke_exit(int code)
{
    const uint32_t args[2] = {0x20026u, (uint32_t)code};
    register uintptr_t op __asm__("r0") = 0x20;
    register uintptr_t arg __asm__("r1") = (uintptr_t)args;
    __asm__ volatile("bkpt 0xab" : "+r"(op), "+r"(arg) : : "memory");
    for (;;)
    {
    }
}
static void write_number(unsigned value)
{
    char buffer[16], reverse[12];
    unsigned n = 0;
    do
    {
        reverse[n++] = (char)('0' + value % 10);
        value /= 10;
    } while (value);
    for (unsigned i = 0; i < n; ++i)
        buffer[i] = reverse[n - 1 - i];
    buffer[n] = '\0';
    host_write(buffer);
}
static void fill_stack(void)
{
    uintptr_t sp;
    __asm__ volatile("mrs %0, msp" : "=r"(sp));
    for (volatile unsigned char *p = &_stack_bottom; (uintptr_t)p < sp - 256; ++p)
        *p = 0xa5;
}
static unsigned stack_used(void)
{
    volatile unsigned char *p = &_stack_bottom;
    while (p < &_stack_top && *p == 0xa5)
        ++p;
    return (unsigned)(&_stack_top - p);
}
static int near(k_real a, k_real b)
{
    k_real error = a - b;
    return error < (k_real)2e-5 && error > -(k_real)2e-5;
}
int main(void)
{
    k_model model;
    if (k_transform_identity(&unit) != K_OK)
        return 20;
    k_transform out;
    const k_real pi = (k_real)3.14159265358979323846;
    joints[0] = (k_dh_joint){K_REVOLUTE, 0, pi / 2, (k_real)0.1, 0};
    joints[1] = (k_dh_joint){K_REVOLUTE, (k_real)0.3, 0, 0, 0};
    joints[2] = (k_dh_joint){K_REVOLUTE, (k_real)0.2, 0, 0, 0};
    q[2] = pi / 2;
    if (test_setup(&model, storage, 64, joints, 3, &unit, &unit) != K_OK ||
        k_forward(&model, q, 3, &out) != K_OK)
        return 1;
    if (!near(out.m[0][3], (k_real)0.3) || !near(out.m[1][3], 0) || !near(out.m[2][3], (k_real)0.3))
        return 2;
    joints[0] = (k_dh_joint){K_REVOLUTE, 1, 0, 0, 0};
    joints[1] = (k_dh_joint){K_PRISMATIC, 0, 0, 2, 0};
    q[0] = pi / 2;
    q[1] = 3;
    if (test_setup(&model, storage, 64, joints, 2, &unit, &unit) != K_OK)
        return 3;
    model.world_base.m[2][3] = 10;
    model.last_tool.m[0][3] = 2;
    if (k_forward_frames(&model, q, 2, frames, 66, work, 66) != K_OK)
        return 4;
    if (!near(frames[3].m[0][3], 0) || !near(frames[3].m[1][3], 3) || !near(frames[3].m[2][3], 15))
        return 5;
    k_transform before = frames[3];
    q[0] = (k_real)NAN;
    if (k_forward(&model, q, 2, &frames[3]) != K_NONFINITE_INPUT ||
        memcmp(&before, &frames[3], sizeof(before)))
        return 6;
    q[0] = pi / 2;
    model.last_tool.m[3][0] = (k_real)1e-10;
    if (k_forward(&model, q, 2, &frames[3]) != K_INVALID_TRANSFORM)
        return 7;
    model.last_tool.m[3][0] = 0;
#if K_USE_FLOAT
    volatile k_real sub = FLT_TRUE_MIN;
    k_real largest = FLT_MAX;
#else
    volatile k_real sub = DBL_TRUE_MIN;
    k_real largest = DBL_MAX;
#endif
    joints[0] = (k_dh_joint){K_PRISMATIC, 0, 0, 0, 0};
    q[0] = sub;
    if (test_setup(&model, storage, 64, joints, 1, &unit, &unit) != K_OK ||
        k_forward(&model, q, 1, &out) != K_OK || out.m[2][3] != sub)
        return 8;
    joints[0].d_offset = largest;
    q[0] = largest;
    if (test_setup(&model, storage, 64, joints, 1, &unit, &unit) != K_OK)
        return 21;
    if (k_forward(&model, q, 1, &out) != K_NUMERIC_ERROR)
        return 9;
    for (unsigned i = 0; i < 64; ++i)
    {
        joints[i] = (k_dh_joint){K_REVOLUTE, (k_real)0.1, 0, 0, 0};
        q[i] = 0;
    }
    const unsigned sizes[] = {3, 6, 64};
    for (unsigned k = 0; k < 3; ++k)
    {
        unsigned n = sizes[k];
        if (test_setup(&model, storage, 64, joints, n, &unit, &unit) != K_OK)
            return 10;
        fill_stack();
        if (k_forward_frames(&model, q, n, frames, 66, work, 66) != K_OK)
            return 11;
        unsigned used = stack_used();
        if (!near(frames[n + 1].m[0][3], (k_real)n * (k_real)0.1))
            return 12;
        host_write("STACK ");
        write_number(n);
        host_write(" ");
        write_number(used);
        host_write("\n");
    }
    for (unsigned i = 0; i < 64; ++i)
    {
        joints[i] = (k_dh_joint){K_PRISMATIC, 0, 0, 0, 0};
        q[i] = 0;
        scales[i] = 1;
        steps[i] = (k_real)0.1;
    }
    for (unsigned k = 0; k < 3; ++k)
    {
        unsigned n = sizes[k];
        if (test_setup(&model, storage, 64, joints, n, &unit, &unit) != K_OK)
            return 22;
        k_vec3 target = {0, 0, (k_real)n * (k_real)0.01};
        k_ik_options options = {100, 16, (k_real)2e-5, 1, (k_real)0.005, scales, steps, n};
        k_ik_report report;
        fill_stack();
        if (k_inverse_position(&model, &target, q, n, &options, ik_workspace, 384, ik_output, 64,
                               &report) != K_OK)
            return 23;
        unsigned used = stack_used();
        if (k_forward(&model, ik_output, n, &out) != K_OK || !near(out.m[2][3], target.z))
            return 24;
        host_write("IK_STACK ");
        write_number(n);
        host_write(" ");
        write_number(used);
        host_write("\n");
    }
    joints[0] = (k_dh_joint){K_REVOLUTE, 0, pi / 2, (k_real)0.1, 0};
    joints[1] = (k_dh_joint){K_REVOLUTE, (k_real)0.3, 0, 0, 0};
    joints[2] = (k_dh_joint){K_REVOLUTE, (k_real)0.2, 0, 0, 0};
    q[0] = (k_real)0.1;
    q[1] = (k_real)0.2;
    q[2] = (k_real)0.3;
    if (test_setup(&model, storage, 64, joints, 3, &unit, &unit) != K_OK)
        return 25;
    k_vec3 target = {(k_real)0.3, 0, (k_real)0.3};
    k_ik_options options = {300, 16, (k_real)2e-5, 1, (k_real)0.005, scales, steps, 3};
    if (k_inverse_position(&model, &target, q, 3, &options, ik_workspace, 384, ik_output, 64,
                           NULL) != K_OK)
        return 26;
    if (k_forward(&model, ik_output, 3, &out) != K_OK || !near(out.m[0][3], target.x) ||
        !near(out.m[2][3], target.z))
        return 27;
    host_write("IK_SMOKE_OK\n");
    host_write("SMOKE_OK\n");
    return 0;
}

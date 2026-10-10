#include <manipulator/forward.h>
#include <errno.h>
#include <float.h>
#include <locale.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef K_DRIVER_MAX_JOINTS
#define K_DRIVER_MAX_JOINTS 64
#endif
#define LINE_LIMIT 65536u

static char line_buffer[LINE_LIMIT + 2];
static k_dh_joint joints[K_DRIVER_MAX_JOINTS];
static k_real coordinates[K_DRIVER_MAX_JOINTS];

static char *token(char **cursor)
{
    char *p = *cursor;
    while (*p == ' ' || *p == '\t') ++p;
    if (!*p) { *cursor=p; return NULL; }
    char *start=p;
    while (*p && *p != ' ' && *p != '\t') ++p;
    if (*p) *p++='\0';
    *cursor=p;
    return start;
}
static int digit(char c) { return c >= '0' && c <= '9'; }
static const char *parse_real(char *text, k_real *out)
{
    if (!text) return "PARSE_ERROR";
    if (strcmp(text,"nan")==0) { *out=(k_real)NAN; return NULL; }
    if (strcmp(text,"inf")==0 || strcmp(text,"+inf")==0) { *out=(k_real)INFINITY; return NULL; }
    if (strcmp(text,"-inf")==0) { *out=-(k_real)INFINITY; return NULL; }
    const char *p=text;
    if (*p=='+' || *p=='-') ++p;
    size_t digits=0;
    while (digit(*p)) { ++p; ++digits; }
    if (*p=='.') {
        ++p;
        while (digit(*p)) { ++p; ++digits; }
    }
    if (!digits) return "PARSE_ERROR";
    if (*p=='e' || *p=='E') {
        ++p;
        if (*p=='+' || *p=='-') ++p;
        if (!digit(*p)) return "PARSE_ERROR";
        while (digit(*p)) ++p;
    }
    if (*p) return "PARSE_ERROR";
    char *end=NULL;
    errno=0;
#if K_USE_FLOAT
    k_real value=strtof(text,&end);
#else
    k_real value=strtod(text,&end);
#endif
    if (errno==ERANGE || !isfinite(value)) return "NUMBER_RANGE_ERROR";
    if (end==text || *end) return "PARSE_ERROR";
    *out=value;
    return NULL;
}
static const char *parse_count(char *text, size_t *out)
{
    if (!text || *text<'1' || *text>'9') return "PARSE_ERROR";
    size_t n=0;
    for (const char *p=text; *p; ++p) {
        if (!digit(*p)) return "PARSE_ERROR";
        unsigned d=(unsigned)(*p-'0');
        if (n > (SIZE_MAX-d)/10) return "REQUEST_TOO_LARGE";
        n=n*10+d;
    }
    if (n>K_DRIVER_MAX_JOINTS) return "REQUEST_TOO_LARGE";
    *out=n;
    return NULL;
}
static const char *status_name(k_status status)
{
    switch (status) {
    case K_OK: return "K_OK";
    case K_INVALID_ARGUMENT: return "K_INVALID_ARGUMENT";
    case K_INVALID_MODEL: return "K_INVALID_MODEL";
    case K_SIZE_MISMATCH: return "K_SIZE_MISMATCH";
    case K_BUFFER_TOO_SMALL: return "K_BUFFER_TOO_SMALL";
    case K_NONFINITE_INPUT: return "K_NONFINITE_INPUT";
    case K_NUMERIC_ERROR: return "K_NUMERIC_ERROR";
    case K_INVALID_TRANSFORM: return "K_INVALID_TRANSFORM";
    }
    return "INTERNAL_ERROR";
}
static const char *request(char *line, k_transform *result)
{
    char *cursor=line;
    char *command=token(&cursor);
    if (!command || strcmp(command,"FK")!=0) return "PARSE_ERROR";
    size_t n;
    const char *error=parse_count(token(&cursor),&n);
    if (error) return error;
    for (size_t i=0; i<n; ++i) {
        char *type=token(&cursor);
        if (!type) return "PARSE_ERROR";
        if (strcmp(type,"R")==0) joints[i].type=K_REVOLUTE;
        else if (strcmp(type,"P")==0) joints[i].type=K_PRISMATIC;
        else return "PARSE_ERROR";
        k_real *fields[]={&joints[i].a,&joints[i].alpha,&joints[i].d_offset,
                          &joints[i].theta_offset,&coordinates[i]};
        for (size_t f=0; f<5; ++f) {
            error=parse_real(token(&cursor),fields[f]);
            if (error) return error;
        }
    }
    k_transform base, tool;
    k_transform *matrices[]={&base,&tool};
    for (size_t t=0; t<2; ++t)
        for (size_t i=0; i<4; ++i)
            for (size_t j=0; j<4; ++j) {
                error=parse_real(token(&cursor),&matrices[t]->m[i][j]);
                if (error) return error;
            }
    if (token(&cursor)) return "PARSE_ERROR";
    k_model model;
    k_status status=k_model_init(&model,joints,n);
    if (status!=K_OK) return status_name(status);
    model.world_base=base; model.last_tool=tool;
    status=k_forward(&model,coordinates,n,result);
    return status==K_OK ? NULL : status_name(status);
}
static int respond(size_t length, int too_long, int bad_byte, int has_lf)
{
    if (has_lf && length && line_buffer[length-1]=='\r') --length;
    if (length>LINE_LIMIT) too_long=1;
    line_buffer[length]='\0';
    if (memchr(line_buffer,'\r',length)) bad_byte=1;
    k_transform out;
    const char *error=too_long ? "REQUEST_TOO_LARGE" : bad_byte ? "PARSE_ERROR" : request(line_buffer,&out);
    if (error) {
        if (printf("ERR %s\n",error)<0) return 0;
    } else {
        if (printf("OK")<0) return 0;
#if K_USE_FLOAT
        const int precision=FLT_DECIMAL_DIG;
#else
        const int precision=DBL_DECIMAL_DIG;
#endif
        for (size_t i=0; i<4; ++i) for (size_t j=0; j<4; ++j)
            if (printf(" %.*g",precision,(double)out.m[i][j])<0) return 0;
        if (putchar('\n')==EOF) return 0;
    }
    return fflush(stdout)==0;
}
int main(void)
{
    if (!setlocale(LC_NUMERIC,"C")) return 1;
    size_t length=0;
    int too_long=0, bad_byte=0, received=0, ch;
    while ((ch=getchar())!=EOF) {
        if (ch=='\n') {
            if (!respond(length,too_long,bad_byte,1)) return 1;
            length=0; too_long=0; bad_byte=0; received=0;
        } else {
            received=1;
            if ((ch<32 && ch!='\t' && ch!='\r') || ch>126) bad_byte=1;
            if (length<LINE_LIMIT+1) line_buffer[length++]=(char)ch;
            else too_long=1;
        }
    }
    if (ferror(stdin)) { fputs("stdin read failure\n",stderr); return 1; }
    if (received && !respond(length,too_long,bad_byte,0)) return 1;
    return 0;
}

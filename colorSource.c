#include "colorPrint.h"

void printf_red(const char* s)
{
    printf("\033[0m\033[1;31m%s\033[0m", s);
}
void printf_green(const char* s)
{
    printf("\033[0m\033[1;32m%s\033[0m", s);
}
void printf_yellow(const char* s)
{
    printf("\033[0m\033[1;33m%s\033[0m", s);
}
void printf_blue(const char* s)
{
    printf("\033[0m\033[1;34m%s\033[0m", s);
}
void printf_pink(const char* s)
{
    printf("\033[0m\033[1;35m%s\033[0m", s);
}
void printf_cyan(const char* s)
{
    printf("\033[0m\033[1;36m%s\033[0m", s);
}
#pragma once
#ifndef __MathwWorks__
#define __MathwWorks__
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Windows.h>
#include <stdarg.h>
#include <time.h>

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#ifndef IN_OUT
#define IN_OUT
#endif

#define mini_delta 0.000000001

#define pi 3.141592653

typedef double (*oneSymFunc)(double);

typedef double (*twoSymsFunc)(double, double);

typedef double (*threeSymsFunc)(double, double, double);

typedef double dim2Vec[2];

void ExFreeMemoryToNULL(PVOID* mem);

#endif
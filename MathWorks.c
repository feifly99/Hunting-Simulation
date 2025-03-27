#include "MathWorks.h"

double derivative(oneSymFunc f, double x)
{
	return (f(x + mini_delta) - f(x - mini_delta)) / ( 2 * mini_delta );
}
void ExFreeMemoryToNULL(PVOID* mem)
{
	free(*mem);
	*mem = NULL;
	return;
}

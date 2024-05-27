#include<stdio.h>
void ge(double* ray)
{
	ray[0] = 1;
	ray[1] = 2;
	ray[2] = 3;

}

int main()
{
	double ray[3];
	ge(ray);
	printf("%f", ray[0]);


	return 0;
}
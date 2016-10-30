#pragma once

#include <stdexcept>
using namespace std;

namespace BBW
{
	extern "C" { __declspec(dllexport) double* ComputeBBW(double *Vext, int *Text, double *Cext, int *Pext, int *BEext, int *CEext, int *binSizes, bool isTetra); }//, int *BEext, int *CEext, int *PEext, int *binSizes);}
//	extern "C" { __declspec(dllexport) double* pre_draw(double anim_ext, int *Usize, double *trans); }
	extern "C" { __declspec(dllexport) double* forward_kinematics(double *dQ, double *dT, double *Cext, int *BEext, int *binSizes); }
	extern "C" { __declspec(dllexport) double* matrix_multiplication(double *b_in, double *c_in, int *matSizes);}
//	extern "C" { __declspec(dllexport) double* send_Values(); }
	extern "C" __declspec(dllexport) int ReleaseMemory(double* pArray)
	{
		delete[] pArray;
		//delete[] Usize;
		return 0;
	}
}


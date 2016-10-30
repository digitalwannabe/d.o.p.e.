// If you don't have mosek installed and don't want to install it. Then
// uncomment the following six lines.  Don't use static library for this
// example because of Mosek complications
//
#define IGL_NO_MOSEK
#ifdef IGL_NO_MOSEK
#ifdef IGL_STATIC_LIBRARY
#undef IGL_STATIC_LIBRARY
#endif
#endif
#include <igl/boundary_conditions.h>
//#include <igl/colon.h>
#include <igl/column_to_quats.h>
#include <igl/directed_edge_parents.h>
#include <igl/forward_kinematics.h>
//#include <igl/jet.h>
#include <igl/lbs_matrix.h>
#include <igl/deform_skeleton.h>
#include <igl/normalize_row_sums.h>
#include <igl/readDMAT.h>
#include <igl/readMESH.h>
#include <igl/readTGF.h>
//#include <igl/viewer/Viewer.h>
#include <igl/bbw/bbw.h>
//#include <igl/embree/bone_heat.h>
#include <igl/copyleft/cgal/remesh_self_intersections.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>

//#include "../tutorial_shared_path.h"

#include "main.h"

namespace BBW{

typedef 
  std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> >
  RotationList;

//const Eigen::RowVector3d sea_green(70./255.,252./255.,167./255.);
//int selected = 0;
//Eigen::MatrixXd V,W,U,C,M,Vex,Cex,Atemp;
//Eigen::MatrixXi T,F,Tex,Fex,BEex;
//Eigen::VectorXi P, Pex;
//RotationList pose;

using namespace Eigen;
using namespace std;

MatrixXd FillMatrix( double Arr[], int nRows, int nCols) {
	MatrixXd Md(nRows,nCols);
//	Matrix<double, Dynamic, Dynamic, RowMajor> Md;
//	Md.resize(nRows, nCols);
	for (int i = 0; i < nRows; i++) {
		for (int j = 0; j < nCols; j++) {
			Md(i, j) = Arr[i*nCols + j];
		}
	}
	return Md;

}

MatrixXi FillMatrix(int Arr[], int nRows, int nCols) {
	MatrixXi Mi(nRows, nCols);
//	Matrix<int, Dynamic, Dynamic, RowMajor> Mi;
//	Mi.resize(nRows, nCols);
	for (int i = 0; i < nRows; i++) {
		for (int j = 0; j < nCols; j++) {
			Mi(i, j) = Arr[i*nCols + j];
		}
	}
	return Mi;

}

VectorXi FillVector(int Arr[], int nRows) {
	VectorXi Vi(nRows);
	for (int i = 0; i < nRows; i++) {
		Vi(i) = Arr[i];
	}
	return Vi;

}

Affine3d Array2Transform(double Arr[], int nRows, int nCols, int bin) {
	Affine3d Md = Affine3d::Identity();
	for (int i = 0; i < nRows; i++) {
		for (int j = 0; j < nCols; j++) {
			Md.matrix()(i, j) = Arr[bin*16+i*nCols + j];
		}
	}
	return Md;

}

/*vector<Vector3d> Array2VV3D(double Arr[], int entries) {
	vector<Vector3d> Vd(entries);
	for (int i = 0; i < entries; i++) {
//		for (int j = 0; j < 3; j++) {
		Vd[i].x = Arr[i * 3];
		Vd[i].y = Arr[i * 3+1];
		Vd[i].z = Arr[i * 3+2];
	//		}
			
	}

	return Vd;
}
*/

double* matrix_multiplication(double *b_in, double *c_in, int *matSizes)
{
	MatrixXd A_out, B_in, C_in, U;
	B_in = FillMatrix(b_in, matSizes[0], matSizes[1]);
	C_in = FillMatrix(c_in, matSizes[1], matSizes[2]);
	// Compute deformation via LBS as matrix multiplication

	U = B_in*C_in;

	MatrixXd UT = U.transpose().eval();//transpose matrix to get correct ordered values (x,y,z) in vvvv
//	*Usize = UT.size();//set pointer Usize to number of entries in U
	double* retArr = new double[UT.size()];//create array pointer in correct size

	Map<Matrix<double, -1, -1, RowMajor>>(retArr, U.rows(), U.cols()) = U;//set array pointer with values from matrix
													 //		Map<double>(T, T.rows(), T.cols())=transf;
	return retArr;//return the array in the end
	delete[] retArr;
}
/*
void remesh_self_intersection(double *b_in, double *c_in, int *matSizes)
{
	MatrixXd A_out, B_in, C_in, U;
	MatrixXi I_in, I_out, IF, J, IM;
	B_in = FillMatrix(b_in, matSizes[0], matSizes[1]);
	C_in = FillMatrix(c_in, matSizes[1], matSizes[2]);
	// Compute deformation via LBS as matrix multiplication
//	const Eigen::PlainObjectBase<DerivedV> V

	igl::copyleft::cgal::RemeshSelfIntersectionsParam params;
//	params.stitch_all = true;

	U = B_in*C_in;

	igl::copyleft::cgal::remesh_self_intersections(A_out,
		I_in,
		params,	
		B_in,
		I_out,
		IF,J,IM);

	MatrixXd UT = U.transpose().eval();//transpose matrix to get correct ordered values (x,y,z) in vvvv
									   //	*Usize = UT.size();//set pointer Usize to number of entries in U
	double* retArr = new double[UT.size()];//create array pointer in correct size

	Map<Matrix<double, -1, -1, RowMajor>>(retArr, U.rows(), U.cols()) = U;//set array pointer with values from matrix
																		  //		Map<double>(T, T.rows(), T.cols())=transf;
//	return retArr;//return the array in the end
	delete[] retArr;
}
*/
double* forward_kinematics(double *dQ, double *dT, double *Cext, int *BEext, int *binSizes )
{
	int CCount = binSizes[0];
	int BECount = binSizes[1];

	Eigen::MatrixXi BoneIndices;
	Eigen::MatrixXd ControlPoints;
	Eigen::VectorXi Parents;

	ControlPoints = FillMatrix(Cext, CCount, 3);
	BoneIndices = FillMatrix(BEext, BECount, 2);


	// retrieve parents for forward kinematics
	igl::directed_edge_parents(BoneIndices, Parents);

	
	RotationList dQs(BoneIndices.rows());
	vector<Eigen::Vector3d> dTs(BoneIndices.rows());
	for (int e = 0; e<BoneIndices.rows(); e++)
	{
//		for (int i = 0; i < 4; i++)
//		{
		dQs[e] = Eigen::Quaterniond(dQ[e * 4 + 0], dQ[e * 4 + 1], dQ[e * 4 + 2], dQ[e * 4 + 3] );
		dTs[e] = Eigen::Vector3d(dT[e * 3], dT[e * 3 + 1], dT[e * 3 + 2]);
//		}
	}

	


	// Propogate relative rotations via FK to retrieve absolute transformations
	RotationList vQuaternion;
	vector<Vector3d> vTranslate;
	const int dim = ControlPoints.cols();
	MatrixXd TransformMatrix(BoneIndices.rows()*(dim + 1), dim);

	igl::forward_kinematics(ControlPoints, BoneIndices, Parents, dQs,dTs, TransformMatrix);
	

/*	for (int e = 0; e<BoneIndices.rows(); e++)
	{
		Affine3d a = Affine3d::Identity();//3x4 matrix
		a.translate(vTranslate[e]);
		a.rotate(vQuaternion[e]);
		//			a = Array2Transform(Transform, dim + 1, dim + 1, e);
		TransformMatrix.block(e*(dim + 1), 0, dim + 1, dim) =
			a.matrix().transpose().block(0, 0, dim + 1, dim);//store 4x3 transform in T
	}*/
	double* tMatrix = new double[BoneIndices.rows()*(dim + 1)*dim];

	MatrixXd Ttrans = TransformMatrix.transpose().eval();

	Map<Matrix<double,-1,-1,RowMajor>>(tMatrix, TransformMatrix.rows(), TransformMatrix.cols()) = TransformMatrix;

	return tMatrix;
	delete[] tMatrix;
}


double* ComputeBBW(double *Vertices, int *TriangleI, double *Cext, int *Pext, int *BEext, int *CEext, int *binSizes, bool isTetra) {//, int *PEext, int *binSizes ) {
		
		using namespace Eigen;
//		using namespace std;

		Eigen::MatrixXi BoneIndices, TriangleIndices, CageIndices;
		Eigen::MatrixXd VertexPoints, ControlPoints, LinearBlendMatrix, Weights;
		Eigen::VectorXi PointIndices;
		
		int vertexCount = binSizes[0];//assign names for readability
		int triangleCount = binSizes[1];
		int controlCount = binSizes[2];
		int pointCount = binSizes[3];
		int boneCount = binSizes[4];
		int cageEdgeCount = binSizes[5];
		int PECount = binSizes[6];
		

			//set matrix to values from vvvv

			if (isTetra) {
				ControlPoints = FillMatrix(Cext, controlCount, 3);
				VertexPoints = FillMatrix(Vertices, vertexCount, 3);
				TriangleIndices = FillMatrix(TriangleI, triangleCount, 4);
			} 
				else 
				{
					ControlPoints = FillMatrix(Cext, controlCount, 2);
					VertexPoints = FillMatrix(Vertices, vertexCount, 2);
					TriangleIndices = FillMatrix(TriangleI, triangleCount, 3);
				}
			
//			V = Map<Matrix<double, eleCount, 3, RowMajor>>(Vext);
			
//			U = VertexPoints;
			
//			igl::readTGF("hand.tgf", C, BE);
			
//			C = FillMatrix(Cext, CCount, 3);
			if (boneCount >= 1) {
				BoneIndices = FillMatrix(BEext, boneCount, 2);
			}

			if (pointCount >= 1) {
				PointIndices = FillVector(Pext, pointCount);
			}

			if (cageEdgeCount >= 1) {
				CageIndices = FillMatrix(CEext, cageEdgeCount, 2);
			}
			
			// List of boundary indices (aka fixed value indices into VV)
			VectorXi b;
			// List of boundary conditions of each weight function
			MatrixXd bc;
			igl::boundary_conditions(VertexPoints, TriangleIndices, ControlPoints, PointIndices, BoneIndices, CageIndices, b, bc);

			// compute BBW weights matrix
			igl::bbw::BBWData bbw_data;
			// only a few iterations for sake of demo
			bbw_data.active_set_params.max_iter = 8;//expose to VVVV!
			bbw_data.verbosity = 0;//expose to VVVV!?
			if (!igl::bbw::bbw(VertexPoints, TriangleIndices, b, bc, bbw_data, Weights))
			{
				return false;
			}
//		}
		// Normalize weights to sum to one
		igl::normalize_row_sums(Weights, Weights);
		// precompute linear blend skinning matrix
		igl::lbs_matrix(VertexPoints, Weights, LinearBlendMatrix);

//		MatrixXd Mtrans = M.transpose().eval();//transpose matrix to get correct ordered values (x,y,z) in vvvv
//		*Usize = UT.size();//set pointer Usize to number of entries in U
		double* retArr = new double[LinearBlendMatrix.size()];//create array pointer in correct size

		Map<Matrix<double, -1, -1, RowMajor>>(retArr, LinearBlendMatrix.rows(), LinearBlendMatrix.cols()) = LinearBlendMatrix;//set array pointer with values from matrix
		
		return retArr;
		delete[] retArr;
	}
/*

double* send_Values()
{

MatrixXd UT = M.transpose().eval();//transpose matrix to get correct ordered values (x,y,z) in vvvv
//	*Usize = UT.size();//set pointer Usize to number of entries in U
double* retArr = new double[UT.size()];//create array pointer in correct size

Map<MatrixXd>(retArr, UT.rows(), UT.cols()) = UT;//set array pointer with values from matrix
//		Map<double>(T, T.rows(), T.cols())=transf;
return retArr;//return the array in the end
delete[] retArr;
}



double* pre_draw(double anim_ext, int *Usize, double *trans)
{
	//	using namespace Eigen;
	//	using namespace std;

	//	if (viewer.core.is_animating)
	//	{
	Eigen::MatrixXi BE;
	// retrieve parents for forward kinematics
	igl::directed_edge_parents(BE, P);

	// Read pose as matrix of quaternions per row
	MatrixXd Q;
	igl::readDMAT("hand-pose.dmat", Q);
	igl::column_to_quats(Q, pose);
	assert(pose.size() == BE.rows());

	// Interpolate pose and identity
	RotationList anim_pose(pose.size());
	for (int e = 0; e<pose.size(); e++)
	{
		anim_pose[e] = pose[e].slerp(anim_ext, Quaterniond::Identity());
	}
	// Propogate relative rotations via FK to retrieve absolute transformations
	RotationList vQold, vQ;
	vector<Vector3d> vTold, vT;
	igl::forward_kinematics(C, BE, P, anim_pose, vQ, vT);
	const int dim = C.cols();
	MatrixXd T(BE.rows()*(dim + 1), dim);
	//		Affine3d a = Affine3d::Identity();

	//		vT = Array2VV3D(vText);
	//		vQ = Array2quats

	MatrixXd T_ext = FillMatrix(trans, BE.rows()*(dim + 1), dim);
	//		MatrixXd T_ext2 = T_ext.block(0,0,BE.rows()*(dim + 1), dim);

	for (int e = 0; e<BE.rows(); e++)
	{
		Affine3d a = Affine3d::Identity();
		a.translate(vT[e]);
		a.rotate(vQ[e]);
		//			a = Array2Transform(Transform, dim + 1, dim + 1, e);
		T.block(e*(dim + 1), 0, dim + 1, dim) =
			a.matrix().transpose().block(0, 0, dim + 1, dim);
	}



	// Compute deformation via LBS as matrix multiplication
	U = M*T;

	// Also deform skeleton edges
	MatrixXd CT;
	MatrixXi BET;
	igl::deform_skeleton(C, BE, T, CT, BET);



	//		*Transform = transf;


	MatrixXd UT = U.transpose().eval();//transpose matrix to get correct ordered values (x,y,z) in vvvv
	*Usize = UT.size();//set pointer Usize to number of entries in U
	double* retArr = new double[UT.size()];//create array pointer in correct size

	Map<MatrixXd>(retArr, UT.rows(), UT.cols()) = UT;//set array pointer with values from matrix
													 //		Map<double>(T, T.rows(), T.cols())=transf;



	return retArr;//return the array in the end 
	delete[] retArr;
}
*/


}
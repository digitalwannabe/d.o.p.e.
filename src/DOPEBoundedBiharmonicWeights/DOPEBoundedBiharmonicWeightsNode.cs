#region usings
using System;
using System.ComponentModel.Composition;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Reflection;
using System.Text;

using VVVV.PluginInterfaces.V1;
using VVVV.PluginInterfaces.V2;
using VVVV.Utils.VColor;
using VVVV.Utils.VMath;




using VVVV.Core.Logging;

using System.Runtime.InteropServices;
#endregion usings

namespace VVVV.Nodes
{
	
	
	
	
	#region PluginInfo
	[PluginInfo(Name = "BoundedBiharmonicWeights", Category = "DOPE", Help = "Automatic Weight Computation for Meshes", Author="digitalWannabe", Credits = "igl@ETH Zurich, Alec Jacobson (New York University), Ilya Baran (Disney Research, Zurich), Jovan Popovic (Adobe Systems, Inc.), Olga Sorkine (ETH Zurich), lichterloh",Tags = "Mesh, deform, character animation, blend weights, dope")]
	#endregion PluginInfo
	public class DOPEBoundedBiharmonicWeightsNode : IPluginEvaluate, IDisposable
    {
        


        #region fields & pins
        //		[Input("Input", DefaultValue = 1.0)]
        //		public ISpread<double> FInput;

        [Input("Input ", DefaultValue = 0.0)]
		public ISpread<Vector3D> FV;
		
		[Input("Triangle/Tetrahedron Indices", DefaultValue = 0)]
		public ISpread<int> FVi;
		
		[Input("2D/3D", DefaultValue = 0)]
		public ISpread<bool> FTet;
		
		[Input("Controls ", DefaultValue = 0)]
		public ISpread<Vector3D> FCV;
				
		[Input("Point Handle Indices", DefaultValue = -1)]
		public ISpread<int> FPHi;
		
		[Input("Bone Edge Indices", DefaultValue = -1)]
		public ISpread<int> FBEi;
		
		[Input("Cage Edge Indices", DefaultValue = -1)]
		public ISpread<int> FCEi;
		
//		[Input("Pseudo Edge Indices")]
//		public ISpread<int> FPEi;
			
		[Input("Bind", IsBang = true)]
		public ISpread<bool> FBind;	

		
		[Output("Blend Weight Matrix")]
		public ISpread<double> FLbsArray;

		[Import()]
		public ILogger FLogger;
		#endregion fields & pins
		
		//[MarshalAs(UnmanagedType.LPArray, SizeParamIndex=0)] 
		[System.Runtime.InteropServices.DllImport("BBW4vvvv.dll")]
        private static extern IntPtr ComputeBBW(double [] Vertices, int[] TI, double[] ControlV, int[] PointI, int[] BoneEdgeI, int[] CageEdgeI, int[] binSizes, bool isTetra);//, IntPtr PseudoEdgeI);
		
		[System.Runtime.InteropServices.DllImport("BBW4vvvv.dll")]
		private static extern int ReleaseMemory(IntPtr ptr);
	
		
		
		

		//called when data for any output pin is requested
		public void Evaluate(int SpreadMax)
		{


            

            int dim;
			int entries = FV.SliceCount;
			int entriesXYz;
			int tritet0123 = FVi.SliceCount;
			int tritets;
			int cPoints = FCV.SliceCount;
			int cPointsXYz = cPoints*3;
			
			 //use bin size info!!
//			int boneEdges = boneEdgesXY/2;
			
			
			
			int cageEdges = 0;
			int boneEdges = 0;
			int pointHandles = 0;			
			if (FCEi.All(c=>c>=0)) cageEdges = FCEi.SliceCount/2;
			if (FBEi.All(c=>c>=0)) boneEdges = FBEi.SliceCount/2;
			if (FPHi.All(c=>c>=0)) pointHandles = FPHi.SliceCount;
			
	
			
			if (FTet[0]) dim=3; else dim=2;
			
			entriesXYz =entries*dim;
			cPointsXYz = cPoints*dim;
			tritets = tritet0123/(dim+1);

            int lbsSize = entries * (dim + 1) * (boneEdges + pointHandles);

            FLbsArray.SliceCount = lbsSize;

            if (FBind[0])
            {
            double[] V = new double[entriesXYz];
			double[] C = new double[cPointsXYz];
			
			var help = new Helpers();
			
			
			if (FTet[0]) {			
				V = help.Vector3DToArray(V,FV);
				C = help.Vector3DToArray(C,FCV);			
			}else 
				{					
				V = help.Vector3DToArray2D(V,FV);
				C = help.Vector3DToArray2D(C,FCV);	
				}
			
			
			int[] binSizes = new int[7];
			binSizes[0]=entries;
			binSizes[1]=tritets;
			binSizes[2]=cPoints;
			binSizes[3]=pointHandles;
			binSizes[4]=boneEdges;
			binSizes[5]=cageEdges;
			binSizes[6]=tritets;
					
			int[] T = new int[tritet0123];
			T = FVi.ToArray();
			
			int[] BE = new int[FBEi.SliceCount];
			BE = FBEi.ToArray();
			
			int[] PH = new int[FPHi.SliceCount];
			PH = FPHi.ToArray();
			
			int[] CE = new int[FCEi.SliceCount];
			CE = FCEi.ToArray();
				
			try
			{			

			IntPtr lbs = ComputeBBW(V,T,C,PH,BE,CE,binSizes,FTet[0]);
			
			double[] lbsArr = new double[lbsSize];
			Marshal.Copy(lbs, lbsArr,0,lbsSize );
			
			
			for(int i=0; i<lbsSize;i++){
				FLbsArray[i]=lbsArr[i];
			}
				
			ReleaseMemory(lbs);
						
			}
//			catch (ex) {FLogger.Log(LogType.Debug, ex.Message);} 
						
			finally
			{
				
			}
			}
						
			//FLogger.Log(LogType.Debug, "hi tty!");
		}
		 public void Dispose()
         {
			//	Marshal.FreeHGlobal(Vptr);
         }
	}
	
	#region PluginInfo
	[PluginInfo(Name = "ForwardKinematics", Category = "DOPE", Help = "A Simple Forward Kinematic Solver for Bones", Author="digitalWannabe", Credits = "igl@ETH Zurich, Alec Jacobson (New York University), lichterloh",Tags = "Mesh, deform, character animation, bones, dope")]
	#endregion PluginInfo
	public unsafe class DOPEForwardKinematicsNode : IPluginEvaluate, IDisposable
	{

		
		#region fields & pins
		
		[Input("Controls ", DefaultValue = 0)]
		public ISpread<Vector3D> FCV;
		
		[Input("Bone Edge Indices")]
		public ISpread<int> FBEi;
		
		[Input("Quaternion Rotate ", DefaultValue = 1.0)]
		public ISpread<Vector4D> FRot;
		
		[Input("Translate ")]
		public ISpread<Vector3D> FTrans;
		
		[Output("Transform Out")]
		public ISpread<Matrix4x4> FTransform;
		

		[Import()]
		public ILogger FLogger;
		#endregion fields & pins
		
		//[MarshalAs(UnmanagedType.LPArray, SizeParamIndex=0)] 
		[System.Runtime.InteropServices.DllImport("BBW4vvvv.dll")]
        private static extern IntPtr ComputeBBW(IntPtr Vertices, IntPtr TI, IntPtr ControlV, IntPtr BoneEdgeI, IntPtr binSizes, bool isTetra);//, IntPtr PointI, IntPtr ControlEdgeI, IntPtr PseudoEdgeI, IntPtr binSizes );
		
		[System.Runtime.InteropServices.DllImport("BBW4vvvv.dll")]
        private static extern IntPtr pre_draw(double anim_ext, ref int size, IntPtr Trnsltptr );
		
		[System.Runtime.InteropServices.DllImport("BBW4vvvv.dll")]
		private static extern IntPtr forward_kinematics(IntPtr dQ, IntPtr dT, IntPtr Cext, IntPtr BEext, IntPtr binSizes);
		
		[System.Runtime.InteropServices.DllImport("BBW4vvvv.dll")]
		private static extern int ReleaseMemory(IntPtr ptr);
	

		
		//called when data for any output pin is requested
		public void Evaluate(int SpreadMax)
		{


			int cPoints = FCV.SliceCount;
			int cPointsXYZ = cPoints*3;
			int boneEdgesXY = FBEi.SliceCount;
			int boneEdges = boneEdgesXY/2;

			
			int[] binSizes = new int[2];
			binSizes[0]=cPoints;
			binSizes[1]=boneEdges;
			
			var help = new Helpers();
			
			double[] C = new double[cPointsXYZ];
			C = help.Vector3DToArray(C,FCV);		
			
			
			int[] BE = new int[boneEdgesXY];
			BE = FBEi.ToArray();
			
			double[] Q = new double[boneEdges*4];
			for (int i =0; i<boneEdges;i++){
				Q[i*4]=FRot[i].w;
                Q[i * 4+1] = FRot[i].x;
                Q[i * 4+2] = FRot[i].y;
                Q[i * 4+3] = FRot[i].z;
            }
            

			
			double[] T = new double[boneEdges*3];
			for (int i =0; i<boneEdges;i++){
				T[i*3]=FTrans[i].x;
                T[i * 3+1] = FTrans[i].y;
                T[i * 3+2] = FTrans[i].z;
            }
							
			
			IntPtr Qptr = Marshal.AllocHGlobal(boneEdges*4*sizeof(double));
			IntPtr Tptr = Marshal.AllocHGlobal(boneEdges*3*sizeof(double));
			IntPtr Cptr = Marshal.AllocHGlobal(cPointsXYZ*sizeof(double));
			IntPtr BEptr = Marshal.AllocHGlobal(boneEdgesXY*sizeof(int));	
			IntPtr Binptr = Marshal.AllocHGlobal(2*sizeof(int));
				
			try
			{
			Marshal.Copy(Q, 0, Qptr, boneEdges*4);
			Marshal.Copy(T, 0, Tptr, boneEdges*3);	
			Marshal.Copy(C, 0, Cptr, cPointsXYZ);
			Marshal.Copy(BE, 0, BEptr, boneEdgesXY);	
			Marshal.Copy(binSizes, 0, Binptr, 2);
				
			
			IntPtr tMatrix = forward_kinematics(Qptr,Tptr,Cptr,BEptr,Binptr);
			int tMatrixSize=3*4*boneEdges;
			double[] tMatrixArr = new double[tMatrixSize];
			Marshal.Copy(tMatrix, tMatrixArr,0,tMatrixSize );
			
				
			FTransform.SliceCount=boneEdges;
			for(int i=0; i<boneEdges;i++){
				Matrix4x4 mTemp = new Matrix4x4();
				mTemp.row1=new Vector4D(tMatrixArr[i*12],tMatrixArr[i*12+1],tMatrixArr[i*12+2],0.0);
				mTemp.row2=new Vector4D(tMatrixArr[i*12+3],tMatrixArr[i*12+4],tMatrixArr[i*12+5],0.0);
				mTemp.row3=new Vector4D(tMatrixArr[i*12+6],tMatrixArr[i*12+7],tMatrixArr[i*12+8],0.0);
				mTemp.row4=new Vector4D(tMatrixArr[i*12+9],tMatrixArr[i*12+10],tMatrixArr[i*12+11],1.0);				
				FTransform[i]=mTemp;					
			}
				
				
			ReleaseMemory(tMatrix);
			
			
			}
						
			finally
			{
				Marshal.FreeHGlobal(Qptr);
				Marshal.FreeHGlobal(Tptr);
				Marshal.FreeHGlobal(Cptr);
				Marshal.FreeHGlobal(BEptr);
				Marshal.FreeHGlobal(Binptr);
				
				
			}
			
			
			//FLogger.Log(LogType.Debug, "hi tty!");
		}
		 public void Dispose()
         {
			//	Marshal.FreeHGlobal(Vptr);
         }
	}

    #region PluginInfo
    [PluginInfo(Name = "MatrixMultiplication", Category = "Value", Help = "Multiply 2 Matrices", Author = "digitalWannabe", Credits = "igl@ETH Zurich, Alec Jacobson (New York University), Eigen, lichterloh", Tags = "matrix, eigen, dope")]
    #endregion PluginInfo
    public class ValueMatrixMultiplicationNode : IPluginEvaluate, IDisposable
    {


        #region fields & pins


        [Input("Values B", DefaultValue = 1.0, BinVisibility =PinVisibility.OnlyInspector)]
        public ISpread<ISpread<double>> F_B;

        [Input("Values C", DefaultValue = 1.0, BinVisibility = PinVisibility.OnlyInspector)]
        public ISpread<ISpread<double>> F_C;

        [Input("CommonSize", DefaultValue = 1.0)]
        public ISpread<int> F_Common;

        [Output("Values A", BinVisibility = PinVisibility.Hidden)]
        public ISpread<ISpread<double>> F_A;

        [Import()]
        public ILogger FLogger;
        #endregion fields & pins

        //[MarshalAs(UnmanagedType.LPArray, SizeParamIndex=0)] 

        [System.Runtime.InteropServices.DllImport("BBW4vvvv.dll")]
        private static extern IntPtr matrix_multiplication(IntPtr b_in, IntPtr c_in, IntPtr matSizes);

        [System.Runtime.InteropServices.DllImport("BBW4vvvv.dll")]
        private static extern int ReleaseMemory(IntPtr ptr);


        //called when data for any output pin is requested
        public void Evaluate(int SpreadMax)
        {
            SpreadMax = SpreadUtils.SpreadMax(F_B,F_C,F_Common);

            F_A.SliceCount = SpreadMax;

            for (int binID = 0; binID < SpreadMax; binID++)
            {


                int b_count = F_B[binID].SliceCount;
                int c_count = F_C[binID].SliceCount;
                int b_Size = b_count / F_Common[binID];
                int c_Size = c_count / F_Common[binID];
                int aSize = b_Size * c_Size;
                F_A[binID].SliceCount = aSize;


                int[] binSizes = new int[3];
                //			binSizes = IndicesToArray(binSizes,FBins);
                binSizes[0] = b_Size;
                binSizes[1] = F_Common[binID];
                binSizes[2] = c_Size;

                double[] B = new double[b_count];
                for (int i = 0; i < b_count; i++)
                {
                    B[i] = F_B[binID][i];
                }

                double[] C = new double[c_count];
                for (int i = 0; i < c_count; i++)
                {
                    C[i] = F_C[binID][i];
                }

                //			A = Matrix4x4ToArray3x4(A,FTrans);

                IntPtr Bptr = Marshal.AllocHGlobal(b_count * sizeof(double));
                IntPtr Cptr = Marshal.AllocHGlobal(c_count * sizeof(double));
                IntPtr Binptr = Marshal.AllocHGlobal(3 * sizeof(int));

                try
                {
                    Marshal.Copy(B, 0, Bptr, b_count);
                    Marshal.Copy(C, 0, Cptr, c_count);
                    Marshal.Copy(binSizes, 0, Binptr, 3);


                    IntPtr Aptr = matrix_multiplication(Bptr, Cptr, Binptr);
                    //			int aSize=Asize;
                    double[] aArr = new double[aSize];
                    Marshal.Copy(Aptr, aArr, 0, aSize);

                    //			FtMatrixArray.SliceCount=tMatrixSize;
                    for (int i = 0; i < aSize; i++)
                    {
                        F_A[binID][i] = aArr[i];
                    }

                    ReleaseMemory(Aptr);


                }

                finally
                {
                    Marshal.FreeHGlobal(Cptr);
                    Marshal.FreeHGlobal(Bptr);
                    Marshal.FreeHGlobal(Binptr);
                }
            }

            //FLogger.Log(LogType.Debug, "hi tty!");
        }
        public void Dispose()
        {
            //	Marshal.FreeHGlobal(Vptr);
        }
    }


    public class Helpers
	{
	public double[] Vector3DToArray(double[] V,ISpread<Vector3D> VertexSpread){
				int entries = VertexSpread.SliceCount;				
				for (int i=0; i<entries; i++){
				V[i*3]=VertexSpread[i].x;
				V[i*3+1]=VertexSpread[i].y;
				V[i*3+2]=VertexSpread[i].z;
			}
			return V;
		}
		
	public double[] Vector3DToArray2D(double[] V,ISpread<Vector3D> VertexSpread){
				int entries = VertexSpread.SliceCount;				
				for (int i=0; i<entries; i++){
				V[i*2]=VertexSpread[i].x;
				V[i*2+1]=VertexSpread[i].y;
			}
			return V;
		}
		
	public double[] Matrix4x4ToArray(double[] V,ISpread<Matrix4x4> Transform){
				int entries = Transform.SliceCount;				
				for (int i=0; i<entries; i++){
					double[] trans = Transform[i].Values;		
					for (int j=0; j<4; j++){
						for (int k=0; k<4; k++){			
						V[i*16+j*4+k]=trans[j*4+k];
						}			
				}		
			}
			return V;
		}
		
	public double[] Matrix4x4ToArray3x4(double[] V,ISpread<Matrix4x4> Transform){
				int entries = Transform.SliceCount;				
				for (int i=0; i<entries; i++){
					double[] trans = Transform[i].Values;
					for (int j=0; j<4; j++){
						for (int k=0; k<3; k++){
						V[i*12+j*3+k]=trans[j*4+k];
						}	
				}
			}
			return V;
		}
		
	public int[] IndicesToArray(int[] I,ISpread<int> IndexSpread){
				int entries = IndexSpread.SliceCount;
				for (int i=0; i<entries; i++){
				I[i]=IndexSpread[i];
			}
			return I;
		}
	
		
	}
}

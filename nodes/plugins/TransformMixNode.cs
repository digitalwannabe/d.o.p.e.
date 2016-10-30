#region usings
using System;
using System.ComponentModel.Composition;

using VVVV.PluginInterfaces.V1;
using VVVV.PluginInterfaces.V2;
using VVVV.Utils.VColor;
using VVVV.Utils.VMath;

using VVVV.Core.Logging;
#endregion usings

namespace VVVV.Nodes
{
	#region PluginInfo
	[PluginInfo(Name = "Mix", Category = "Transform", Author = "lichterloh", Help = "Mix 2 Transforms", Tags = "Mix")]
	#endregion PluginInfo
	public class TransformMixNode : IPluginEvaluate
	{
		#region fields & pins
		[Input("Input 1")]
		public ISpread<Matrix4x4> FInput;
		
		[Input("Input 2")]
		public ISpread<Matrix4x4> FInput2;
		
		[Input("Mix")]
		public ISpread<double> FMix;

		[Output("Output")]
		public ISpread<Matrix4x4> FOutput;

		[Import()]
		public ILogger FLogger;
		#endregion fields & pins

		//called when data for any output pin is requested
		public void Evaluate(int SpreadMax)
		{
			FOutput.SliceCount = SpreadMax;

			for (int i = 0; i < SpreadMax; i++)
				FOutput[i] = FInput[i] + FMix[i]*(FInput2[i]-FInput[i]);

			//FLogger.Log(LogType.Debug, "Logging to Renderer (TTY)");
		}
	}
}

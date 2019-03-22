using AirSimRpc;
using Emgu.CV;
using imageLidarVisualizer.Data;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace imageLidarVisualizer.Fusion
{
    /// <summary>
    /// Input to ImageLidarFuser
    /// </summary>
    internal sealed class ImageLidarData
    {
        internal readonly float[] Xs;


        internal readonly float[] Ys;


        internal readonly Mat Image;


        internal readonly CarState State;


        private const float c_CutOffZHigh = 1.0f;


        private const float c_CutOffZLow = -1.0f;

        private ImageLidarData(float[] Xs, float[] Ys, Mat Image, CarState State)
        {
            this.Xs = Xs;
            this.Ys = Ys;
            this.Image = Image;
            this.State = State;
        }

        /// <summary>
        /// Creates an input to the ImageLidar fuser by filtering and transforming sensordata
        /// </summary>
        /// <param name="sensorData"></param>
        /// <returns></returns>
        internal static ImageLidarData Create(SensorData sensorData)
        {
            Mat Image = ImageUtils.GetMat(sensorData.ImageData);

            int Size = sensorData.LidarData.Sum(A => A.PointCloud.Length);

            float[] XPoints = new float[Size];
            float[] YPoints = new float[Size];

            foreach (LidarData lidarData in sensorData.LidarData)
            {
                
                float[] Arr = lidarData.PointCloud;
                for (int i = 0; i < Arr.Length; i += 3)
                {
                    float X = Arr[i] - lidarData.Pose.Position.X;
                    float Y = (Arr[i + 1]) - lidarData.Pose.Position.Y;
                    float Z = Arr[i + 2] - lidarData.Pose.Position.Z;

                    XPoints[i] = X;
                    YPoints[i] = Y;
                }
            }
            return new ImageLidarData(XPoints, YPoints, Image, sensorData.StateData);
        }
    }
}

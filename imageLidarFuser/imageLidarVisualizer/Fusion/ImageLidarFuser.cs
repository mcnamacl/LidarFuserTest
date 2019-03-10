using AirSimRpc;
using Emgu.CV;
using Emgu.CV.CvEnum;
using imageLidarVisualizer.Data;
using imageLidarVisualizer.Map;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace imageLidarVisualizer.Fusion
{
    public class ImageLidarFuser : ISensorFuser<SensorData, ObstacleMap>
    {
        public ObstacleMap Fuse(SensorData Input)
        {
            LidarData lidarData = Input.LidarData;          // Lidar reading
            Mat Image = Input.ImageData.Img;                // OpenCV image
            CarState State = Input.StateData;               // Access to magnetometer and accelerometer

            ObstacleMap obstacleMap = new ObstacleMap();
            PredictedObject obj = new PredictedObject(0f, 0f, 1);
            List<PredictedObject> tmpList = new List<PredictedObject>();

            for (int i = 0; i < lidarData.PointCloud.Length - 1; i += 3)
            {
                obj = getNearestObj(lidarData.PointCloud[i], lidarData.PointCloud[i + 1], tmpList);
                if (obj == null)
                {
                    obj = new PredictedObject(lidarData.PointCloud[i], lidarData.PointCloud[i + 1], 1);
                    tmpList.Add(obj);
                }
                else 
                {
                    obj.X = obj.X + lidarData.PointCloud[i];
                    obj.Y = obj.Y + lidarData.PointCloud[i + 1];
                    obj.counter++;
                }
            }

            MessageBox.Show(tmpList.Count.ToString());

            for (int i = 0; i < tmpList.Count; i++)
            {
                Obstacle obstacle = new Obstacle(tmpList[i].MeanX, tmpList[i].MeanY, 0.5f);
                if (tmpList[i].MeanX < 0)
                {
                    obstacleMap.ObstaclesLeft.AddLast(obstacle);
                } else
                {
                    obstacleMap.ObstaclesRight.AddLast(obstacle);
                }
            }

            return obstacleMap;
        }

        internal PredictedObject getNearestObj(float x, float y, List<PredictedObject> objs)
        {
            PredictedObject obj = objs.Where((O) => O.MeanX <= x && O.MeanY <= y)
                .OrderBy((O) => O.MeanX).FirstOrDefault();
            if (obj != null && Math.Sqrt(Math.Pow(obj.MeanX - x, 2) + Math.Pow(obj.MeanY - y, 2)) < 1f)
            {
                return obj;
            } else
            {
                return null;
            }
        }

        internal class PredictedObject
        {
            internal float X, Y;
            internal int counter;

            internal float MeanX => X / counter;
            internal float MeanY => Y / counter;

            internal PredictedObject(float X, float Y, int counter)
            {
                this.X = X;
                this.Y = Y;
                this.counter = counter;
            }
        }

    }

}

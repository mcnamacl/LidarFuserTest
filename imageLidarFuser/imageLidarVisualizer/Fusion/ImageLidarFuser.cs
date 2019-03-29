using AirSimRpc;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using imageLidarVisualizer.Data;
using imageLidarVisualizer.Fusion.Odometry;
using imageLidarVisualizer.Map;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace imageLidarVisualizer.Fusion
{
    internal sealed class ImageLidarFuser : ISensorFuser<ImageLidarData, ObstacleMap>
    {

        public ImageLidarFuser()
        {
        }

        public ObstacleMap Fuse(ImageLidarData Input)
        {
 
            //Get predicted obstacles
            List<PredictedObstacle> predictedObstacles = GetPredictedObstacles(Input.Xs, Input.Ys);

            
            ObstacleMap obstacleMap = new ObstacleMap();

            
            //orientate the obstacles
            List<PredictedObstacle> rotatedObstacles = orientate(predictedObstacles, Input.State);

            rotatedObstacles = rotatedObstacles.OrderBy((O) => O.MeanY).ToList();

            obstacleMap = designateForStraight(obstacleMap, rotatedObstacles, predictedObstacles);

            Mat threshHoldImage = GetThresholdImageCone(Input.Image);

            return obstacleMap;
        }

        private List<PredictedObstacle> orientate(List<PredictedObstacle> obs, StateData State)
        {
            List<PredictedObstacle> rotatedObs = new List<PredictedObstacle>();
            float yaw = State.Yaw;

            foreach (PredictedObstacle obj in obs)
            {
                PredictedObstacle newObs = new PredictedObstacle(getX(yaw, obj.MeanX, obj.MeanY), getY(yaw, obj.MeanX, obj.MeanY), obj.ID);
                rotatedObs.Add(newObs);
            }
            return rotatedObs;
        }

        private float getX(float theta, float x, float y)
        {
            return (x * (float)Math.Cos(theta)) - (y * (float)Math.Sin(theta));
        }

        private float getY(float theta, float x, float y)
        {
            return (x * (float)Math.Sin(theta)) + (y * (float)Math.Cos(theta));
        }

        private ObstacleMap designateForStraight(ObstacleMap map, List<PredictedObstacle> obs, List<PredictedObstacle> original)
        {
            obs = obs.OrderBy((O) => O.ID).ToList();
            original = original.OrderBy((O) => O.ID).ToList();
            List<Obstacle> right = new List<Obstacle>();
            List<Obstacle> left = new List<Obstacle>();

            //assign left and right
            for (int i = 0; i < obs.Count(); i++)
            {
                Obstacle newObs = new Obstacle(original[i].MeanX, original[i].MeanY, 1f);
                if (obs[i].MeanY > 0)
                {
                    if (obs[i].MeanX > 0)
                    {
                        right.Add(newObs);
                    }
                    else
                    {
                        left.Add(newObs);
                    }
                }
            }

            right = right.OrderBy((O) => O.Y).ToList();
            left = left.OrderBy((O) => O.Y).ToList();

            foreach (Obstacle obj in right)
            {
                map.ObstaclesRight.AddLast(obj);
            }

            foreach (Obstacle obj in left)
            {
                map.ObstaclesLeft.AddLast(obj);
            }

            return map;
        }

        List<PredictedObstacle> GetPredictedObstacles(float[] Xs, float[] Ys)
        {
            List<PredictedObstacle> obs = new List<PredictedObstacle>();

            int counter = 0;
            for (int i = 0;i<Xs.Length;i++)
            {
                float X = Xs[i];
                float Y = Ys[i];
                PredictedObstacle obstacle = GetNearestObstacle(X, Y, obs);

                if (!(X < 0.2 && X > -0.2) && !(Y < 0.2 && Y > -0.2))
                {
                    if (obstacle == null)
                    {
                        obs.Add(new PredictedObstacle(X, Y, counter));
                        counter++;
                    }

                    else
                        obstacle.Add(X, Y);
                }
                
            }

            return obs;
        }

        internal PredictedObstacle GetNearestObstacle(float X, float Y, List<PredictedObstacle> obs)
        {
            return obs.Where((O) => Dist(O, X, Y) < MinDist)
                .OrderBy((O) => Dist(O, X, Y)).FirstOrDefault();
        }

        private float Dist(PredictedObstacle O, float X, float Y)
        {

            return (float)Math.Sqrt((X - O.MeanX) * (X - O.MeanX) + (Y - O.MeanY) * (Y - O.MeanY));
        }

        internal sealed class PredictedObstacle
        {
            private float X, Y;

            internal int ID;

            private int Count;

            internal float MeanX => X / Count;

            internal float MeanY => Y / Count;

            internal PredictedObstacle(float X, float Y, int ID)
            {
                this.ID = ID;
                this.X = X;
                this.Y = Y;
                Count = 1;
            }

            internal void Add(float X, float Y)
            {
                this.X = this.X + X;
                this.Y = this.Y + Y;
                Count++;
            }
        }

        public static Mat GetThresholdImageCone(Mat mat)
        {
            //Some bad OpenCV code
            Mat img = mat;

            Mat threshLow = new Mat();
            Mat threshHigh = new Mat();

            CvInvoke.InRange(img, new ScalarArray(new MCvScalar(0, 25, 50)),
                 new ScalarArray(new MCvScalar(34, 148, 199)), threshLow);
            CvInvoke.InRange(img, new ScalarArray(new MCvScalar(23, 164, 215)),
                 new ScalarArray(new MCvScalar(126, 202, 237)), threshHigh);

            Mat imgThresh = new Mat();
            CvInvoke.BitwiseOr(threshLow, threshHigh, imgThresh);

            Mat imgThreshBlur = new Mat();
            CvInvoke.MedianBlur(imgThresh, imgThreshBlur, 3);

            Mat imgEdge = new Mat();
            CvInvoke.Canny(imgThreshBlur, imgEdge, 80, 160);

            return imgThreshBlur;
            
            //CvInvoke.DrawContours(img, imgEdge, 1, new MCvScalar(255, 255, 255));
        }

        private enum Side
        { Left,
        Right,
        Centre
        };

        private const float MinDist = 1.3f;


        private int InitialStamp;


        private CarState InitialState;




    }

}

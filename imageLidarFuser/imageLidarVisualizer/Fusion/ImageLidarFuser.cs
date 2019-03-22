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
        private readonly IAirSimCarProxy m_proxy;

        public ImageLidarFuser(IAirSimCarProxy proxy)
        {
            m_proxy = proxy;
        }

        public ObstacleMap Fuse(ImageLidarData Input)
        {
 
            //Get predicted obstacles
            List<PredictedObstacle> predictedObstacles = GetPredictedObstacles(Input.Xs, Input.Ys);

            
            ObstacleMap obstacleMap = new ObstacleMap();
            // TODO - CategorizeObstacles - create obstacles and insert into left and right road
            // -> Add them to obstacleMap.LeftObstacles or obstacleMap.RightObstacles
            // -> You can use Computer Vision here
            // -> Think of useful heuristics for determining the road.

            predictedObstacles = predictedObstacles.OrderBy((O) => O.MeanY).ToList();

            //orientate the obstacles
            List<PredictedObstacle> rotatedObstacles = orientate(predictedObstacles);

            //assign obstacles left and right
            obstacleMap = sortLeftAndRight(rotatedObstacles, predictedObstacles);

            Mat threshHoldImage = GetThresholdImageCone(Input.Image);

            // Maybe TODO map ran through odometry to check for missed obstacles
            //ObstacleMap ReturnedMap = AddOdometricEstimation(obstacleMap);
            //InitialState = Input.State;

            return obstacleMap;
        }

        private List<PredictedObstacle> orientate(List<PredictedObstacle> obs)
        {
            float yaw = getYaw();
            float rotation = 45 * ((float)Math.PI / 180);
            List<PredictedObstacle> rotatedObs = new List<PredictedObstacle>();
            foreach (PredictedObstacle obj in obs)
            {
                PredictedObstacle newObs = new PredictedObstacle(getX((float)Math.Asin(yaw) - rotation, obj.MeanX, obj.MeanY), getY((float)Math.Asin(yaw) - rotation, obj.MeanX, obj.MeanY));
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
            return (x * (float)Math.Cos(theta)) + (y * (float)Math.Sin(theta));
        }

        private float getYaw()
        {
            return m_proxy.GetCarStateAsync().Result.Value.KinematicsEstimated.Orientation.Z; 
        }

        private ObstacleMap sortLeftAndRight(List<PredictedObstacle> obs, List<PredictedObstacle> original)
        {
            ObstacleMap map = new ObstacleMap();

            PredictedObstacle maxY = obs.LastOrDefault();
            PredictedObstacle minYLeft = obs.LastOrDefault();
            PredictedObstacle minYRight = obs.LastOrDefault();

            bool foundRight = false, foundLeft = false;

            foreach (PredictedObstacle obj in obs)
            {
                if (obj.MeanX > 0 && !foundRight)
                {
                    minYRight = obj;
                    foundRight = true;
                }
                else if (obj.MeanX < 0 && !foundLeft){
                    minYLeft = obj;
                    foundLeft = true;
                }
                if (foundLeft && foundRight)
                {
                    break;
                }
            }

            bool pos = false; //left -> true, right -> false

            if (maxY.MeanX < minYLeft.MeanX && Math.Abs(maxY.MeanX - minYLeft.MeanX) > 2)
            {
                pos = true; //left turn
                map = designateForTurn(pos, map, obs, minYLeft, maxY, original);
            } 
            else if (maxY.MeanX > minYRight.MeanX && Math.Abs(maxY.MeanX - minYRight.MeanX) > 2)
            {
                //right turn
                map = designateForTurn(pos, map, obs, minYRight, maxY, original);
            } else
            {
                map = designateForStraight(map, obs, original);
            }
            
            return map;
        }

        private ObstacleMap designateForStraight(ObstacleMap map, List<PredictedObstacle> obs, List<PredictedObstacle> original)
        {
            for (int i = 0; i < obs.Count(); i++)
            {
                Obstacle newObs = new Obstacle(original[i].MeanX, original[i].MeanY, 1f);
                if (obs[i].MeanX > 0)
                {
                    map.ObstaclesRight.AddLast(newObs);
                } else
                {
                    map.ObstaclesLeft.AddLast(newObs);
                }
            }
            return map;
        }

        private ObstacleMap designateForTurn(bool pos, ObstacleMap map, List<PredictedObstacle> obs, PredictedObstacle p0, PredictedObstacle p1, List<PredictedObstacle> original)
        {
            for (int i = 0; i < obs.Count(); i++) 
            {
                Obstacle newObs = new Obstacle(original[i].MeanX, original[i].MeanY, 1f);

                if (line(p0, p1, obs[i]) + 1 < 0) // righthand side
                {
                    map.ObstaclesRight.AddLast(newObs);
                }
                else if (line(p0, p1, obs[i]) + 1 > 0) // lefthand side
                {
                    map.ObstaclesLeft.AddLast(newObs);
                }
                else if (line(p0, p1, obs[i]) + 1 == 0 && pos)
                {
                    map.ObstaclesRight.AddLast(newObs);
                }
                else if (line(p0, p1, obs[i]) + 1 == 0 && !pos)
                {
                    map.ObstaclesLeft.AddLast(newObs);
                }
            }
            return map;
        }

        private double line(PredictedObstacle p0, PredictedObstacle p1, PredictedObstacle p2)
        {
            return (p1.MeanX - p0.MeanX) * (p2.MeanY - p0.MeanY) - (p2.MeanX - p0.MeanX) * (p1.MeanY - p0.MeanY);
        }

        private ObstacleMap AddOdometricEstimation(ObstacleMap obstacleMap)
        {
            int CurrentStamp = TimeUtils.GetTimeStamp();
            ObstacleMap EstimatedMap = OdometricEstimator.Run(obstacleMap, InitialStamp, CurrentStamp, InitialState);

            // TODO - reference predicted obstacles against actual new obstacles
            // to check if the old reading has obstacles that the new one has missed

            return EstimatedMap;
        }

        List<PredictedObstacle> GetPredictedObstacles(float[] Xs, float[] Ys)
        {
            List<PredictedObstacle> obs = new List<PredictedObstacle>();

            for (int i = 0;i<Xs.Length;i++)
            {
                float X = Xs[i];
                float Y = Ys[i];
                PredictedObstacle obstacle = GetNearestObstacle(X, Y, obs);

                if(!(X < 0.2 && X > -0.2) && !(Y  < 0.2 && Y > -0.2))
                {
                    if (obstacle == null)
                        obs.Add(new PredictedObstacle(X, Y));
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

            private int Count;

            internal float MeanX => X / Count;

            internal float MeanY => Y / Count;

            internal PredictedObstacle(float X, float Y)
            {
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

        private const float MinDist = 1.3f;


        private int InitialStamp;


        private CarState InitialState;




    }

}

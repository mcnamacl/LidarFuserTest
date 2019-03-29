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
            // TODO - CategorizeObstacles - create obstacles and insert into left and right road
            // -> Add them to obstacleMap.LeftObstacles or obstacleMap.RightObstacles
            // -> You can use Computer Vision here
            // -> Think of useful heuristics for determining the road.

            
            //orientate the obstacles
            List<PredictedObstacle> rotatedObstacles = orientate(predictedObstacles, Input.State);
            //rotatedObstacles = rotatedObstacles.Where(O => O.MeanY > 0).ToList();

            //HashSet<int> idSet = new HashSet<int>();
            //for (int i = 0; i < rotatedObstacles.Count(); i++)
            //{
            //    idSet.Add(rotatedObstacles[i].ID);
            //}

            //List<PredictedObstacle> newPredictedObs = new List<PredictedObstacle>();

            //for (int i = 0; i < predictedObstacles.Count(); i++)
            //{
            //    if (idSet.Contains(predictedObstacles[i].ID))
            //    {
            //        newPredictedObs.Add(predictedObstacles[i]);
            //    }
            //}

            rotatedObstacles = rotatedObstacles.OrderBy((O) => O.MeanY).ToList();

            //assign obstacles left and right
            obstacleMap = sortLeftAndRight(rotatedObstacles, predictedObstacles);

            Mat threshHoldImage = GetThresholdImageCone(Input.Image);

            // Maybe TODO map ran through odometry to check for missed obstacles
            //ObstacleMap ReturnedMap = AddOdometricEstimation(obstacleMap);
            //InitialState = Input.State;

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


            Side pos = Side.Right; //left -> true, right -> false

            if (maxY.MeanX < minYLeft.MeanX && Math.Abs(maxY.MeanX - minYLeft.MeanX) > 3)
            {
                pos = Side.Left; //left turn
                map = designateForTurn(pos, map, obs, minYRight, maxY, original);
            } 
            else if (maxY.MeanX > minYRight.MeanX && Math.Abs(maxY.MeanX - minYRight.MeanX) > 3)
            {
                //right turn
                MessageBox.Show(maxY.MeanX.ToString() + " " + maxY.MeanY.ToString() + " " + minYLeft.MeanX.ToString() + " " + minYLeft.MeanY.ToString() + " " + minYLeft.ID);

                map = designateForTurn(pos, map, obs, minYLeft, maxY, original);
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

        private ObstacleMap designateForTurn(Side pos, ObstacleMap map, List<PredictedObstacle> obs, PredictedObstacle p0, PredictedObstacle p1, List<PredictedObstacle> original)
        {
            obs = obs.OrderBy((O) => O.ID).ToList();
            original = original.OrderBy((O) => O.ID).ToList();
            for (int i = 0; i < obs.Count(); i++) 
            {
                Obstacle newObs = new Obstacle(original[i].MeanX, original[i].MeanY, 1f);
                Side side = line(p0, p1, obs[i]);
                if (side == Side.Right) // righthand side
                {
                    map.ObstaclesRight.AddLast(newObs);
                    //MessageBox.Show("Right " + obs[i].MeanX.ToString() + " " + obs[i].MeanY.ToString() + " " + obs[i].ID);
                } 
                else if (side == Side.Left) // lefthand side
                {
                    map.ObstaclesLeft.AddLast(newObs);
                    //MessageBox.Show("Left " + obs[i].MeanX.ToString() + " " + obs[i].MeanY.ToString() + " " + obs[i].ID);
                }
                else if (side == Side.Centre && pos == Side.Left)
                {
                    //MessageBox.Show("Right " + obs[i].MeanX.ToString() + " " + obs[i].MeanY.ToString() + " " + obs[i].ID);
                    map.ObstaclesLeft.AddLast(newObs);
                }
                else if (side == Side.Centre && pos == Side.Right)
                {
                    //MessageBox.Show("Left " + obs[i].MeanX.ToString() + " " + obs[i].MeanY.ToString() + " " + obs[i].ID);
                    map.ObstaclesRight.AddLast(newObs);
                }
            }
            return map;
        }

        //p0=min p0x = x1, p0y = y1, p1=max p1x = x2, p1y = y2, p2=point DO NOT KNOW IF POINT IS TO THE LEFT OR  RIGHT OF THE LINE 

        private Side line(PredictedObstacle p0, PredictedObstacle p1, PredictedObstacle p2)
        {
            double val = (p2.MeanX - p0.MeanX) * (p1.MeanY - p0.MeanY) - (p2.MeanY - p0.MeanY) * (p1.MeanX - p0.MeanX);
            if (val < 0.1 && val > -0.1)
            {
                return Side.Centre;
            }
            return (val > 0) ? Side.Right : Side.Left;
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

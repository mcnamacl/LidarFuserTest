using AirSimRpc;
using Emgu.CV;
using Emgu.CV.CvEnum;
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

        public ObstacleMap Fuse(ImageLidarData Input)
        {
 
            //Get predicted obstacles
            List<PredictedObstacle> predictedObstacles = GetPredictedObstacles(Input.Xs, Input.Ys).OrderBy(O => O.MeanY).ToList();

            
            ObstacleMap obstacleMap = new ObstacleMap();
            // TODO - CategorizeObstacles - create obstacles and insert into left and right road
            // -> Add them to obstacleMap.LeftObstacles or obstacleMap.RightObstacles
            // -> You can use Computer Vision here
            // -> Think of useful heuristics for determining the road.



            foreach(PredictedObstacle ob in predictedObstacles)
            {
                if(ob.MeanX < -0.5f || ob.MeanX > 0.5f)
                {
                    Obstacle newOb = new Obstacle(ob.MeanX, ob.MeanY, 1.0f);

                    if (newOb.X > 0)
                        obstacleMap.ObstaclesRight.AddLast(newOb);
                    else
                        obstacleMap.ObstaclesLeft.AddLast(newOb);
                }               
            }


            // Maybe TODO map ran through odometry to check for missed obstacles
            //ObstacleMap ReturnedMap = AddOdometricEstimation(obstacleMap);
            //InitialState = Input.State;

            return obstacleMap;
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

                if (obstacle == null)
                    obs.Add(new PredictedObstacle(X, Y));
                else
                    obstacle.Add(X, Y);
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

        private const float MinDist = 1.3f;


        private int InitialStamp;


        private CarState InitialState;

    }

}

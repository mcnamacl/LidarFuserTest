using AirSimRpc;
using imageLidarVisualizer.Map;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace imageLidarVisualizer.Fusion.Odometry
{

    internal static class OdometricEstimator
    {
        /// <summary>
        /// Predict where obstacles in initialMap will be at the currentstamp based on initalstate
        /// </summary>
        /// <param name="InitialMap"></param>
        /// <param name="InitialStamp"></param>
        /// <param name="CurrentStamp"></param>
        /// <param name="InitialState"></param>
        /// <returns></returns>
        public static ObstacleMap Run(ObstacleMap InitialMap, int InitialStamp, int CurrentStamp, CarState InitialState)
        {
            ObstacleMap Output = new ObstacleMap();

            // TODO - Predict where obstacles will be in map
            // -> You should cut off objects that are too far away from the car
            // -> consider what might be done if the timespan is too large.

            return Output;
        }
    }
}

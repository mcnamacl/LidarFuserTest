using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace imageLidarVisualizer.Map
{
    /// <summary>
    /// A grid showing obstacles relative to the car in the vecinity,
    /// SI units are used throughout
    /// </summary>
    public sealed class ObstacleMap
    {
        /// <summary>
        /// Obstacles believed to be to the left of the Car
        /// </summary>
        public readonly LinkedList<Obstacle> ObstaclesLeft;


        /// <summary>
        /// Obstacles believed to be to the right of the car
        /// </summary>
        public readonly LinkedList<Obstacle> ObstaclesRight;


        /// <summary>
        ///  The length of the Horizontal Map (W/E)
        /// </summary>
        public static readonly float SizeX = 25.0f;


        /// <summary>
        /// The Length of the Vertical Map (N/S)
        /// </summary>
        public static readonly float SizeY = 25.0f;



        /// <summary>
        /// How Wide is the car
        /// </summary>
        public static readonly float CarWidth = 1.5f;


        /// <summary>
        /// How Wide is the car
        /// </summary>
        public static readonly float CarHeight = 3.0f;

        public ObstacleMap()
        {
            ObstaclesLeft = new LinkedList<Obstacle>();
            ObstaclesRight = new LinkedList<Obstacle>();
        }

        /// <summary>
        /// Removes obstacles that are out of bounds of the map
        /// </summary>
        public void ClipObstacles()
        {
            ClipList(ObstaclesLeft);
            ClipList(ObstaclesRight);
        }

        private void ClipList(LinkedList<Obstacle> list)
        {
            foreach (Obstacle obstacle in list)
                if (IsInBounds(obstacle)) list.Remove(obstacle);
        }

        private bool IsInBounds(Obstacle obstacle)
        {
            return obstacle.X < SizeX && obstacle.X > 0
                && obstacle.Y < SizeY && obstacle.Y > 0;
        }
    }
}

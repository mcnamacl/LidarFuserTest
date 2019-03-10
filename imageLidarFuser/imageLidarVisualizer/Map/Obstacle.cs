using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace imageLidarVisualizer.Map
{
    /// <summary>
    /// Obstacle on a Grid Map
    /// </summary>
    public sealed class Obstacle
    {
        /// <summary>
        /// X position of the obstacle
        /// </summary>
        public float X { get; set; }


        /// <summary>
        /// Y Postition of the obstacle
        /// </summary>
        public float Y { get; set; }


        /// <summary>
        /// Rough estimate of the Radius of the object
        /// </summary>
        public float R { get; set; }



        /// <summary>
        /// Unique Identifier of an Obstacle
        /// </summary>
        public uint Id { get; private set; }

        //public Obstacle(float X, float Y, float R, uint Id)
        public Obstacle(float X, float Y, float R)
        {
            this.X = X;
            this.Y = Y;
            this.R = R;
            this.Id = Id;
        }
    }
}

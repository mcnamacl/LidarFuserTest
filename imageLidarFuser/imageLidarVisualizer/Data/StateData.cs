using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace imageLidarVisualizer.Data
{
    public class StateData
    {
        /// <summary>
        /// Yaw in radians from N (-) to West, (+) to east
        /// </summary>
        public readonly float Yaw;


        public readonly Vector2 LinearVelocity;


        public readonly Vector2 LinearAcceleration;


        public readonly Vector2 AngularVelocity;


        public readonly Vector2 AngularAcceleration;

        public StateData(float Yaw, Vector2 LinearVelocity, Vector2 LinearAcceleration,
            Vector2 AngularVelocity, Vector2 AngularAcceleration)
        {
            this.Yaw = Yaw;
            this.LinearVelocity = LinearVelocity;
            this.LinearAcceleration = LinearAcceleration;
            this.AngularVelocity = AngularVelocity;
            this.AngularAcceleration = AngularAcceleration;
        }
    }
}

/* Author(s) : Renaldas Sepaitis */

using AirSimRpc;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Car.Auto.Control.Actuators
{
    public sealed class CarActuator : IActuator<CarControls>
    {
        private readonly IAirSimCarProxy m_proxy;

        public CarActuator(IAirSimCarProxy proxy)
        {
            m_proxy = proxy;
        }

        public void Set(CarControls Controls)
        {
            m_proxy.SetCarControls(Controls);
        }

        public float Yaw()
        {
            return m_proxy.GetCarStateAsync().Result.Value.KinematicsEstimated.Orientation.Z;
        }
    }
}

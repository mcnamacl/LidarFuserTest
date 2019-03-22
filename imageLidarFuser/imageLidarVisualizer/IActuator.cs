/* Author(s) : Isaac Walker */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Car.Auto.Control.Actuators
{
    /// <summary>
    /// Interface for output to the car
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public interface IActuator<T>
    {
        void Set(T Controls);
        float Yaw();
    }
}

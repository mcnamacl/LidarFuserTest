using AirSimRpc;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace imageLidarVisualizer.Data
{
    public class SensorData : Data
    {
        public LidarData[] LidarData { get; set; }


        public byte[] ImageData { get; set; }


        public CarState StateData { get; set; }
    }
}

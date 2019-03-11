using Emgu.CV;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace imageLidarVisualizer.Data
{
    public class ImageData
    {
        public readonly byte[] Img;

        public ImageData(byte[] Img)
        {
            this.Img = Img;
        }
    }
}

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
        public readonly Mat Img;

        public ImageData(Mat Img)
        {
            this.Img = Img;
        }
    }
}

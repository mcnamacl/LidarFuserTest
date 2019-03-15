using Emgu.CV;
using Emgu.CV.CvEnum;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace imageLidarVisualizer.Data
{
    public static class ImageUtils
    {
        public static Mat GetMat(byte[] buffer)
        {
            Mat mat = new Mat();
            CvInvoke.Imdecode(buffer, ImreadModes.ReducedColor8, mat);
            return mat;
        }

        public static Bitmap ByteToImage(byte[] blob)
        {
            byte[] imageSource = blob;
            Bitmap image;
            using (MemoryStream stream = new MemoryStream(imageSource))
            {
                image = new Bitmap(stream);
            }
            return image;
        }
    }
}

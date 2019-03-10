using Emgu.CV;
using Emgu.CV.CvEnum;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace imageLidarVisualizer.Data
{
    public static class ImageUtils
    {
        public static unsafe Mat CreateMat(byte[] buffer)
        {
            fixed (byte* p = buffer)
            {
                IntPtr ptr = (IntPtr)p;
                Mat mat = new Mat(new int[] { 256, 144 }, DepthType.Cv8U, ptr);
                return mat;
            }
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

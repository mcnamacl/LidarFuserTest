﻿using AirSimRpc;
using Emgu.CV;
using Emgu.CV.CvEnum;
using imageLidarVisualizer.Data;
using imageLidarVisualizer.Fusion;
using imageLidarVisualizer.Map;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace imageLidarVisualizer
{
    public partial class Form1 : Form
    {
        private Bitmap m_image;


        private Bitmap[] frames;


        private readonly ISensorFuser<SensorData,ObstacleMap> m_fuser;


        private ObstacleMap m_map;


        private SensorData[] sensorDatas;


        private readonly float ScaleX, ScaleY;

        private readonly int OffsetX, OffsetY;

        private int DIndex = 0;

        public Form1()
        {
            InitializeComponent();
            ScaleX = panel2.Width / ObstacleMap.SizeX;
            ScaleY = panel2.Height / ObstacleMap.SizeY;
            OffsetX = panel2.Width / 2;
            OffsetY = panel2.Height / 2;
            int size = 3;
            sensorDatas = new SensorData[size];
            frames = new Bitmap[size];
            for (int i = 0; i < size; i++)
            {
                StreamReader reader = new StreamReader(i + ".txt");

                  LidarData lidarData = JsonConvert.DeserializeObject<LidarData>(reader.ReadLine());
                  byte[] imgData = JsonConvert.DeserializeObject<byte[]>(reader.ReadLine());
                  CarState stateData = JsonConvert.DeserializeObject<CarState>(reader.ReadLine());

                frames[i] = new Bitmap(ImageUtils.ByteToImage(imgData), panel1.Size);
                sensorDatas[i] = new SensorData()
                  {
                      LidarData = lidarData,
                      ImageData = new ImageData(ImageUtils.CreateMat(imgData)),
                      StateData = stateData
                  };
            }

            m_image = new Bitmap(Image.FromFile("car.png"), new Size((int)(ObstacleMap.CarWidth * ScaleX),(int)( ObstacleMap.CarHeight * ScaleY)));
            
            m_fuser = new ImageLidarFuser();
            panel1.Refresh();
            panel2.Refresh();
            
        }

        private void panel1_Paint(object sender, PaintEventArgs e)
        {

            Graphics g = panel1.CreateGraphics();
            g.DrawImage(frames[DIndex], 0, 0);
        }

        private void panel2_Paint(object sender, PaintEventArgs e)

        {
            Graphics g = this.panel2.CreateGraphics();
            float Rot = GetYaw(sensorDatas[0].LidarData.Pose.Orientation);

            DrawGrid(g);
            Image img = RotateBitmap(m_image, Rot);

            
            int X = OffsetX - (img.Width / 2);
            int Y = OffsetY - (img.Height/2);
            g.DrawImage(img, X, Y);

            if (m_map != null)
            {   
                LinkedList<Obstacle> r, l;
                r = m_map.ObstaclesRight;
                l = m_map.ObstaclesLeft;
                PaintRoad(r, g);
                PaintRoad(l, g);
            }

            DrawLidarPoints(g);
        }

        private void PaintRoad(LinkedList<Obstacle> r, Graphics g)
        {
            Obstacle prevOb = null;
            foreach (Obstacle ob in r)
            {
                if (prevOb != null)
                {
                    g.DrawLine(Pens.Black, (prevOb.X * ScaleX) + OffsetX, panel2.Height - ((prevOb.Y * ScaleY) + OffsetY),
                        (ob.X * ScaleX) + OffsetX, panel2.Height - ((ob.Y * ScaleY) + OffsetY));
                }
                int ROff = (int)((ob.R * ScaleX) / 2);
                int XPos = (int)(ob.X * ScaleX) + OffsetX -ROff;
                int YPos = panel2.Height - ((int)(ob.Y * ScaleY) + OffsetY + ROff);
                g.FillEllipse(Brushes.Orange, XPos, YPos, ROff * 2, ROff*2);
                prevOb = ob;
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            DIndex++;

            if (DIndex == 1)
                DIndex = 0;

            m_map = m_fuser.Fuse(sensorDatas[DIndex]);

            this.panel1.Refresh();
            this.panel2.Refresh();
            

            
        }

        private void label1_Click(object sender, EventArgs e)
        {
            
        }

        float GetYaw(QuaternionR q)
        {
            return (float)Math.Asin(2 * q.X * q.Y + 2 * q.Z * q.W);
        }

        // Return a bitmap rotated around its center.
        private Bitmap RotateBitmap(Bitmap bm, float angle)
        {
            // Make a Matrix to represent rotation
            // by this angle.
            Matrix rotate_at_origin = new Matrix();
            rotate_at_origin.Rotate(angle);

            // Rotate the image's corners to see how big
            // it will be after rotation.
            PointF[] points =
            {
        new PointF(0, 0),
        new PointF(bm.Width, 0),
        new PointF(bm.Width, bm.Height),
        new PointF(0, bm.Height),
    };
            rotate_at_origin.TransformPoints(points);
            float xmin, xmax, ymin, ymax;
            GetPointBounds(points, out xmin, out xmax,
                out ymin, out ymax);

            // Make a bitmap to hold the rotated result.
            int wid = (int)Math.Round(xmax - xmin);
            int hgt = (int)Math.Round(ymax - ymin);
            Bitmap result = new Bitmap(wid, hgt);

            // Create the real rotation transformation.
            Matrix rotate_at_center = new Matrix();
            rotate_at_center.RotateAt(angle,
                new PointF(wid / 2f, hgt / 2f));

            // Draw the image onto the new bitmap rotated.
            using (Graphics gr = Graphics.FromImage(result))
            {
                // Use smooth image interpolation.
                gr.InterpolationMode = InterpolationMode.High;

                // Clear with the color in the image's upper left corner.
                gr.Clear(bm.GetPixel(0, 0));

                //// For debugging. (It's easier to see the background.)
                //gr.Clear(Color.LightBlue);

                // Set up the transformation to rotate.
                gr.Transform = rotate_at_center;

                // Draw the image centered on the bitmap.
                int x = (wid - bm.Width) / 2;
                int y = (hgt - bm.Height) / 2;
                gr.DrawImage(bm, x, y);
            }

            // Return the result bitmap.
            return result;
        }

        // Find the bounding rectangle for an array of points.
        private void GetPointBounds(PointF[] points,
            out float xmin, out float xmax,
            out float ymin, out float ymax)
        {
            xmin = points[0].X;
            xmax = xmin;
            ymin = points[0].Y;
            ymax = ymin;
            foreach (PointF point in points)
            {
                if (xmin > point.X) xmin = point.X;
                if (xmax < point.X) xmax = point.X;
                if (ymin > point.Y) ymin = point.Y;
                if (ymax < point.Y) ymax = point.Y;
            }
        }

        private void DrawGrid(Graphics g)
        {
            int Lines = 10;
            int Width = panel2.Width/ Lines;
            int Height = panel2.Height / Lines;
            for (int i =0; i<= Lines; i++)
            {
                g.DrawLine(Pens.Aquamarine, new Point(0, i * Height), new Point(panel2.Width, i * Height));
                g.DrawLine(Pens.Aquamarine, new Point(i * Width, 0), new Point(i * Width, panel2.Height));
            }
        }


        private void DrawLidarPoints(Graphics g)
        {
            float[] fs = sensorDatas[DIndex].LidarData.PointCloud;
            
            for(int i=0;i<fs.Length;i+=3)
            {
                float RealN = fs[i];     // X axis on lidar reading
                float RealE = fs[i + 1]; // Y axis on lidar reading

                g.DrawEllipse(Pens.Blue, (RealN * ScaleX) + OffsetX , panel2.Height - ((RealE * ScaleY)+OffsetY), 2, 2);
            }
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace imageLidarVisualizer.Data
{
    /// <summary>
    /// Utilities for timekeeping 
    /// </summary>
    public static class TimeUtils
    {
        private static readonly DateTime s_initTime = DateTime.UtcNow;

        /// <summary>
        /// Gets the current timestamp of the system
        /// </summary>
        /// <returns></returns>
        public static int GetTimeStamp()
        {
            TimeSpan span = DateTime.UtcNow - s_initTime;
            return span.Seconds;
        }
    }
}

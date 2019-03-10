using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace imageLidarVisualizer.Data
{
    /// <summary>
    /// Abstract data that is contained within a blackboard
    /// </summary>
    public abstract class Data
    {
        private int m_timestamp = TimeUtils.GetTimeStamp();

        /// <summary>
        /// Gets the timestamp of the last time a the data has been written to.
        /// </summary>
        /// <returns></returns>
        public float GetTimestamp()
        {
            return m_timestamp;
        }

        /// <summary>
        /// Updates the timestamp
        /// </summary>
        public void UpdateTimestamp()
        {
            m_timestamp = TimeUtils.GetTimeStamp();
        }


    }
}

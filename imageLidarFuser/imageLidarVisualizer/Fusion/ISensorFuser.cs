using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace imageLidarVisualizer.Fusion
{
    /// <summary>
    /// Fuses input against a control
    /// </summary>
    /// <typeparam name="TIn"></typeparam>
    /// <typeparam name="TCtrl"></typeparam>
    /// <typeparam name="TOut"></typeparam>
    public interface ISensorFuser<TIn, TOut>
    {
        TOut Fuse(TIn Input);
    }

    public interface ISensorFuser<TIn, TInTwo, TOut>
    {
        TOut Fuse(TIn Input, TInTwo Controls);
    }

}

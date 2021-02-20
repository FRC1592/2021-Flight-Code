package frc.robot.lib1592.motion;

public class InterpolatingDouble1D {
	
	//Lower index from last interpolation cycle
	int mIdx = 0;
	//Data
	public final double[] mX;
	public final double[] mY;
	//Last valid index = length-1
	private final int mLastIdx;
	private boolean isXMonotonic = true;
	private boolean isYMonotonic = true;

	public InterpolatingDouble1D(double[] x, double[] y) {
		//Get length
		mLastIdx = Math.min(x.length - 1, y.length - 1);
		
		//Make sure x matches length
		if (x.length > mLastIdx+1) {
			System.err.println("X has fewer elements than y.  Truncating");
			mX = new double[mLastIdx+1];
			System.arraycopy(x, 0, mX, 0, mLastIdx+1);
		} else {
			mX = x;
		}
			
		//Make sure y matches length
		if (y.length > mLastIdx+1) {
			System.err.println("Y has fewer elements than x.  Truncating");
			mY = new double[mLastIdx+1];
			System.arraycopy(y, 0, mY, 0, mLastIdx+1);
		} else {
			mY = y;
		}
		
		//Warn if table is not monotonically increasing for both indices
		for (int i = 1; i <= mLastIdx; i++) {
			if (mX[i] <= mX[i-1]) {
				System.out.println("Warning: X is not monotonically increasing.  "
						+ "Interpolation will be problematic.");
				isXMonotonic = false;
				break;
			}
			
		}
		
		for (int i = 1; i <= mLastIdx; i++) {
			if (mY[i] <= mY[i-1]) {
//				System.out.println("Warning: Y is not monotonically increasing.  "
//						+ "Inverse interpolation will be problematic.");
				isYMonotonic = false;
				break;
			}
		}
	}
	
	/**
	 * Interpolate x/y LUT by x starting at the last search index
	 * @param   x
	 * @return  y
	 */
	public double interp(double x) {
		if (!isXMonotonic) System.err.println("X is non-monotonic.  Interpolation is problematic.");
		if (x < mX[mIdx] && mIdx > 0) {
			mIdx--;
			return interp(x);
		} else if (x > mX[mIdx+1] && mIdx < mLastIdx-1) {
			mIdx++;
			return interp(x);
		} else {
			double slope = (mY[mIdx+1] - mY[mIdx]) / (mX[mIdx+1] - mX[mIdx]);
			return mY[mIdx] + slope * (x - mX[mIdx]);
		}
	}
	
	/**
	 * Interpolate x/y LUT by y starting at the last search index
	 * @param   y
	 * @return  x
	 */
	public double invInterp(double y) {
		if (!isYMonotonic) System.err.println("Y is non-monotonic.  Inverse interpolation is problematic.");
		if (y < mY[mIdx] && mIdx > 0) {
			mIdx--;
			return invInterp(y);
		} else if (y > mY[mIdx+1] && mIdx < mLastIdx-1) {
			mIdx++;
			return invInterp(y);
		} else {
			double slope = (mX[mIdx+1] - mX[mIdx]) / (mY[mIdx+1] - mY[mIdx]);
			return mX[mIdx] + slope * (y - mY[mIdx]);
		}
	}

	public int length() {
		return mLastIdx+1;
	}
}

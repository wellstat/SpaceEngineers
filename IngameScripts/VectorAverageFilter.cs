public class VectorAverageFilter
{
    public int vectorIndex = 0;
    public Vector3D[] vectorArr = null;
    public Vector3D vectorSum = new Vector3D();

    public VectorAverageFilter(int size)
    {
        vectorArr = new Vector3D[size];
    }

    public void Filter(ref Vector3D vectorIn, out Vector3D vectorOut)
    {
        vectorSum -= vectorArr[vectorIndex];
        vectorArr[vectorIndex] = vectorIn;
        vectorSum += vectorArr[vectorIndex];
        vectorIndex++;
        if (vectorIndex >= vectorArr.Length)
        {
            vectorIndex = 0;
        }
        vectorOut = vectorSum / vectorArr.Length;
    }

    public void Set(ref Vector3D vector)
    {
        vectorSum = default(Vector3D);
        for (int i = 0; i < vectorArr.Length; i++)
        {
            vectorArr[i] = vector;
            vectorSum += vectorArr[i];
        }
    }
}
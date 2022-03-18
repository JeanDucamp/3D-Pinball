using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wall : MonoBehaviour
{
    [SerializeField]
    private float RecoveryFactor = 1.0f;
    [SerializeField]
    private bool DestroyZone = false;
    [SerializeField]
    private bool LaunchZone = false;

    public Vector3 GetNormalVector() { return gameObject.transform.TransformDirection(Vector3.up); }
    public Vector3 GetPos() { return gameObject.transform.position; }
    public float GetXScale() { return 10 * gameObject.transform.localScale.x; }
    public float GetZScale() { return 10 * gameObject.transform.localScale.z; }
    public Vector3 GetHalfScaledForward() { return 0.5f * gameObject.transform.TransformDirection(Vector3.forward) * GetZScale(); }
    public Vector3 GetHalfScaledRight() { return 0.5f * gameObject.transform.TransformDirection(Vector3.right) * GetXScale(); }
    public float GetRecoveryFactor() { return RecoveryFactor; }
    public bool GetDestroyZone() { return DestroyZone; }
    public bool GetLaunchZone() { return LaunchZone; }
    public void GetPointsPos(out Vector3 A, out Vector3 B, out Vector3 C, out Vector3 D)
    {
        Vector3 HalfScaleForward = GetHalfScaledForward();
        Vector3 HalfScaleRight = GetHalfScaledRight();
        Vector3 Position = GetPos();

        A = Position + HalfScaleForward + HalfScaleRight;
        B = Position + HalfScaleForward - HalfScaleRight;
        C = Position - HalfScaleForward - HalfScaleRight;
        D = Position - HalfScaleForward + HalfScaleRight;
    }
}



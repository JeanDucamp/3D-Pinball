using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Flip : MonoBehaviour
{
    private Transform Transform;
    [SerializeField]
    private float MaxAngle;
    [SerializeField]
    private float FlipperSpeed;
    [SerializeField]
    private float Origin;
    [SerializeField]
    private Wall Wall;
    private float LastZAngle;

    void Start() { Transform = gameObject.transform; }
    public void GoUp()
    {
        LastZAngle = Transform.eulerAngles.z;
        float ZAngle = Mathf.LerpAngle(LastZAngle, MaxAngle, Time.fixedDeltaTime * FlipperSpeed);
        Transform.eulerAngles = new Vector3(
                Transform.eulerAngles.x,
                Transform.eulerAngles.y,
                ZAngle);
    }
    public void GoDown()
    {
        LastZAngle = Transform.eulerAngles.z;
        float ZAngle = Mathf.LerpAngle(Transform.eulerAngles.z, Origin, Time.fixedDeltaTime * FlipperSpeed);
        Transform.eulerAngles = new Vector3(
                Transform.eulerAngles.x,
                Transform.eulerAngles.y,
                ZAngle);
    }
    public Wall GetWall() { return Wall; }
    public float GetAngularSpeed()
    {
        return Mathf.Abs((Transform.eulerAngles.z - LastZAngle + 540) % 360 - 180) * Mathf.Deg2Rad / Time.fixedDeltaTime;
    }
    public Vector3 GetAxis()
    {
        Vector3 FlipRight = Transform.TransformDirection(Vector3.right);
        float Sign = Vector3.Dot(FlipRight, Wall.GetPos() - Transform.position) > 0 ? 1 : -1;
        return Sign * FlipRight;
    }
    public float GetLeverArm(Vector3 FulcrumPos) { return Vector3.Project(FulcrumPos - Transform.position, Transform.TransformDirection(Vector3.right)).magnitude; }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Bumper : MonoBehaviour
{
    [SerializeField]
    private float RecoveryFactor = 1.0f;
    [SerializeField]
    private float RepulsionForceMagnitude = 1.0f;
    [SerializeField] 
    private int Score = 100;

    public Vector3 GetAxis() { return gameObject.transform.up; }
    public Vector3 GetPos() { return gameObject.transform.position; }
    public float GetRadius() { return 0.5f * gameObject.transform.localScale.x; }
    public float GetRecoveryFactor() { return RecoveryFactor; }
    public float GetRepulsionForceMagnitude() { return RepulsionForceMagnitude; }
    public int GetScore() { return Score; }
}

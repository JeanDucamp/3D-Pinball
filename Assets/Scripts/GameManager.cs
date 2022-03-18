using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class GameManager : MonoBehaviour
{
    public static GameManager Instance;

    private Wall[] Walls;
    private Bumper[] Bumpers;

    private bool CanThrow;

    [Header("UI")]
    [SerializeField]
    private Text TextScore;
    private int Score;
    [SerializeField]
    private Text TextBestScore;
    private int BestScore;

    [Header("Flippers")]
    [SerializeField]
    private Flip LeftFlip;

    [SerializeField]
    private Flip RightFlip;

    [Header("Ball")]
    [SerializeField]
    private GameObject PrefabBall;
    private GameObject Ball;
    [SerializeField]
    private Vector3 BallSpawnPos;

    private void Awake()
    {
        if (Instance != null) { DestroyImmediate(this); }   
        else { Instance = this; }
            
        GameObject[] WallsGameObjects = GameObject.FindGameObjectsWithTag("Wall");     // Not optimal
        int WallAmount = WallsGameObjects.Length;
        Walls = new Wall[WallAmount];
        for (int i = 0; i < WallAmount; i++) { Walls[i] = (WallsGameObjects[i].GetComponent<Wall>()); };

        GameObject[] BumpersGameObjects = GameObject.FindGameObjectsWithTag("Bumper"); // Not optimal
        int BumperAmount = BumpersGameObjects.Length;
        Bumpers = new Bumper[BumperAmount];
        for (int i = 0; i < BumperAmount; i++) { Bumpers[i] = (BumpersGameObjects[i].GetComponent<Bumper>()); };
    }
    void FixedUpdate()
    {
        if (Input.GetKey(KeyCode.LeftArrow)) { LeftFlip.GoUp(); }
        else { LeftFlip.GoDown(); }

        if (Input.GetKey(KeyCode.RightArrow)) { RightFlip.GoUp(); }
        else { RightFlip.GoDown(); }

        if (CanThrow && Input.GetKey(KeyCode.Space))
        {
            CanThrow = false;
            Ball.GetComponent<Ball>().Launch();
            UpdateScore(0);
        }

        if (Input.GetKey(KeyCode.R)) { ResetBall(); }
    }
    public void SpawnBall() { Ball = Instantiate(PrefabBall, BallSpawnPos, Quaternion.identity); }
    public void ResetBall()
    {
        if (Ball) { Destroy(Ball); }
        SpawnBall();
    }
    public void UpdateScore(int score)
    {
        if (score == 0)
        {
            Score = 0;
        }
        Score += score;
        TextScore.text = Score.ToString();
        if (Score > BestScore)
        {
            BestScore = Score;
            TextBestScore.text = BestScore.ToString();
            TextBestScore.color = Color.green;
        }
        else
        {
            TextBestScore.color = Color.white;
        }
    }
    public void ResetLauncher() { CanThrow = true; }
    public Wall[] GetWalls() { return Walls; }
    public Bumper[] GetBumpers() { return Bumpers; }
    public Flip GetLeftFlip() { return LeftFlip; }
    public Flip GetRightFlip() { return RightFlip; }
}

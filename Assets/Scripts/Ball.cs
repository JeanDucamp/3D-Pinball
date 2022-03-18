using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

public class Ball : MonoBehaviour
{
    private GameManager GameManager;

    [SerializeField]
    private float Damping = 0.0f;
    [SerializeField]
    private float Mass = 0.008f;
    [SerializeField]
    private Vector3 LaunchVelocity;
    private Vector3 GWithoutConsideringSimulationSize = new Vector3(0f, -9.81f, 0f);
    private Vector3 G;
    private float Radius;
    private Vector3 Position;
    private Vector3 Velocity;
    private float DT;
    private Vector3 EstimatedPos;
    private Vector3 DeltaPos;
    private bool Collision;
    private float DistBeforeCollision;
    private float TotalDistToTravel;
    private float DistToTravel;
    private Vector3 ImpactPos;
    private Vector3 ImpactNormal;
    private Vector3 RepulsionForce;
    private Vector3 FlipForce;
    private int MaxSimulationSteps = 100;
    private float RecoveryFactor = 1;
    private float RoundingAdjustment = 0.0000001f; // 10^-7

    Wall[] Walls;
    Bumper[] Bumpers;
    Flip LeftFlip;
    Flip RightFlip;

    void Start()
    {
        GameManager = GameManager.Instance;
        Position = gameObject.transform.position;
        Radius = .5f * gameObject.transform.localScale.x;
        G = GWithoutConsideringSimulationSize * Radius / .027f; // .027f is the usual size of a pinball ball

        Walls = GameManager.GetWalls();
        Bumpers = GameManager.GetBumpers();
        LeftFlip = GameManager.GetLeftFlip();
        RightFlip = GameManager.GetRightFlip();
    }
    void FixedUpdate()
    {
        FlipsForceCalculation();
        PositionEstimation();
        CollisionManagement();
        BallUpdate();
    }
    void FlipsForceCalculation()
    {
        FlipForceCalculation(LeftFlip);
        FlipForceCalculation(RightFlip);
    }
    void FlipForceCalculation(Flip Flip)
    {
        Wall FlipWall = Flip.GetWall();
        Vector3 FlipNormal = FlipWall.GetNormalVector();
        Vector3 WallPos = FlipWall.GetPos();

        if (CalcDistPointPlane(Position, WallPos, FlipNormal) >= Radius) { return; }; // Ball contacted flip borderless plane
        Vector3 PosToWall = WallPos - Position;
        bool BallInsideZBorders = Vector3.Project(PosToWall, FlipWall.GetHalfScaledForward()).magnitude < 0.5f * FlipWall.GetZScale();
        bool BallInsideXBorders = Vector3.Project(PosToWall, FlipWall.GetHalfScaledRight()).magnitude < 0.5f * FlipWall.GetXScale();

        if (!(BallInsideZBorders && BallInsideXBorders)) { return; }; //Ball inside flip borders
        float LeverArm = Flip.GetLeverArm(Position);
        float AngularSpeed = Flip.GetAngularSpeed();
        FlipForce = LeverArm * AngularSpeed * FlipWall.GetNormalVector();
    }
    void PositionEstimation()
    {
        DT = Time.fixedDeltaTime;
        Velocity = Velocity * (1 - Damping * DT) + G * DT;
        DeltaPos = Velocity * DT;
        TotalDistToTravel = DeltaPos.magnitude;
        DistToTravel = TotalDistToTravel;
        EstimatedPos = Position + DeltaPos;
    }
    void CollisionManagement()
    {
        bool TraveledAskedDist = false;
        for (int i = 0; i < MaxSimulationSteps; i++)
        {
            DistBeforeCollision = float.MaxValue;
            Collision = false;
            WallCollisionDetection();
            BumperCollisionDetection();
            ResolveCollision();
            TraveledAskedDist = DistToTravel < RoundingAdjustment;
            if (TraveledAskedDist && !Collision) { break; };
        }
    }
    void WallCollisionDetection()
    {
        bool WallCollided;
        
        for (int i = 0; i < Walls.Length; i++)
        {
            Wall Wall = Walls[i];

            WallCollided = false;

            Vector3 A;
            Vector3 B;
            Vector3 C;
            Vector3 D;

            Wall.GetPointsPos(out A, out B, out C, out D);

            //Frontal collision
            PlaneCollisionDetection(Wall, ref WallCollided);

            //Edge Collision
            EdgeCollisionDetection(A, B, Wall, ref WallCollided);
            EdgeCollisionDetection(B, C, Wall, ref WallCollided);
            EdgeCollisionDetection(C, D, Wall, ref WallCollided);
            EdgeCollisionDetection(D, A, Wall, ref WallCollided);

            //Angle Collision
            AngleCollisionDetection(A, Wall, ref WallCollided);
            AngleCollisionDetection(B, Wall, ref WallCollided);
            AngleCollisionDetection(C, Wall, ref WallCollided);
            AngleCollisionDetection(D, Wall, ref WallCollided);

            CheckWall(Wall, WallCollided) ;
        }
    }
    void CheckWall(Wall wall, bool WallCollided)
    {
        if (!WallCollided) { return; }
        if (wall.GetDestroyZone()) { GameManager.ResetBall(); }
        else if (wall.GetLaunchZone()) { GameManager.ResetLauncher(); }
    }
    bool PlaneCollisionDetection(Wall Wall, ref bool WallCollided)
    {
        Vector3 WallPos = Wall.GetPos();
        Vector3 WallNormal = Wall.GetNormalVector();
        float IncidenceFactor = Vector3.Dot(WallNormal, WallPos - Position);
        Vector3 WallPosWithWallRadius = IncidenceFactor < 0 ? WallPos + WallNormal * Radius : WallPos - WallNormal * Radius;
        if (CalcDistPointPlane(EstimatedPos, WallPos, WallNormal) >= Radius) { return false; } //Ball crossed borderless plane adapted to radius

        Vector3 CollisionPosWithBorderlessPlane;
        SegmentPlaneIntersection(Position, EstimatedPos, WallPosWithWallRadius, WallNormal, out CollisionPosWithBorderlessPlane);
        float DistCollisionPlane = CalcDistPointPlane(CollisionPosWithBorderlessPlane, WallPos, WallNormal);
        if (DistCollisionPlane < Radius)
        {
            CollisionPosWithBorderlessPlane += WallNormal.normalized * (Radius - DistCollisionPlane);
            for (int i = 0; i < MaxSimulationSteps; i++)
            {
                if (CalcDistPointPlane(CollisionPosWithBorderlessPlane, WallPos, WallNormal) > Radius) { break; };
                CollisionPosWithBorderlessPlane += WallNormal.normalized * RoundingAdjustment * i;
            }
        }
        Vector3 CollisionToWall = WallPos - CollisionPosWithBorderlessPlane;

        bool BallInsideZBorders = Vector3.Project(CollisionToWall, Wall.GetHalfScaledForward()).magnitude < 0.5f * Wall.GetZScale();
        bool BallInsideXBorders = Vector3.Project(CollisionToWall, Wall.GetHalfScaledRight()).magnitude < 0.5f * Wall.GetXScale();
        if (!(BallInsideZBorders && BallInsideXBorders)) { return false; } //Ball inside wall borders

        float TempDistBeforeCollision = (CollisionPosWithBorderlessPlane - Position).magnitude;
        if (TempDistBeforeCollision >= DistBeforeCollision) { return false; } //Collision will occur before the saved one

        Collision = true;
        WallCollided = true;
        DistBeforeCollision = TempDistBeforeCollision;
        ImpactPos = CollisionPosWithBorderlessPlane;
        ImpactNormal = IncidenceFactor < 0 ? WallNormal : -WallNormal;
        RecoveryFactor = Wall.GetRecoveryFactor();
        return true;
    }
    bool EdgeCollisionDetection(Vector3 Angle1, Vector3 Angle2, Wall Wall, ref bool WallCollided)
    {
        Vector3 Edge = Angle2 - Angle1;
        if (Mathf.Pow(Vector3.Dot(DeltaPos, Edge), 2) == Mathf.Pow(DeltaPos.magnitude * Edge.magnitude, 2)) { return false; }

        Vector3 NearestEdgePoint;
        Vector3 NearestDeltaPosPoint;
        if (!NearestPointsTwoLines(Angle1, Angle2, Position, EstimatedPos, out NearestEdgePoint, out NearestDeltaPosPoint)) { return false; }

        Vector3 EdgeToDeltaPos = NearestDeltaPosPoint - NearestEdgePoint;
        if (!(EdgeToDeltaPos.magnitude < Radius && Vector3.Dot(NearestEdgePoint - Angle2, NearestEdgePoint - Angle1) <= RoundingAdjustment && (Vector3.Dot(NearestDeltaPosPoint - EstimatedPos, NearestDeltaPosPoint - Position) <= RoundingAdjustment || (EstimatedPos - NearestEdgePoint).magnitude < Radius))) { return false; }
        
        float RadiusAdaptationGap = Mathf.Sqrt(Mathf.Pow(Radius, 2) - Mathf.Pow(EdgeToDeltaPos.magnitude, 2));
        Vector3 TempImpactPos = NearestEdgePoint + EdgeToDeltaPos - DeltaPos.normalized * RadiusAdaptationGap;
        float TempDistBeforeCollision = (TempImpactPos - Position).magnitude;
        
        if (TempDistBeforeCollision >= DistBeforeCollision) { return false; }
        DistBeforeCollision = TempDistBeforeCollision;
        Collision = true;
        WallCollided = true;
        ImpactPos = TempImpactPos;
        ImpactNormal = ImpactPos - NearestEdgePoint;
        RecoveryFactor = Wall.GetRecoveryFactor();
        return true;
    }

    bool AngleCollisionDetection(Vector3 Angle, Wall Wall, ref bool WallCollided)
    {
        Vector3 NearestPoint = NearestPointFromLine(Angle, EstimatedPos, Position);
        Vector3 PointToTrajectory = NearestPoint - Angle;
        if (!(PointToTrajectory.magnitude < Radius && (Vector3.Dot(NearestPoint - EstimatedPos, NearestPoint - Position) <= RoundingAdjustment || (EstimatedPos - Angle).magnitude < Radius))) { return false; }

        float RadiusAdaptationGap = Mathf.Sqrt(Mathf.Pow(Radius, 2) - Mathf.Pow(PointToTrajectory.magnitude, 2));
        Vector3 TempImpactPos = Angle + PointToTrajectory - DeltaPos.normalized * RadiusAdaptationGap;
        float TempDistBeforeCollision = (TempImpactPos - Position).magnitude;
        if (TempDistBeforeCollision < DistBeforeCollision) { return false; }

        DistBeforeCollision = TempDistBeforeCollision;
        Collision = true;
        WallCollided = true;
        ImpactPos = TempImpactPos;
        ImpactNormal = ImpactPos - Angle;
        RecoveryFactor = Wall.GetRecoveryFactor();
        return true;
    }
    void BumperCollisionDetection() { for (int i = 0; i < Bumpers.Length; i++) { RadialCollisionDetection(Bumpers[i]); }; }
    bool RadialCollisionDetection(Bumper Bumper)
    {
        Vector3 BumperPos = Bumper.GetPos();
        Vector3 BumperAxis = Bumper.GetAxis();
        Vector3 BumperOtherAxisPointPos = BumperPos + BumperAxis;
        Vector3 BumperToPos = Position - BumperPos;
        Vector3 BumperToEstimatedPos = Position - BumperPos;
        Vector3 BumperRadialAxisToPos = BumperToPos - Vector3.Project(BumperToPos, BumperAxis);
        Vector3 BumperRadialAxisToEstimatedPos = BumperToPos - Vector3.Project(BumperToEstimatedPos, BumperAxis);
        float BumperRadius = Bumper.GetRadius();
        float RadiusSum = Radius + BumperRadius;

        Vector3 NearestAxisPoint;
        Vector3 NearestDeltaPosPoint;
        if (!(NearestPointsTwoLines(BumperPos, BumperOtherAxisPointPos, Position, EstimatedPos, out NearestAxisPoint, out NearestDeltaPosPoint))) { return false; }

        Vector3 AxisToDeltaPos = NearestDeltaPosPoint - NearestAxisPoint;
        if (!(AxisToDeltaPos.magnitude < RadiusSum && (Vector3.Dot(NearestDeltaPosPoint - EstimatedPos, NearestDeltaPosPoint - Position) <= RoundingAdjustment || Vector3.Project(EstimatedPos - NearestAxisPoint, BumperRadialAxisToEstimatedPos).magnitude < RadiusSum))) { return false; }
        
        float RadiusAdaptationGapOnBumperPlane = Mathf.Sqrt(Mathf.Pow(RadiusSum, 2) - Mathf.Pow(AxisToDeltaPos.magnitude, 2));
        float RadiusAdaptationGap = DeltaPos.magnitude * RadiusAdaptationGapOnBumperPlane / Vector3.ProjectOnPlane(DeltaPos, BumperAxis).magnitude;
        Vector3 TempImpactPos = NearestDeltaPosPoint - DeltaPos.normalized * RadiusAdaptationGap;
        float DistCollisionBumper = CalcDistPointLine(TempImpactPos, BumperPos, BumperOtherAxisPointPos);

        if (DistCollisionBumper < Radius)
        {
            TempImpactPos += ImpactNormal.normalized * (Radius - DistCollisionBumper);
            for (int i = 0; i < MaxSimulationSteps; i++)
            {
                if (CalcDistPointLine(TempImpactPos, BumperPos, BumperOtherAxisPointPos) < Radius) { break; };
                TempImpactPos += ImpactNormal.normalized * RoundingAdjustment;
            }
        }

        float TempDistBeforeCollision = (TempImpactPos - Position).magnitude;
        if (TempDistBeforeCollision < DistBeforeCollision) { return false; }

        DistBeforeCollision = TempDistBeforeCollision;
        Collision = true;
        GameManager.UpdateScore(Bumper.GetScore());
        ImpactPos = TempImpactPos;
        ImpactNormal = Vector3.Project(ImpactPos - NearestAxisPoint, BumperRadialAxisToPos);
        RecoveryFactor = Bumper.GetRecoveryFactor();
        RepulsionForce = ImpactNormal * Bumper.GetRepulsionForceMagnitude();
        return true;
    }
    void ResolveCollision()
    {
        if (Collision)
        {
            Vector3 NormalizedDeltaPos = DeltaPos.normalized;
            Vector3 DeltaPosPerpendicular = Vector3.Project(NormalizedDeltaPos, ImpactNormal);
            Vector3 DeltaPosParallel = (NormalizedDeltaPos - DeltaPosPerpendicular);

            Vector3 VelocityDir = DeltaPosParallel - DeltaPosPerpendicular * RecoveryFactor;

            float TravelCompletionFactor = (EstimatedPos - ImpactPos).magnitude / TotalDistToTravel;

            Velocity = VelocityDir * Velocity.magnitude + (RepulsionForce + FlipForce) * DT / Mass;
            RepulsionForce = Vector3.zero;
            Vector3 CollisionedFlipPerpendicularVelocity = Vector3.Project(Velocity, FlipForce);
            FlipForce = Vector3.zero;

            Debug.DrawLine(Position, ImpactPos, Color.red, 5);
            Debug.DrawLine(ImpactPos, ImpactPos - ImpactNormal.normalized * Radius, Color.cyan, 5);

            Position = ImpactPos;
            EstimatedPos = ImpactPos + Velocity * TravelCompletionFactor * DT;
            DeltaPos = EstimatedPos - Position;
            DistToTravel = DeltaPos.magnitude;
        }
        else
        {
            Debug.DrawLine(Position, EstimatedPos, Color.red, 5);
            DistToTravel = 0;
            Position = EstimatedPos;
        }
    }
    void BallUpdate() { gameObject.transform.position = Position; }
    bool NearestPointsTwoLines(Vector3 A, Vector3 B, Vector3 C, Vector3 D, out Vector3 P, out Vector3 Q)
    {
        Vector3 U = A - C;
        Vector3 V = B - A;
        Vector3 W = D - C;
        float VW = Vector3.Dot(V, W);
        float VV = Vector3.Dot(V, V);
        float WW = Vector3.Dot(W, W);
        float UW = Vector3.Dot(U, W);
        float UV = Vector3.Dot(U, V);
        float X = UW * VW - UV * WW;
        float Y = UW * VV - UV * VW;
        float Z = VV * WW - VW * VW;

        if (Z != 0)
        {
            float ZInverse = 1 / Z;
            P = A + X * ZInverse * V;
            Q = C + Y * ZInverse * W;
            Vector3 PQ = Q - P;

            return true;
        }
        else
        {
            P = Vector3.zero;
            Q = Vector3.zero;

            return false;
        }
    }
    Vector3 NearestPointFromLine(Vector3 A, Vector3 B, Vector3 C) { return B + Vector3.Project(A - B, C - B); } // between A and BC 
    float CalcDistPointLine(Vector3 A, Vector3 B, Vector3 C) { return (A - NearestPointFromLine(A, B, C)).magnitude; }
    float CalcDistPointPlane(Vector3 PointPos, Vector3 PlaneCenterPos, Vector3 PlaneNormal) { return (Vector3.ProjectOnPlane(PointPos - PlaneCenterPos, PlaneNormal) + PlaneCenterPos - PointPos).magnitude; }
    void SegmentPlaneIntersection(Vector3 APos, Vector3 BPos, Vector3 PlaneCenterPos, Vector3 PlaneNormal, out Vector3 IntersectionPos)
    {
        float AToPlane = CalcDistPointPlane(APos, PlaneCenterPos, PlaneNormal);
        float BToPlane = CalcDistPointPlane(BPos, PlaneCenterPos, PlaneNormal);
        if (AToPlane == 0) { IntersectionPos = APos; }
        else if (BToPlane == 0) { IntersectionPos = BPos; }
        else { IntersectionPos = APos + AToPlane * (BPos - APos) / (AToPlane + BToPlane); }
    }
    public void Launch() { Velocity = LaunchVelocity; }
}

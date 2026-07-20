using Dexter.Spider;
using UnityEngine;

namespace Dexter.Butterfly
{
    /// <summary>
    /// Realistic-ish butterfly locomotion: erratic 3D flight, occasional ground rests with slowed wings.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class ButterflyBehavior : MonoBehaviour
    {
        private enum State
        {
            Flying,
            Landing,
            Resting,
            TakingOff
        }

        [Header("Flight")]
        [SerializeField, Min(0.2f)] private float flySpeed = 1.8f;
        [SerializeField, Min(0.1f)] private float turnResponsiveness = 1.4f;
        [SerializeField, Min(0.2f)] private float cruiseHeightMin = 0.6f;
        [SerializeField, Min(0.2f)] private float cruiseHeightMax = 2.2f;
        [SerializeField, Range(0.05f, 0.5f)] private float verticalWanderStrength = 0.18f;
        [SerializeField, Min(1f)] private float minFlyTimeBeforeLanding = 4f;
        [SerializeField, Range(0f, 1f)] private float landingChancePerSecond = 0.08f;

        [Header("Ground")]
        [SerializeField, Min(0.01f)] private float groundClearance = 0.04f;
        [SerializeField] private Vector2 restDurationRange = new(2.5f, 6f);
        [SerializeField, Min(0.2f)] private float landingDescentSpeed = 1.1f;
        [SerializeField, Min(0.2f)] private float takeoffClimbSpeed = 1.4f;

        [Header("Wings")]
        [SerializeField, Range(0f, 1f)] private float restingFlapSpeedMultiplier = 0.05f;
        [SerializeField, Range(0f, 1f)] private float landingFlapSpeedMultiplier = 0.65f;
        [SerializeField, Range(0f, 1f)] private float takeoffFlapSpeedMultiplier = 0.85f;

        private State state = State.Flying;
        private Vector3 velocity = Vector3.forward;
        private Vector3 landingPoint;
        private float cruiseHeight;
        private float flyTimer;
        private float restTimer;
        private float noiseSeed;
        private Terrain terrain;
        private ButterflyWingFlapPreview wingFlap;
        private CirclePrey prey;

        private void Awake()
        {
            wingFlap = GetComponent<ButterflyWingFlapPreview>();
            prey = GetComponent<CirclePrey>();
            terrain = Terrain.activeTerrain;
            noiseSeed = Random.Range(0f, 1000f);
            cruiseHeight = Random.Range(cruiseHeightMin, cruiseHeightMax);
            velocity = transform.forward.sqrMagnitude > 0.01f
                ? transform.forward * flySpeed
                : Random.onUnitSphere * flySpeed;
            velocity.y = 0f;
        }

        private void Update()
        {
            if (prey != null && prey.IsCaptured)
                return;

            switch (state)
            {
                case State.Flying:
                    UpdateFlying();
                    break;
                case State.Landing:
                    UpdateLanding();
                    break;
                case State.Resting:
                    UpdateResting();
                    break;
                case State.TakingOff:
                    UpdateTakingOff();
                    break;
            }
        }

        private void UpdateFlying()
        {
            SetWingMultipliers(1f, 1f);
            flyTimer += Time.deltaTime;

            Vector3 wanderDirection = SampleWanderDirection(includeVertical: true);
            velocity = Vector3.Slerp(
                velocity,
                wanderDirection * flySpeed,
                turnResponsiveness * Time.deltaTime);

            MoveAndOrient(velocity, alignToVelocity: true);
            MaintainCruiseAltitude();

            if (flyTimer >= minFlyTimeBeforeLanding &&
                Random.value < landingChancePerSecond * Time.deltaTime)
            {
                BeginLanding();
            }
        }

        private void UpdateLanding()
        {
            SetWingMultipliers(landingFlapSpeedMultiplier, 0.75f);

            Vector3 toLanding = landingPoint - transform.position;
            toLanding.y -= landingDescentSpeed * Time.deltaTime;
            velocity = Vector3.Lerp(velocity, toLanding.normalized * landingDescentSpeed, 2f * Time.deltaTime);

            MoveAndOrient(velocity, alignToVelocity: true);

            float groundY = SampleTerrainHeight(transform.position);
            if (transform.position.y <= groundY + groundClearance + 0.02f)
            {
                Vector3 grounded = transform.position;
                grounded.y = groundY + groundClearance;
                transform.position = grounded;
                state = State.Resting;
                restTimer = Random.Range(restDurationRange.x, restDurationRange.y);
                velocity = Vector3.zero;
            }
        }

        private void UpdateResting()
        {
            SetWingMultipliers(restingFlapSpeedMultiplier, 0.2f);
            restTimer -= Time.deltaTime;

            float groundY = SampleTerrainHeight(transform.position);
            Vector3 grounded = transform.position;
            grounded.y = groundY + groundClearance;
            transform.position = grounded;

            if (restTimer <= 0f)
                state = State.TakingOff;
        }

        private void UpdateTakingOff()
        {
            SetWingMultipliers(takeoffFlapSpeedMultiplier, 0.9f);

            Vector3 climb = velocity;
            climb.y = takeoffClimbSpeed;
            if (climb.sqrMagnitude < 0.01f)
                climb = transform.forward * 0.4f + Vector3.up * takeoffClimbSpeed;

            velocity = Vector3.Lerp(velocity, climb, 2.5f * Time.deltaTime);
            MoveAndOrient(velocity, alignToVelocity: true);

            float groundY = SampleTerrainHeight(transform.position);
            if (transform.position.y >= groundY + cruiseHeight - 0.1f)
            {
                state = State.Flying;
                flyTimer = 0f;
                cruiseHeight = Random.Range(cruiseHeightMin, cruiseHeightMax);
                SetWingMultipliers(1f, 1f);
            }
        }

        private void BeginLanding()
        {
            landingPoint = PickLandingPoint();
            state = State.Landing;
        }

        private Vector3 PickLandingPoint()
        {
            Vector2 offset = Random.insideUnitCircle * 2.5f;
            Vector3 candidate = transform.position + new Vector3(offset.x, 0f, offset.y);
            candidate.y = SampleTerrainHeight(candidate) + groundClearance;
            return candidate;
        }

        private Vector3 SampleWanderDirection(bool includeVertical)
        {
            float time = Time.time;
            float nx = Mathf.PerlinNoise(noiseSeed, time * 0.22f) * 2f - 1f;
            float nz = Mathf.PerlinNoise(time * 0.22f + 40f, noiseSeed + 40f) * 2f - 1f;
            float ny = includeVertical
                ? (Mathf.PerlinNoise(time * 0.17f + 80f, noiseSeed + 80f) * 2f - 1f) * verticalWanderStrength
                : 0f;

            Vector3 direction = new Vector3(nx, ny, nz);
            if (direction.sqrMagnitude < 0.0001f)
                direction = transform.forward;

            return direction.normalized;
        }

        private void MaintainCruiseAltitude()
        {
            float groundY = SampleTerrainHeight(transform.position);
            float targetY = groundY + cruiseHeight;
            Vector3 position = transform.position;
            position.y = Mathf.Lerp(position.y, targetY, 2f * Time.deltaTime);
            transform.position = position;
        }

        private void MoveAndOrient(Vector3 desiredVelocity, bool alignToVelocity)
        {
            Vector3 delta = desiredVelocity * Time.deltaTime;
            transform.position += delta;

            if (!alignToVelocity || desiredVelocity.sqrMagnitude < 0.0001f)
                return;

            Vector3 flatForward = desiredVelocity;
            flatForward.y = 0f;
            if (flatForward.sqrMagnitude < 0.0001f)
                return;

            Quaternion targetRotation = Quaternion.LookRotation(flatForward.normalized, Vector3.up);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 4f * Time.deltaTime);
        }

        private float SampleTerrainHeight(Vector3 worldPosition)
        {
            if (terrain == null)
                return worldPosition.y;

            return terrain.SampleHeight(worldPosition) + terrain.transform.position.y;
        }

        private void SetWingMultipliers(float speedMultiplier, float angleMultiplier)
        {
            if (wingFlap == null)
                return;

            wingFlap.FlapSpeedMultiplier = speedMultiplier;
            wingFlap.FlapAngleMultiplier = angleMultiplier;
        }
    }
}

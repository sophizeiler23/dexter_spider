using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Keeps a camera smoothly positioned behind and above a moving spider root.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class ThirdPersonSpiderCamera : MonoBehaviour
    {
        [SerializeField] private Transform target;

        [Header("Position")]
        [SerializeField, Min(0f)] private float distanceBehind = 3f;
        [SerializeField] private float height = 2f;
        [SerializeField, Min(0.01f)] private float positionSmoothTime = 0.15f;

        [Header("View")]
        [SerializeField] private float lookHeight = 0.5f;
        [SerializeField, Min(0f)] private float rotationSpeed = 10f;

        private Vector3 positionVelocity;

        private void OnEnable()
        {
            positionVelocity = Vector3.zero;
        }

        private void LateUpdate()
        {
            if (target == null)
                return;

            Vector3 desiredPosition = target.position
                - target.forward * distanceBehind
                + Vector3.up * height;

            transform.position = Vector3.SmoothDamp(
                transform.position,
                desiredPosition,
                ref positionVelocity,
                positionSmoothTime);

            Vector3 lookTarget = target.position + Vector3.up * lookHeight;
            Vector3 lookDirection = lookTarget - transform.position;
            if (lookDirection.sqrMagnitude < 0.0001f)
                return;

            Quaternion desiredRotation = Quaternion.LookRotation(lookDirection, Vector3.up);
            transform.rotation = Quaternion.Slerp(
                transform.rotation,
                desiredRotation,
                rotationSpeed * Time.deltaTime);
        }
    }
}

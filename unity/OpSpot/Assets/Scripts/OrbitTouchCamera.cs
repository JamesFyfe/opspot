using UnityEngine;

public class OrbitTouchCamera : MonoBehaviour
{
    public Transform target;
    public float distance = 70000f;
    public float minDistance = 25010f;
    public float maxDistance = 100000;
    [Header("Planet")]
    [Tooltip("Planet radius used to compute altitude-based scaling.")]
    public float planetRadius = 25000f;

    // Rotation sensitivity: degrees rotated per pixel dragged (before zoom scaling)
    public float rotationDegreesPerPixel = 0.2f;
    // Rotation sensitivity scales with zoom: near -> small, far -> large
    public float rotationNearScale = 0.01f;   // scale at minDistance
    public float rotationFarScale = 1.0f;     // scale at maxDistance

    // Zoom speeds scale with current distance, so far = faster, near = slower
    public float pinchZoomSpeed = 2f;       // multiplier for pinch (per screen fraction per second)
    public float mouseZoomSpeed = 2f;       // multiplier for scroll (per tick per second)

    public float minPitch = -90f;
    public float maxPitch = 90f;

    // Inertia configuration
    public float angularDamping = 5f;    // higher = stops faster
    public float zoomDamping = 10f;       // higher = stops faster
    public float maxAngularSpeed = 360f; // deg/sec
    public float maxZoomSpeed = 60000f;     // units/sec

    private float yaw;
    private float pitch;
    private float desiredDistance;
    private float yawVelocity;   // deg/sec
    private float pitchVelocity; // deg/sec
    private float zoomVelocity;  // units/sec

    private Vector2 _lastMousePos;
    private bool _mouseDragging;

    void Start()
    {
        if (target == null)
        {
            var go = new GameObject("OrbitTarget");
            go.transform.position = Vector3.zero;
            target = go.transform;
        }

        var e = transform.eulerAngles;
        yaw = e.y;
        pitch = e.x;
        desiredDistance = Mathf.Clamp(distance, minDistance, maxDistance);
    }

    void LateUpdate()
    {
        HandleInput();

        // Exponential damping (frame-rate independent)
        float angDamp = Mathf.Exp(-angularDamping * Time.deltaTime);
        float zDamp = Mathf.Exp(-zoomDamping * Time.deltaTime);
        yawVelocity *= angDamp;
        pitchVelocity *= angDamp;
        zoomVelocity *= zDamp;

        // Integrate velocities
        yaw += yawVelocity * Time.deltaTime;
        pitch += pitchVelocity * Time.deltaTime;
        pitch = Mathf.Clamp(pitch, minPitch, maxPitch);

        desiredDistance += zoomVelocity * Time.deltaTime;
        desiredDistance = Mathf.Clamp(desiredDistance, minDistance, maxDistance);

        // Final camera transform (locked to target without positional lag)
        Quaternion rotation = Quaternion.Euler(pitch, yaw, 0f);
        Vector3 desiredPos = target.position + rotation * new Vector3(0f, 0f, -desiredDistance);
        transform.SetPositionAndRotation(desiredPos, rotation);
    }

    void HandleInput()
    {
#if UNITY_EDITOR || UNITY_STANDALONE
        if (Input.GetMouseButtonDown(0))
        {
            _mouseDragging = true;
            _lastMousePos = Input.mousePosition;
        }
        if (Input.GetMouseButtonUp(0))
        {
            _mouseDragging = false;
        }
        if (_mouseDragging && Input.GetMouseButton(0))
        {
            Vector2 mp = Input.mousePosition;
            Vector2 d = mp - _lastMousePos; // pixels
            _lastMousePos = mp;

            float minAlt = Mathf.Max(0f, minDistance - planetRadius);
            float maxAlt = Mathf.Max(minAlt + 1f, maxDistance - planetRadius);
            float alt = Mathf.Max(0.001f, desiredDistance - planetRadius);
            float t = Mathf.InverseLerp(minAlt, maxAlt, alt);
            float rotScale = Mathf.Lerp(rotationNearScale, rotationFarScale, t);

            // Convert pixel drag directly to angles (constant speed regardless of screen size)
            float dyaw = d.x * rotationDegreesPerPixel * rotScale;
            float dpitch = -d.y * rotationDegreesPerPixel * rotScale;

            float invDt = 1f / Mathf.Max(Time.deltaTime, 0.0001f);
            yawVelocity = Mathf.Clamp(dyaw * invDt, -maxAngularSpeed, maxAngularSpeed);
            pitchVelocity = Mathf.Clamp(dpitch * invDt, -maxAngularSpeed, maxAngularSpeed);
        }

        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (Mathf.Abs(scroll) > 0.0001f)
        {
            // Zoom velocity scales with current altitude (distance - radius)
            float invDt = 1f / Mathf.Max(Time.deltaTime, 0.0001f);
            float alt = Mathf.Max(0.001f, desiredDistance - planetRadius);
            float targetZoomVel = -scroll * mouseZoomSpeed * alt * invDt;
            zoomVelocity = Mathf.Clamp(targetZoomVel, -maxZoomSpeed, maxZoomSpeed);
        }
#endif

        if (Input.touchCount == 1)
        {
            Touch t = Input.GetTouch(0);
            if (t.phase == TouchPhase.Moved)
            {
                float invDt = 1f / Mathf.Max(Time.deltaTime, 0.0001f);
                float minAlt = Mathf.Max(0f, minDistance - planetRadius);
                float maxAlt = Mathf.Max(minAlt + 1f, maxDistance - planetRadius);
                float alt = Mathf.Max(0.001f, desiredDistance - planetRadius);
                float tZoom = Mathf.InverseLerp(minAlt, maxAlt, alt);
                float rotScale = Mathf.Lerp(rotationNearScale, rotationFarScale, tZoom);
                // Convert pixel drag directly to angles (constant speed regardless of screen size)
                float dyaw = t.deltaPosition.x * rotationDegreesPerPixel * rotScale;
                float dpitch = -t.deltaPosition.y * rotationDegreesPerPixel * rotScale;
                float targetYawVel = dyaw * invDt;
                float targetPitchVel = dpitch * invDt;
                yawVelocity = Mathf.Clamp(targetYawVel, -maxAngularSpeed, maxAngularSpeed);
                pitchVelocity = Mathf.Clamp(targetPitchVel, -maxAngularSpeed, maxAngularSpeed);
            }
        }
        else if (Input.touchCount == 2)
        {
            Touch t0 = Input.GetTouch(0);
            Touch t1 = Input.GetTouch(1);

            Vector2 prev0 = t0.position - t0.deltaPosition;
            Vector2 prev1 = t1.position - t1.deltaPosition;

            float prevMag = (prev0 - prev1).magnitude;
            float currMag = (t0.position - t1.position).magnitude;
            float deltaMag = currMag - prevMag; // pixels

            float invDt = 1f / Mathf.Max(Time.deltaTime, 0.0001f);
            // Normalize pinch by screen size, scale by current altitude
            float pinchNorm = deltaMag / Mathf.Max(1f, Mathf.Min(Screen.width, Screen.height));
            float alt = Mathf.Max(0.001f, desiredDistance - planetRadius);
            float targetZoomVel = -pinchNorm * pinchZoomSpeed * alt * invDt;
            zoomVelocity = Mathf.Clamp(targetZoomVel, -maxZoomSpeed, maxZoomSpeed);
        }
    }
}
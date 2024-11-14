using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.ProBuilder;
using Random = UnityEngine.Random;

public class ParticleController : MonoBehaviour
{
    [Header("Particle Filter Configuration")]
    [Tooltip("Number of particles in the filter. Higher values improve accuracy but reduce performance.")]
    [Range(10, 1000)]
    public int nParticles = 100;

    [Tooltip("Variance for the particle sensors' measurements.")]
    [Range(0f, 50f)]
    public float sensorVariance = 10f;

    [Tooltip("Prefab representing each particle in the simulation.")]
    public GameObject particlePrefab;

    [Tooltip("Reference to the robot object in the simulation.")]
    public GameObject robot;

    [Tooltip("Reference to the map area for boundary constraints.")]
    public GameObject mapArea;

    [Tooltip("Line renderer for visualization (optional).")]
    public LineRenderer lineRend;

    [Header("AMCL Parameters")]
    [Tooltip("Effective Sample Size (ESS) threshold for resampling.")]
    [Range(0f, 1f)]
    public float resamplingThreshold = 0.5f;

    [Tooltip("Enable adaptive resampling based on particle spread.")]
    public bool useAdaptiveResampling = true;

    [Tooltip("Minimum distance for a particle to be moved.")]
    [Range(0f, 1f)]
    public float minParticleDistance = 0.1f;

    [Header("Adaptive Resampling Settings")]
    [Tooltip("Minimum number of particles during resampling.")]
    [Range(1, 100)]
    public int minParticles = 50;

    [Tooltip("Maximum number of particles allowed in the filter.")]
    [Range(100, 1000)]
    public int maxParticles = 500;

    [Tooltip("Ratio of particles that are randomly placed to prevent false convergence.")]
    [Range(0f, 1f)]
    public float randomParticleRatio = 0.1f; // Percentage of particles to randomize

    [Header("Noise Parameters")]
    [Tooltip("Standard deviation for positional noise added to particles.")]
    [Range(0f, 1f)]
    public float PositionNoiseStdDev = 0.1f;

    [Tooltip("Standard deviation for orientation noise added to particles.")]
    [Range(0f, 10f)]
    public float OrientationNoiseStdDev = 1.0f;

    [Tooltip("Time interval between particle updates.")]
    [Range(0f, 1f)]
    public float UpdateInterval = 0.1f;

    [Header("Sensor Settings")]
    [Tooltip("Maximum distance for raycasting sensor readings.")]
    [Range(0f, 100f)]
    public float SensorMaxDistance = 50f;

    [Tooltip("Name of the child object containing the sensors on each particle prefab.")]
    public string sensorsParentName = "DistanceSensors";

    // Private fields (no need to show in Inspector)
    private Transform[] robotSensorsOrigins;
    private List<Transform[]> sensorsOrigins;
    private List<Particle> particles;
    private List<GameObject> particleObjects;
    private float minX, maxX, minZ, maxZ;
    private Vector3 lastPosition;
    private float lastAngle;
    private LayerMask mapLayerMask;
    private LayerMask particleLayerMask;
    private int nSensors = -1;

    // Constants (fixed values not shown in Inspector)
    private const int MapLayer = 6;
    private const int ParticleLayer = 7;
    private const float shiftUpParticles = 0.25f;

    // Debug fields (hidden unless debugging is enabled)
    [Header("Debug Settings (Optional)")]
    [Tooltip("Show debug particles if enabled.")]
    public bool enableDebugParticles = false;

    private List<Transform> sensorShow = null;
    private List<LineRenderer> sensorLineRenderShow = null;
    private GameObject debugParticle = null;

    private void Start()
    {
        mapLayerMask = 1 << MapLayer;
        particleLayerMask = 1 << ParticleLayer;

        nParticles = Mathf.Clamp(nParticles, minParticles, maxParticles);

        nSensors = particlePrefab.transform.Find(sensorsParentName).childCount;
        debugParticle = Instantiate(particlePrefab);
        debugParticle.SetActive(false);

        InitializeMapBounds();
        InitializeParticleSystem();
        InitializeRobotSensors();
        InvokeRepeating(nameof(AMCLUpdate), 0.0f, UpdateInterval);
    }

    private void Update()
    {
        if (enableDebugParticles && Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity, particleLayerMask))
            {
                Transform sensorsParent = hit.collider.transform.parent.Find(sensorsParentName);
                sensorShow = new List<Transform>(nSensors);
                for (int i = 0; i < nSensors; i++)
                {
                    sensorShow.Add(sensorsParent.GetChild(i).transform);
                }
                if (sensorLineRenderShow == null)
                {
                    sensorLineRenderShow = new List<LineRenderer>(nSensors);
                    for (int i = 0; i < nSensors; i++)
                    {
                        sensorLineRenderShow.Add(Instantiate(lineRend));
                    }
                }
            }
            else if (sensorShow != null)
            {
                for (int i = 0; i < sensorShow.Count; i++)
                {
                    sensorLineRenderShow[i].enabled = false;
                }
                debugParticle.SetActive(false);
                sensorShow = null;
            }
        }
        if (enableDebugParticles && Input.GetMouseButton(1))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity, mapLayerMask) && hit.collider.CompareTag(mapArea.tag))
            {
                debugParticle.transform.position = hit.point + Vector3.up * shiftUpParticles;

                if (!debugParticle.activeSelf)
                {
                    Transform sensorsParent = debugParticle.transform.Find(sensorsParentName);

                    sensorShow = new List<Transform>(nSensors);
                    for (int i = 0; i < nSensors; i++)
                    {
                        sensorShow.Add(sensorsParent.GetChild(i).transform);
                    }
                    if (sensorLineRenderShow == null)
                    {
                        sensorLineRenderShow = new List<LineRenderer>(nSensors);
                        for (int i = 0; i < nSensors; i++)
                        {
                            sensorLineRenderShow.Add(Instantiate(lineRend));
                        }
                    }
                    debugParticle.SetActive(true);
                }
                for (int i = 0; i < sensorShow.Count; i++)
                {
                    if (Physics.Raycast(sensorShow[i].position, sensorShow[i].up, out RaycastHit hitSensor, SensorMaxDistance, mapLayerMask))
                    {
                        sensorLineRenderShow[i].enabled = true;
                        sensorLineRenderShow[i].SetPosition(0, sensorShow[i].position);
                        sensorLineRenderShow[i].SetPosition(1, hitSensor.point);
                    }
                    else
                    {
                        sensorLineRenderShow[i].enabled = false;
                    }
                }
                float[] robotDistances = GetRaycastDistances(robotSensorsOrigins);
                float[] particleDist = GetRaycastDistances(sensorShow.ToArray());

                float weight = 1.0f;

                for (int j = 0; j < robotDistances.Length; j++)
                {
                    float distanceError = robotDistances[j] - particleDist[j];
                    float a = -(distanceError * distanceError) / (2 * sensorVariance * sensorVariance);
                    weight *= Mathf.Exp(a); // Gaussian likelihood
                }

            }
            else if (sensorShow != null)
            {
                for (int i = 0; i < sensorShow.Count; i++)
                {
                    sensorLineRenderShow[i].enabled = false;
                }
                debugParticle.SetActive(false);
                sensorShow = null;
            }
        }
        if (enableDebugParticles && Mathf.Abs(Input.mouseScrollDelta.y) > 0)
        {
            debugParticle.transform.Rotate(Vector3.up * Input.mouseScrollDelta.y);
        }
    }

    // Initializes the map boundaries based on ProBuilder mesh
    private void InitializeMapBounds()
    {
        Vertex[] vertices = mapArea.GetComponent<ProBuilderMesh>().GetVertices();
        minX = minZ = float.MaxValue;
        maxX = maxZ = float.MinValue;

        foreach (var vertice in vertices)
        {
            Vector3 pos = vertice.position;
            minX = Mathf.Min(minX, pos.x);
            maxX = Mathf.Max(maxX, pos.x);
            minZ = Mathf.Min(minZ, pos.z);
            maxZ = Mathf.Max(maxZ, pos.z);
        }
    }

    // Sets up particle objects and sensors for the robot
    private void InitializeParticleSystem()
    {
        particleObjects = new List<GameObject>(nParticles);
        sensorsOrigins = new List<Transform[]>(nParticles);
        particles = new List<Particle>(nParticles);
        lastPosition = robot.transform.position;
        lastAngle = robot.transform.rotation.eulerAngles.y;

        for (int i = 0; i < maxParticles; i++)
        {
            GameObject particleObj = Instantiate(particlePrefab, Vector3.zero, Quaternion.identity, transform);
            particleObj.SetActive(false); // Initially hidden
            particleObjects.Add(particleObj);

            Transform sensorsParent = particleObj.transform.Find(sensorsParentName);

            Transform[] particleSensors = new Transform[nSensors];

            for (int j = 0; j < nSensors; j++)
            {
                particleSensors[j] = sensorsParent.GetChild(j);
            }
            sensorsOrigins.Add(particleSensors);
        }
        GenerateParticles();
    }

    private void InitializeRobotSensors()
    {
        Transform sensorsParent = robot.transform.Find(sensorsParentName);
        robotSensorsOrigins = new Transform[nSensors];
        for (int i = 0; i < nSensors; i++)
        {
            robotSensorsOrigins[i] = sensorsParent.GetChild(i);
        }
    }


    private void AMCLUpdate()
    {
        // Step 1: Update particles based on robot's motion (odometry data)
        Vector3 deltaPosition = robot.transform.position - lastPosition;
        deltaPosition.y = 0;
        float deltaTheta = robot.transform.rotation.eulerAngles.y - lastAngle;
        Vector3 relativeDeltaPosition = robot.transform.InverseTransformDirection(deltaPosition);
        if (deltaPosition.magnitude < 0.2)
        {
            return;
        }

        UpdateParticlePositions(relativeDeltaPosition, deltaTheta); // Update particle positions

        // Step 2: Get robot sensor readings
        float[] robotDistances = GetRaycastDistances(robotSensorsOrigins);

        // Step 3: Compute particle weights based on sensor data likelihood
        CalculateWeight(robotDistances);

        // Step 4: Resample particles based on weights
        ResampleParticles();

        // Step 5: Ensure particles are on the ground
        EnsureParticlesOnGround();

        // Update robot's last known position and angle for next iteration
        lastPosition = robot.transform.position;
        lastAngle = robot.transform.rotation.eulerAngles.y;

        if (enableDebugParticles && sensorShow != null)
        {
            for (int i = 0; i < sensorShow.Count; i++)
            {
                if (Physics.Raycast(sensorShow[i].position, sensorShow[i].up, out RaycastHit hit, SensorMaxDistance, mapLayerMask))
                {
                    sensorLineRenderShow[i].enabled = true;
                    sensorLineRenderShow[i].SetPosition(0, sensorShow[i].position);
                    sensorLineRenderShow[i].SetPosition(1, hit.point);
                }
                else
                {
                    sensorLineRenderShow[i].enabled = false;
                }
            }
        }
    }

    // Ensure particles are on the ground, repositioning if necessary
    private void EnsureParticlesOnGround()
    {
        // Loop through particles and reposition them if not on the ground
        for (int i = 0; i < particles.Count; i++)
        {
            if (!IsOnGround(particles[i].position))
            {
                // Reinitialize the particle randomly within the map
                ReinitializeParticle(i);
            }
        }
    }

    // Function to reinitialize a particle at a valid position on the ground
    private void ReinitializeParticle(int index)
    {
        Particle particle = particles[index];
        float randomX, randomZ;
        RaycastHit hit;

        // Generate random positions until a valid position is found
        while (true)
        {
            randomX = Random.Range(minX, maxX); // Define minX and maxX based on your map
            randomZ = Random.Range(minZ, maxZ); // Define minZ and maxZ based on your map
            Vector3 rayOrigin = new Vector3(randomX, Camera.main.transform.position.y, randomZ);
            Ray ray = new Ray(rayOrigin, Vector3.down);

            // Perform a raycast to check for ground
            if (Physics.Raycast(ray, out hit, Mathf.Infinity, mapLayerMask) && hit.collider.CompareTag(mapArea.tag))
            {
                // If we hit the ground, set the particle's new position and orientation
                particle.position = new Vector3(hit.point.x, 0, hit.point.z);
                particle.orientation = Random.Range(0f, 360f); // Random orientation as well
                particle.weight = 1.0f / nParticles; // Update weight if needed

                particleObjects[index].transform.position = particle.position + Vector3.up * shiftUpParticles;
                particleObjects[index].transform.rotation = Quaternion.Euler(0, particle.orientation, 0);

                break; // Exit the loop when a valid position is found
            }
        }
    }

    // Function to check if the particle is on the ground
    private bool IsOnGround(Vector3 position)
    {
        Vector3 rayOrigin = new Vector3(position.x, Camera.main.transform.position.y, position.z);
        Ray ray = new Ray(rayOrigin, Vector3.down);
        RaycastHit hit;

        if (Physics.Raycast(ray, out hit, Mathf.Infinity, mapLayerMask) && hit.collider.CompareTag(mapArea.tag))
        {
            return true;
        }
        return false;
    }


    // Improved weight calculation using log-likelihood for numerical stability
    private void CalculateWeight(float[] robotDistances)
    {
        float totalLogWeight = float.NegativeInfinity;

        for (int i = 0; i < particles.Count; i++)
        {
            float[] particleDist = GetRaycastDistances(sensorsOrigins[i]);
            float logWeight = 0;

            for (int j = 0; j < robotDistances.Length; j++)
            {
                float distanceError = robotDistances[j] - particleDist[j];
                logWeight += -(distanceError * distanceError) / (2 * sensorVariance * sensorVariance);
            }

            particles[i].weight = logWeight;
            totalLogWeight = LogSum(totalLogWeight, logWeight);
        }

        // Convert log weights back to probabilities and normalize
        float totalWeight = 0;
        for (int i = 0; i < particles.Count; i++)
        {
            particles[i].weight = Mathf.Exp(particles[i].weight - totalLogWeight);
            totalWeight += particles[i].weight;
        }

        // Normalize weights
        for (int i = 0; i < particles.Count; i++)
        {
            particles[i].weight /= totalWeight;
        }
    }

    private void ResampleParticles()
    {
        float ess = CalculateEffectiveSampleSize();

        // Determine the target particle count based on adaptive resampling
        int targetParticleCount;
        if (useAdaptiveResampling)
        {
            targetParticleCount = Mathf.RoundToInt(Mathf.Lerp(maxParticles, minParticles, ess / (resamplingThreshold * nParticles)));
        }
        else
        {
            targetParticleCount = nParticles;  // Keep particle count constant if adaptive resampling is off
        }
        targetParticleCount = Mathf.Clamp(targetParticleCount, minParticles, maxParticles);

        if (ess < resamplingThreshold * nParticles)
        {
            List<Particle> newParticles = new List<Particle>(targetParticleCount);
            float M_inv = 1.0f / targetParticleCount;
            float r = Random.Range(0, M_inv);
            float c = particles[0].weight;
            int i = 0;

            // Calculate the number of particles to be randomly positioned
            int randomCount = Mathf.RoundToInt(randomParticleRatio * targetParticleCount);

            for (int m = 0; m < targetParticleCount; m++)
            {
                Particle newParticle;

                // Place a portion of particles at random locations
                if (m < randomCount)
                {
                    newParticle = CreateRandomParticle(); // Generate a new particle at a random position
                }
                else
                {
                    float U = r + m * M_inv;
                    while (U > c && i < particles.Count - 1)
                    {
                        i++;
                        c += particles[i].weight;
                    }
                    newParticle = CloneParticle(particles[i]); // Clone based on existing particles
                }

                newParticles.Add(newParticle);

                // Update visualization if needed
                particleObjects[m].transform.position = newParticle.position + Vector3.up * shiftUpParticles;
                particleObjects[m].transform.rotation = Quaternion.Euler(0, newParticle.orientation, 0);
                particleObjects[m].SetActive(true);
            }

            // Deactivate extra particle objects
            for (int m = targetParticleCount; m < maxParticles; m++)
            {
                particleObjects[m].SetActive(false);
            }

            // Update particle count and objects
            particles = newParticles;
            nParticles = targetParticleCount;
        }
    }

    // Helper function to create a random particle
    private Particle CreateRandomParticle()
    {
        Particle randomParticle = null;

        while (randomParticle == null)
        {
            float randomX = Random.Range(minX, maxX);
            float randomZ = Random.Range(minZ, maxZ);
            Vector3 rayOrigin = new Vector3(randomX, Camera.main.transform.position.y, randomZ);
            Ray ray = new Ray(rayOrigin, Vector3.down);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, Mathf.Infinity, mapLayerMask) && hit.collider.CompareTag(mapArea.tag))
            {
                randomParticle = new Particle(new Vector3(hit.point.x, 0, hit.point.z), Random.Range(0f, 360f), 1.0f / nParticles);
            }
        }
        return randomParticle;
    }

    // Helper method to clone a particle, instantiate a new particle object, and update sensor origins
    private Particle CloneParticle(Particle particle)
    {
        // Apply small random noise to position and orientation
        Vector3 noisyPosition = new Vector3(particle.position.x, 0, particle.position.z);
        float noisyOrientation = particle.orientation;

        // Clone particle with updated position and orientation
        Particle newParticle = new Particle(noisyPosition, noisyOrientation, particle.weight); // Uniform weight after resampling
        return newParticle;
    }

    // Generate random particles within the map area
    private void GenerateParticles()
    {
        particles.Clear();
        int nParticleCreated = 0;

        while (nParticleCreated < maxParticles)
        {
            float randomX = Random.Range(minX, maxX);
            float randomZ = Random.Range(minZ, maxZ);
            Vector3 rayOrigin = new Vector3(randomX, Camera.main.transform.position.y, randomZ);
            Ray ray = new Ray(rayOrigin, Vector3.down);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, Mathf.Infinity, mapLayerMask) && hit.collider.CompareTag(mapArea.tag))
            {
                Particle particle = new Particle(new Vector3(hit.point.x, 0, hit.point.z), Random.Range(0f, 360f), 1.0f / nParticles);
                particles.Add(particle);

                particleObjects[nParticleCreated].transform.position = particle.position + Vector3.up * shiftUpParticles;
                particleObjects[nParticleCreated].transform.rotation = Quaternion.Euler(0, particle.orientation, 0);
                particleObjects[nParticleCreated].SetActive(true); // Show the particle
                nParticleCreated++;
            }
        }
    }

    // Update particle positions and orientations based on robot's movement
    private void UpdateParticlePositions(Vector3 deltaPosition, float deltaTheta)
    {
        for (int i = 0; i < particles.Count; i++)
        {
            Particle particle = particles[i];
            Vector3 noisyDeltaPosition = deltaPosition + new Vector3(RandomGaussian(0, PositionNoiseStdDev), 0, RandomGaussian(0, PositionNoiseStdDev));
            float noisyDeltaTheta = deltaTheta + RandomGaussian(0, OrientationNoiseStdDev);

            particle.position += particleObjects[i].transform.TransformDirection(noisyDeltaPosition);
            particle.orientation = NormalizeAngle(particle.orientation + noisyDeltaTheta);

            // Update particle object's position in the scene
            particleObjects[i].transform.position = particle.position + Vector3.up * shiftUpParticles;
            particleObjects[i].transform.rotation = Quaternion.Euler(0, particle.orientation, 0);
        }
    }

    private float[] GetRaycastDistances(Transform[] sensorsOrigins)
    {
        float[] distances = new float[sensorsOrigins.Length];
        for (int i = 0; i < sensorsOrigins.Length; i++)
        {
            if (Physics.Raycast(sensorsOrigins[i].position, sensorsOrigins[i].up, out RaycastHit hit, SensorMaxDistance, mapLayerMask))
            {
                distances[i] = hit.distance;
            }
            else
            {
                distances[i] = SensorMaxDistance; // Set max distance if no hit
            }
        }
        return distances;
    }

    // Utility functions
    private float CalculateEffectiveSampleSize()
    {
        float weightSquareSum = particles.Sum(p => p.weight * p.weight);
        return 1.0f / weightSquareSum;
    }

    private float LogSum(float logA, float logB)
    {
        if (logA == float.NegativeInfinity) return logB;
        if (logB == float.NegativeInfinity) return logA;
        if (logA > logB)
            return logA + Mathf.Log(1 + Mathf.Exp(logB - logA));
        return logB + Mathf.Log(1 + Mathf.Exp(logA - logB));
    }

    private float NormalizeAngle(float angle)
    {
        return (angle % 360 + 360) % 360;
    }

    private float RandomGaussian(float mean, float stdDev)
    {
        float u1 = Random.value;
        float u2 = Random.value;
        float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);
        return mean + stdDev * randStdNormal;
    }
}

// Class to represent a particle with position, orientation, and weight
public class Particle
{
    public Vector3 position;
    public float orientation;
    public float weight;

    public Particle(Vector3 pos, float orient, float w)
    {
        position = pos;
        orientation = orient;
        weight = w;
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.ProBuilder;
using XCharts.Runtime;
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

    [Tooltip("Reference to the \"Arrow\" for the probable pos.")]
    public GameObject probablePos;

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
    [Range(1, 200)]
    public int minParticles = 50;

    [Tooltip("Maximum number of particles allowed in the filter.")]
    [Range(200, 1000)]
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

    // public LineChart chart;

    // Private fields (no need to show in Inspector)
    private Transform[] robotSensorsOrigins; // Stores references to the robot's sensors
    private List<Transform[]> sensorsOrigins; // Stores references to each particle's sensors
    private List<Particle> particles; // List of particles used in the filter
    private List<GameObject> particleObjects; // List of GameObjects representing particles
    private float minX, maxX, minZ, maxZ; // Map boundaries for particle placement
    private Vector3 lastPosition; // Last known position of the robot
    private float lastAngle; // Last known orientation of the robot
    private LayerMask mapLayerMask; // Layer mask for the map
    private LayerMask particleLayerMask; // Layer mask for particles
    private int nSensors = -1; // Number of sensors on each particle

    // Constants (fixed values not shown in Inspector)
    private const int MapLayer = 6; // Layer number for the map
    private const int ParticleLayer = 7; // Layer number for particles
    private const float shiftUpParticles = 0.25f; // Height offset for particle visualization

    // Debug fields (hidden unless debugging is enabled)
    [Header("Debug Settings (Optional)")]
    [Tooltip("Show debug particles if enabled.")]
    public bool enableDebugParticles = false;

    private List<Transform> sensorShow = null; // Sensors to show for debugging
    private List<LineRenderer> sensorLineRenderShow = null; // Line renderers for sensor rays in debug mode
    private GameObject debugParticle = null; // Debug particle object

    private void Start()
    {
        // chart.ClearData(); // Clear chart data at the start
        // chart.SetMaxCache(20); // Set maximum cache for chart to 20

        mapLayerMask = 1 << MapLayer; // Set the map layer mask
        particleLayerMask = 1 << ParticleLayer; // Set the particle layer mask

        nParticles = Mathf.Clamp(nParticles, minParticles, maxParticles); // Clamp the number of particles within allowed limits

        nSensors = particlePrefab.transform.Find(sensorsParentName).childCount; // Get the number of sensors from the particle prefab
        debugParticle = Instantiate(particlePrefab); // Create a debug particle instance
        debugParticle.SetActive(false); // Disable debug particle initially

        InitializeMapBounds(); // Initialize the boundaries of the map
        InitializeParticleSystem(); // Initialize the particle system
        InitializeRobotSensors(); // Initialize the sensors on the robot
        InvokeRepeating(nameof(AMCLUpdate), 0.0f, UpdateInterval); // Schedule periodic updates for the AMCL algorithm
    }

    private void Update()
    {
        // Handle debug particle visualization
        if (enableDebugParticles && Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity, particleLayerMask))
            {
                // Show sensors for the selected particle
                Transform sensorsParent = hit.collider.transform.parent.Find(sensorsParentName);
                sensorShow = new List<Transform>(nSensors);
                for (int i = 0; i < nSensors; i++)
                {
                    sensorShow.Add(sensorsParent.GetChild(i).transform);
                }
                if (sensorLineRenderShow == null)
                {
                    // Create line renderers for visualizing sensor rays
                    sensorLineRenderShow = new List<LineRenderer>(nSensors);
                    for (int i = 0; i < nSensors; i++)
                    {
                        sensorLineRenderShow.Add(Instantiate(lineRend));
                    }
                }
            }
            else if (sensorShow != null)
            {
                // Disable sensor visualization if no particle is selected
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
                // Update debug particle position
                debugParticle.transform.position = hit.point + Vector3.up * shiftUpParticles;

                if (!debugParticle.activeSelf)
                {
                    // Activate debug particle and initialize sensor visualization
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
                    // Cast rays from sensors and update line renderers
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
                float[] robotDistances = new float[nSensors];
                GetRaycastDistances(robotSensorsOrigins, robotDistances);

                float[] particleDist = new float[nSensors];
                GetRaycastDistances(sensorShow.ToArray(), particleDist);

                float weight = 1.0f;

                // Calculate the weight of the particle based on sensor distances (Gaussian likelihood)
                for (int j = 0; j < robotDistances.Length; j++)
                {
                    float distanceError = robotDistances[j] - particleDist[j];
                    float a = -(distanceError * distanceError) / (2 * sensorVariance * sensorVariance);
                    weight *= Mathf.Exp(a); // Gaussian likelihood
                }
            }
            else if (sensorShow != null)
            {
                // Disable sensor visualization if no valid hit
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
            // Rotate debug particle with mouse scroll
            debugParticle.transform.Rotate(Vector3.up * Input.mouseScrollDelta.y);
        }
    }

    // Initializes the map boundaries based on ProBuilder mesh
    private void InitializeMapBounds()
    {
        Vertex[] vertices = mapArea.GetComponent<ProBuilderMesh>().GetVertices();
        minX = minZ = float.MaxValue;
        maxX = maxZ = float.MinValue;

        // Find the min and max coordinates for the map
        foreach (var vertice in vertices)
        {
            Vector3 pos = vertice.position;
            minX = Mathf.Min(minX, pos.x);
            maxX = Mathf.Max(maxX, pos.x);
            minZ = Mathf.Min(minZ, pos.z);
            maxZ = Mathf.Max(maxZ, pos.z);
        }
    }

    public void ResetSimulation(string particleCountStr)
    {
        int newParticleCount;
        if (int.TryParse(particleCountStr, out newParticleCount))
        {
            nParticles = Mathf.Clamp(newParticleCount, minParticles, maxParticles);
            InitializeParticleSystem();
        }
    }

    // Sets up particle objects and sensors for the robot
    private void InitializeParticleSystem()
    {
        // Clear existing particles and objects
        if (particleObjects != null)
        {
            foreach (var obj in particleObjects)
            {
                Destroy(obj);
            }
        }

        particleObjects = new List<GameObject>(nParticles);
        sensorsOrigins = new List<Transform[]>(nParticles);
        particles = new List<Particle>(nParticles);

        lastPosition = robot.transform.position;
        lastAngle = robot.transform.rotation.eulerAngles.y;

        // Initialize particle GameObjects and sensor references
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
        GenerateParticles(); // Generate initial particles
    }

    private void InitializeRobotSensors()
    {
        // Initialize references to the robot's sensors
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
        deltaPosition.y = 0f; // Ignore changes in the y-axis (assuming a 2D plane)
        float deltaTheta = robot.transform.rotation.eulerAngles.y - lastAngle;
        Vector3 relativeDeltaPosition = robot.transform.InverseTransformDirection(deltaPosition);

        if (deltaPosition.sqrMagnitude < 0.04f) // Using sqrMagnitude for efficiency (0.2^2 = 0.04)
        {
            return; // Skip update if robot movement is negligible
        }

        UpdateParticlePositions(relativeDeltaPosition, deltaTheta); // Update particle positions

        // Step 2: Get robot sensor readings
        float[] robotDistances = new float[nSensors];
        GetRaycastDistances(robotSensorsOrigins, robotDistances);

        // Step 3: Compute particle weights based on sensor data likelihood
        CalculateWeight(robotDistances);

        // Step 4: Resample particles based on weights
        ResampleParticles();

        // Step 3.5: Calculate probable position based on weights
        CalculateProbablePosition();

        // Step 5: Ensure particles are on the ground
        EnsureParticlesOnGround();

        // Update robot's last known position and angle for next iteration
        lastPosition = robot.transform.position;
        lastAngle = robot.transform.rotation.eulerAngles.y;

        // Update sensor visualization for debugging
        if (enableDebugParticles && sensorShow != null)
        {
            int sensorCount = sensorShow.Count;
            for (int i = 0; i < sensorCount; i++)
            {
                Transform sensor = sensorShow[i];
                LineRenderer lineRenderer = sensorLineRenderShow[i];

                if (Physics.Raycast(sensor.position, sensor.up, out RaycastHit hit, SensorMaxDistance, mapLayerMask))
                {
                    lineRenderer.enabled = true;
                    lineRenderer.SetPosition(0, sensor.position);
                    lineRenderer.SetPosition(1, hit.point);
                }
                else
                {
                    lineRenderer.enabled = false;
                }
            }
        }
    }

    private void CalculateProbablePosition()
    {
        // int topParticleCount = Mathf.Min(particles.Count, particles.Count);
        int topParticleCount = Mathf.Min(100, particles.Count);

        // Sort particles by weight in descending order
        if (particles.Count != topParticleCount)
        {
            particles.Sort((p1, p2) => p2.weight.CompareTo(p1.weight));
        }

        Vector3 weightedPositionSum = Vector3.zero;
        float weightedOrientationSum = 0f;
        float totalWeight = 0f;

        // Sum weighted positions and orientations of top particles
        for (int i = 0; i < topParticleCount; i++)
        {
            Particle particle = particles[i];
            float weight = particle.weight;
            weightedPositionSum += particle.position * weight;
            weightedOrientationSum += particle.orientation * weight;
            totalWeight += weight;

            // Activate and update the particle visualization
            UpdateParticleVisualization(particleObjects[i], particle);
        }

        // Deactivate the rest of the particle objects
        for (int i = topParticleCount; i < particleObjects.Count; i++)
        {
            // particleObjects[i].SetActive(false);
        }

        if (totalWeight > 0f)
        {
            float invTotalWeight = 1f / totalWeight;
            Vector3 probablePosition = weightedPositionSum * invTotalWeight;
            float probableOrientation = weightedOrientationSum * invTotalWeight;

            Vector3 shiftUpVector = Vector3.up * shiftUpParticles;
            probablePos.transform.position = probablePosition + shiftUpVector;
            probablePos.transform.rotation = Quaternion.Euler(0f, probableOrientation, 0f);
        }
    }


    // Ensure particles are on the ground, repositioning if necessary
    private void EnsureParticlesOnGround()
    {
        int particleCount = particles.Count;
        for (int i = 0; i < particleCount; i++)
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
            randomX = Random.Range(minX, maxX);
            randomZ = Random.Range(minZ, maxZ);
            Vector3 rayOrigin = new Vector3(randomX, Camera.main.transform.position.y, randomZ);
            Ray ray = new Ray(rayOrigin, Vector3.down);

            // Perform a raycast to check for ground
            if (Physics.Raycast(ray, out hit, Mathf.Infinity, mapLayerMask) && hit.collider.CompareTag(mapArea.tag))
            {
                // If we hit the ground, set the particle's new position and orientation
                particle.position = new Vector3(hit.point.x, 0f, hit.point.z);
                particle.orientation = Random.Range(0f, 360f); // Random orientation as well
                particle.weight = 1.0f / nParticles; // Update weight if needed

                Vector3 shiftUpVector = Vector3.up * shiftUpParticles;
                particleObjects[index].transform.position = particle.position + shiftUpVector;
                particleObjects[index].transform.rotation = Quaternion.Euler(0f, particle.orientation, 0f);

                break; // Exit the loop when a valid position is found
            }
        }
    }

    // Function to check if the particle is on the ground
    private bool IsOnGround(Vector3 position)
    {
        Vector3 rayOrigin = new Vector3(position.x, Camera.main.transform.position.y, position.z);
        Ray ray = new Ray(rayOrigin, Vector3.down);
        if (Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity, mapLayerMask) && hit.collider.CompareTag(mapArea.tag))
        {
            return true;
        }
        return false;
    }

    // Improved weight calculation using log-likelihood for numerical stability
    private void CalculateWeight(float[] robotDistances)
    {
        int particleCount = particles.Count;
        int numSensors = robotDistances.Length;
        float invTwoSensorVarianceSquared = 1f / (2f * sensorVariance * sensorVariance);

        float totalLogWeight = float.NegativeInfinity;

        // Preallocate the particle distance array
        float[] particleDist = new float[numSensors];

        for (int i = 0; i < particleCount; i++)
        {
            GetRaycastDistances(sensorsOrigins[i], particleDist);
            float logWeight = 0f;

            // Calculate log weight for each sensor reading
            for (int j = 0; j < numSensors; j++)
            {
                float distanceError = robotDistances[j] - particleDist[j];
                float errorSquared = distanceError * distanceError;
                logWeight += -errorSquared * invTwoSensorVarianceSquared;
            }

            particles[i].weight = logWeight;
            totalLogWeight = LogSum(totalLogWeight, logWeight); // Accumulate log weights for normalization
        }

        // Convert log weights back to probabilities and normalize
        float totalWeight = 0f;
        for (int i = 0; i < particleCount; i++)
        {
            float weight = Mathf.Exp(particles[i].weight - totalLogWeight);
            particles[i].weight = weight;
            totalWeight += weight;
        }

        // Normalize weights
        float invTotalWeight = 1f / totalWeight;
        for (int i = 0; i < particleCount; i++)
        {
            particles[i].weight *= invTotalWeight;
        }
    }

    private int resampleFrameCount = 0; // Track frames for periodic chart updates
    private List<Particle> newParticles = new List<Particle>();

    private void ResampleParticles()
    {
        // Step 1: Calculate Effective Sample Size (ESS)
        // float ess = CalculateEffectiveSampleSize();
        // resampleFrameCount++;

        // Step 2: Update chart data every 5 frames
        // if (resampleFrameCount % 5 == 0)
        // {
        //     chart.AddXAxisData("x" + resampleFrameCount);
        //     chart.AddData(0, ess);
        // }

        // Step 3: Determine target particle count
        int targetParticleCount = nParticles;
        targetParticleCount = Mathf.Clamp(targetParticleCount, minParticles, maxParticles);

        // Step 4: Calculate number of random particles to inject
        int randomCount = Mathf.RoundToInt(randomParticleRatio * targetParticleCount);
        int resampledCount = targetParticleCount - randomCount;

        // Step 5: Initialize newParticles list
        newParticles.Clear();
        newParticles.Capacity = targetParticleCount;

        // Step 6: Generate random particles first
        for (int m = 0; m < randomCount; m++)
        {
            Particle newParticle = CreateRandomParticle();
            newParticles.Add(newParticle);
            UpdateParticleVisualization(particleObjects[m], newParticle); // Update visual representation
            // particleObjects[m].SetActive(false); // To not show particles that are randomly placed
        }

        // Step 7: Perform systematic resampling for the remaining particles
        float M_inv = 1.0f / resampledCount;
        float r = Random.Range(0f, M_inv);
        float c = particles[0].weight;
        int i = 0;

        for (int m = 0; m < resampledCount; m++)
        {
            float U = r + m * M_inv;
            while (U > c && i < particles.Count - 1)
            {
                i++;
                c += particles[i].weight;
            }
            Particle newParticle = CloneParticleWithNoise(particles[i]); // Add noise to cloned particles
            newParticles.Add(newParticle);
            UpdateParticleVisualization(particleObjects[m + randomCount], newParticle); // Update visual representation
        }

        // Step 8: Deactivate unused particle objects
        for (int m = targetParticleCount; m < maxParticles; m++)
        {
            particleObjects[m].SetActive(false);
        }

        // Step 9: Finalize updates
        particles = new List<Particle>(newParticles); // Create a new list to avoid modifying the original reference
        nParticles = targetParticleCount;
    }

    // Helper function to create a random particle
    private Particle CreateRandomParticle()
    {
        float randomX, randomZ;
        RaycastHit hit;

        // Generate random positions until a valid position is found
        while (true)
        {
            randomX = Random.Range(minX, maxX);
            randomZ = Random.Range(minZ, maxZ);
            Vector3 rayOrigin = new Vector3(randomX, Camera.main.transform.position.y, randomZ);
            Ray ray = new Ray(rayOrigin, Vector3.down);

            // Perform a raycast to find a valid position on the ground
            if (Physics.Raycast(ray, out hit, Mathf.Infinity, mapLayerMask) && hit.collider.CompareTag(mapArea.tag))
            {
                return new Particle(new Vector3(hit.point.x, 0f, hit.point.z), Random.Range(0f, 360f), 1.0f / nParticles);
            }
        }
    }

    // Helper function to clone a particle with added noise
    private Particle CloneParticleWithNoise(Particle sourceParticle)
    {
        float noiseX = RandomGaussian(0f, PositionNoiseStdDev);
        float noiseZ = RandomGaussian(0f, PositionNoiseStdDev);
        float noiseTheta = RandomGaussian(0f, OrientationNoiseStdDev);

        Vector3 newPosition = sourceParticle.position + new Vector3(noiseX, 0f, noiseZ);
        float newOrientation = NormalizeAngle(sourceParticle.orientation + noiseTheta);

        return new Particle(newPosition, newOrientation, sourceParticle.weight);
    }

    // Helper function to update particle visualization
    private void UpdateParticleVisualization(GameObject particleObject, Particle particle)
    {
        Vector3 shiftUpVector = Vector3.up * shiftUpParticles;
        particleObject.transform.position = particle.position + shiftUpVector;
        particleObject.transform.rotation = Quaternion.Euler(0f, particle.orientation, 0f);
        particleObject.SetActive(true);
    }

    // Generate random particles within the map area
    private void GenerateParticles()
    {
        particles.Clear();
        int nParticleCreated = 0;
        Vector3 shiftUpVector = Vector3.up * shiftUpParticles;

        while (nParticleCreated < maxParticles)
        {
            float randomX = Random.Range(minX, maxX);
            float randomZ = Random.Range(minZ, maxZ);
            Vector3 rayOrigin = new Vector3(randomX, Camera.main.transform.position.y, randomZ);
            Ray ray = new Ray(rayOrigin, Vector3.down);

            // Generate particles at valid positions within the map
            if (Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity, mapLayerMask) && hit.collider.CompareTag(mapArea.tag))
            {
                Particle particle = new Particle(new Vector3(hit.point.x, 0f, hit.point.z), Random.Range(0f, 360f), 1.0f / nParticles);
                particles.Add(particle);

                GameObject particleObj = particleObjects[nParticleCreated];
                particleObj.transform.position = particle.position + shiftUpVector;
                particleObj.transform.rotation = Quaternion.Euler(0f, particle.orientation, 0f);
                particleObj.SetActive(true); // Show the particle
                nParticleCreated++;
            }
        }
    }

    // Update particle positions and orientations based on robot's movement
    private void UpdateParticlePositions(Vector3 deltaPosition, float deltaTheta)
    {
        // Precompute constants
        Vector3 shiftUpVector = Vector3.up * shiftUpParticles;
        int particleCount = particles.Count;

        for (int i = 0; i < particleCount; i++)
        {
            Particle particle = particles[i];

            // Generate noise for position and orientation
            float noiseX = RandomGaussian(0f, PositionNoiseStdDev);
            float noiseZ = RandomGaussian(0f, PositionNoiseStdDev);
            float noiseTheta = RandomGaussian(0f, OrientationNoiseStdDev);

            Vector3 noisyDeltaPosition = deltaPosition + new Vector3(noiseX, 0f, noiseZ);
            float noisyDeltaTheta = deltaTheta + noiseTheta;

            // Update particle orientation
            particle.orientation = NormalizeAngle(particle.orientation + noisyDeltaTheta);

            // Compute rotation quaternion
            Quaternion particleRotation = Quaternion.Euler(0f, particle.orientation, 0f);

            // Transform noisyDeltaPosition to world space
            Vector3 worldDeltaPosition = particleRotation * noisyDeltaPosition;

            // Update particle position
            particle.position += worldDeltaPosition;

            // Update particle object's transform
            Vector3 newParticlePosition = particle.position + shiftUpVector;
            particleObjects[i].transform.SetPositionAndRotation(newParticlePosition, particleRotation);
        }
    }

    private void GetRaycastDistances(Transform[] sensorsOrigins, float[] distances)
    {
        int sensorCount = sensorsOrigins.Length;
        for (int i = 0; i < sensorCount; i++)
        {
            Transform sensor = sensorsOrigins[i];
            Vector3 origin = sensor.position;
            Vector3 direction = sensor.up;

            // Perform raycast to determine the distance to obstacles from each sensor
            if (Physics.Raycast(origin, direction, out RaycastHit hit, SensorMaxDistance, mapLayerMask))
            {
                distances[i] = hit.distance;
            }
            else
            {
                distances[i] = SensorMaxDistance; // Set max distance if no hit
            }
        }
    }

    // Utility functions
    private float CalculateEffectiveSampleSize()
    {
        float weightSquareSum = 0f;
        int startIndex = Mathf.RoundToInt(randomParticleRatio * nParticles);
        for (int i = startIndex; i < nParticles; i++)
        {
            float weight = particles[i].weight;
            weightSquareSum += weight * weight;
        }
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
        angle %= 360f;
        if (angle < 0f) angle += 360f;
        return angle;
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

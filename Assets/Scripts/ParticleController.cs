using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.ProBuilder;
using Random = UnityEngine.Random;

public class ParticleController : MonoBehaviour
{
    public GameObject robot;
    public GameObject mapArea;
    public GameObject particlePrefab;
    public int nParticles = 10;

    public LineRenderer lineRend;
    private Transform[] robotSensorsOrigins;
    private List<Transform[]> sensorsOrigins;
    private List<Particle> particles; // Particle data
    private List<GameObject> particleObjects; // Visualization of particles
    private float minX, maxX, minZ, maxZ;
    private Vector3 lastPosition;
    private float lastAngle;
    private LayerMask mapLayerMask;
    private LayerMask particleLayerMask;

    // Constants
    public float PositionNoiseStdDev = 0.1f;
    public float OrientationNoiseStdDev = 1.0f;
    public float UpdateInterval = 0.2f;
    private const float SensorMaxDistance = 50f; // Limit raycast distance
    private const int MapLayer = 6;
    private const int ParticleLayer = 7;
    private const float shiftUpParticles = 0.25f;

    private List<Transform> sensorShow = null;
    private List<LineRenderer> sensorLineRenderShow = null;

    private void Start()
    {
        mapLayerMask = 1 << MapLayer;
        particleLayerMask = 1 << ParticleLayer;

        InitializeMapBounds();
        InitializeParticleSystem();
        InitializeRobotSensors();
        InvokeRepeating(nameof(AMCLUpdate), 0.0f, UpdateInterval);
    }

    private void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Debug.Log("RUNNING!!!!");

            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity, particleLayerMask))
            {
                Transform sensorsParent = hit.collider.transform.parent.Find("DistanceSensors");
                sensorShow = new List<Transform>(sensorsParent.childCount);
                for (int i = 0; i < sensorsParent.childCount; i++)
                {
                    sensorShow.Add(sensorsParent.GetChild(i).transform);
                }
                if (sensorLineRenderShow == null)
                {
                    sensorLineRenderShow = new List<LineRenderer>(sensorsParent.childCount);
                    for (int i = 0; i < sensorsParent.childCount; i++)
                    {
                        sensorLineRenderShow.Add(Instantiate(lineRend));
                    }
                }
            }
            else
            {
                sensorShow = null;
            }
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

        for (int i = 0; i < nParticles; i++)
        {
            GameObject particleObj = Instantiate(particlePrefab, Vector3.zero, Quaternion.identity);
            particleObj.SetActive(false); // Initially hidden
            particleObj.transform.parent = transform; // Clean hierarchy
            particleObjects.Add(particleObj);


            Transform sensorsParent = particleObj.transform.Find("DistanceSensors");

            Transform[] particleSensors = new Transform[sensorsParent.childCount];

            for (int j = 0; j < sensorsParent.childCount; j++)
            {
                particleSensors[j] = sensorsParent.GetChild(j);
            }
            sensorsOrigins.Add(particleSensors);
        }
        GenerateParticles();
    }

    private void InitializeRobotSensors()
    {
        Transform sensorsParent = robot.transform.Find("DistanceSensors");
        robotSensorsOrigins = new Transform[sensorsParent.childCount];
        for (int i = 0; i < sensorsParent.childCount; i++)
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
        Debug.Log("" + relativeDeltaPosition.x + "    " + relativeDeltaPosition.z);

        UpdateParticlePositions(relativeDeltaPosition, deltaTheta); // Update particle positions

        if (sensorShow != null)
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

        // Reset particle set on spacebar press (for debugging)
        if (Input.GetKeyDown(KeyCode.Space))
        {
            GenerateParticles();
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
                particle.position = hit.point;
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


    // Function to calculate particle weight using a likelihood model
    private void CalculateWeight(float[] robotDistances)
    {
        float weight = 1.0f;
        float variance = 50.0f; // Sensor noise variance (tunable parameter)
        float totalWeight = 0.0f;

        for (int i = 0; i < particles.Count; i++)
        {
            float[] particleDist = GetRaycastDistances(sensorsOrigins[i]);

            for (int j = 0; j < robotDistances.Length; j++)
            {
                float distanceError = robotDistances[j] - particleDist[j];
                float a = -(distanceError * distanceError) / (2 * variance * variance);
                weight *= Mathf.Exp(a); // Gaussian likelihood
            }

            particles[i].weight = Mathf.Max(weight, 0);
            totalWeight = Mathf.Max(particles[i].weight, totalWeight);
        }

        for (int i = 0; i < particles.Count; i++)
        {
            particles[i].weight /= totalWeight; // Normalize the weight
        }

    }

    private void ResampleParticles()
    {
        List<Particle> newParticles = new List<Particle>(nParticles);

        // Step 2: Resample particles
        for (int i = 0; i < nParticles; i++)
        {
            float randomValue = Random.Range(0f, 1f); // Random number between 0 and total weight
            int tryIdx = Random.Range(0, nParticles);

            // Find the index of the particle corresponding to the random value
            while (particles[tryIdx].weight < randomValue)
            {
                tryIdx = Random.Range(0, nParticles);
                randomValue = Random.Range(0f, 1f);
            }

            // Clone the selected particle and add slight noise to position and orientation
            Particle resampledParticle = CloneParticle(particles[tryIdx]);

            // Add random noise to the position and orientation to avoid clustering
            resampledParticle.position += new Vector3(RandomGaussian(0, PositionNoiseStdDev), 0, RandomGaussian(0, PositionNoiseStdDev));
            resampledParticle.orientation += RandomGaussian(0, OrientationNoiseStdDev); // Adjust noise as needed

            // Update particle object position and rotation
            particleObjects[i].transform.position = resampledParticle.position + Vector3.up * shiftUpParticles;
            particleObjects[i].transform.rotation = Quaternion.Euler(0, resampledParticle.orientation, 0);

            // Add the new particle to the new list
            newParticles.Add(resampledParticle);
        }

        // Replace the old particles with the new resampled set
        particles = newParticles;
    }


    // Helper method to clone a particle, instantiate a new particle object, and update sensor origins
    private Particle CloneParticle(Particle particle)
    {
        // Apply small random noise to position and orientation
        Vector3 noisyPosition = new Vector3(particle.position.x, particle.position.y, particle.position.z);
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

        while (nParticleCreated < nParticles)
        {
            float randomX = Random.Range(minX, maxX);
            float randomZ = Random.Range(minZ, maxZ);
            Vector3 rayOrigin = new Vector3(randomX, Camera.main.transform.position.y, randomZ);
            Ray ray = new Ray(rayOrigin, Vector3.down);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, Mathf.Infinity, mapLayerMask) && hit.collider.CompareTag(mapArea.tag))
            {
                Particle particle = new Particle(hit.point, Random.Range(0f, 360f), 1.0f / nParticles);
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
            int index = particles.IndexOf(particle);
            particleObjects[index].transform.position = particle.position + Vector3.up * shiftUpParticles;
            particleObjects[index].transform.rotation = Quaternion.Euler(0, particle.orientation, 0);
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

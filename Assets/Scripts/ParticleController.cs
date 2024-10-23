using System.Collections.Generic;
using UnityEngine;
using UnityEngine.ProBuilder;

public class ParticleController : MonoBehaviour
{
    public GameObject robot;
    public GameObject mapArea;
    public GameObject particlePrefab;
    public int nParticles = 10;

    private Transform[] robotSensorsOrigins;
    private List<Particle> particles; // Particle data
    private List<GameObject> particleObjects; // Visualization of particles
    private float minX, maxX, minZ, maxZ;
    private Vector3 lastPosition;
    private float lastAngle;
    private LayerMask mapLayerMask;

    // Constants
    private const float UpdateInterval = 0.2f;
    private const float SensorMaxDistance = 50f; // Limit raycast distance
    private const float PositionNoiseStdDev = 0.1f;
    private const float OrientationNoiseStdDev = 1.0f;
    private const int MapLayer = 6;

    private void Start()
    {
        InitializeMapBounds();
        InitializeParticleSystem();
        InitializeRobotSensors();
        InvokeRepeating(nameof(AMCLUpdate), 0.0f, UpdateInterval);
    }

    // Initializes the map boundaries based on ProBuilder mesh
    private void InitializeMapBounds()
    {
        mapLayerMask = 1 << MapLayer;
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
        particles = new List<Particle>(nParticles);
        lastPosition = robot.transform.position;
        lastAngle = robot.transform.rotation.eulerAngles.y;

        for (int i = 0; i < nParticles; i++)
        {
            GameObject particleObj = Instantiate(particlePrefab, Vector3.zero, Quaternion.identity);
            particleObj.SetActive(false); // Initially hidden
            particleObj.transform.parent = transform; // Clean hierarchy
            particleObjects.Add(particleObj);
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
        Vector3 deltaPosition = robot.transform.position - lastPosition;
        deltaPosition.y = 0;
        float deltaTheta = robot.transform.rotation.eulerAngles.y - lastAngle;

        UpdateParticlePositions(deltaPosition, deltaTheta);

        // Robot raycast distances
        float[] robotDistances = GetRaycastDistances(robotSensorsOrigins);

        // Particles raycast distances
        List<float[]> particleDistances = new List<float[]>(particles.Count);
        foreach (var particle in particles)
        {
            particleDistances.Add(GetRaycastDistances(particle.sensorsOrigins));
        }

        lastPosition = robot.transform.position;
        lastAngle = robot.transform.rotation.eulerAngles.y;

        // Reset particle set on spacebar press
        if (Input.GetKeyDown(KeyCode.Space))
        {
            GenerateParticles();
        }
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
                SetupParticleSensors(particle, nParticleCreated);
                nParticleCreated++;
            }
        }
    }

    private void SetupParticleSensors(Particle particle, int index)
    {
        Transform sensorsParent = particleObjects[index].transform.Find("DistanceSensors");
        particle.sensorsOrigins = new Transform[sensorsParent.childCount];
        for (int i = 0; i < sensorsParent.childCount; i++)
        {
            particle.sensorsOrigins[i] = sensorsParent.GetChild(i);
        }

        particleObjects[index].transform.position = particle.position + Vector3.up * 0.5f;
        particleObjects[index].transform.rotation = Quaternion.Euler(0, particle.orientation, 0);
        particleObjects[index].SetActive(true); // Show the particle
        particles.Add(particle);
    }

    // Update particle positions and orientations based on robot's movement
    private void UpdateParticlePositions(Vector3 deltaPosition, float deltaTheta)
    {
        foreach (var particle in particles)
        {
            Vector3 noisyDeltaPosition = deltaPosition + new Vector3(RandomGaussian(0, PositionNoiseStdDev), 0, RandomGaussian(0, PositionNoiseStdDev));
            float noisyDeltaTheta = deltaTheta + RandomGaussian(0, OrientationNoiseStdDev);

            particle.position += noisyDeltaPosition;
            particle.orientation = NormalizeAngle(particle.orientation + noisyDeltaTheta);

            // Update particle object's position in the scene
            int index = particles.IndexOf(particle);
            particleObjects[index].transform.position = particle.position;
            particleObjects[index].transform.rotation = Quaternion.Euler(0, particle.orientation, 0);
        }
    }

    private float[] GetRaycastDistances(Transform[] sensorsOrigins)
    {
        float[] distances = new float[sensorsOrigins.Length];
        for (int i = 0; i < sensorsOrigins.Length; i++)
        {
            if (Physics.Raycast(sensorsOrigins[i].position, sensorsOrigins[i].forward, out RaycastHit hit, SensorMaxDistance, mapLayerMask))
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
    public Transform[] sensorsOrigins;

    public Particle(Vector3 pos, float orient, float w)
    {
        position = pos;
        orientation = orient;
        weight = w;
    }
}

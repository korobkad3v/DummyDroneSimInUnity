using UnityEngine;

public class WindForce : MonoBehaviour
{
    public Transform windZoneCenter;
    private WindZone _windZone;  
    private Rigidbody _rigidbody; 
    private PhysicsMaterial _physicsMaterial;
    private Vector3 frictionForceToApply; 
    private Vector3 windForceToApply;
    private bool isTouchingSomething = false;
    

    void Start()
    {
        _rigidbody = GetComponent<Rigidbody>();
        _physicsMaterial = GetComponent<Collider>().material;
        _windZone = windZoneCenter.GetComponent<WindZone>();
    }

    void FixedUpdate()
    {
                
        switch (_windZone.mode)
        {
            case WindZoneMode.Spherical:
                WindSphericalForce();
                break;
            case WindZoneMode.Directional:
                WindDirectionalForce();
                break;
        }

        if (isTouchingSomething) 
        {
            ApplyAirFriction();
        }
        
    }

    void Update()
    {
        Debug.Log($"Friction force: {frictionForceToApply.magnitude}\nWind force: {_windZone.windMain}");
    }

    void OnCollisionEnter(Collision other) 
    {
        isTouchingSomething = true; 
    }

    void OnCollisionStay(Collision other) 
    {
        
        PhysicsMaterial otherPhysicalMaterial = other.gameObject.GetComponent<Collider>().material;
        if (otherPhysicalMaterial != null) {
            float normalForce = _rigidbody.mass * Physics.gravity.magnitude;
            frictionForceToApply = -_rigidbody.linearVelocity.normalized * otherPhysicalMaterial.dynamicFriction * _physicsMaterial.dynamicFriction * normalForce;
        }
        else {
            Debug.LogError("No PhysicsMaterial component found on " + other.gameObject.name);
        }
        
        
    }

    void OnCollisionExit(Collision other)
    {
        isTouchingSomething = false; 
    }

    void WindSphericalForce() 
    { 
        Vector3 windDirection = _rigidbody.position - windZoneCenter.position;
        float distanceToWind = windDirection.magnitude;

        if (distanceToWind < _windZone.radius)
        {
            windDirection.Normalize();
            float forceMagnitude = _windZone.windMain * (1 - (distanceToWind / _windZone.radius));

            if (frictionForceToApply == Vector3.zero) 
            {
                frictionForceToApply = -_rigidbody.linearVelocity.normalized * _physicsMaterial.dynamicFriction * _rigidbody.mass * Physics.gravity.magnitude;
                
            }
            windForceToApply = windDirection * forceMagnitude;
            _rigidbody.AddForce(frictionForceToApply);
            _rigidbody.AddForce(windForceToApply);
        }
    }

    void WindDirectionalForce() 
    { 
        Vector3 windDirection = _windZone.transform.forward;

        float force = _windZone.windMain;

        if (frictionForceToApply == Vector3.zero) 
        {
            frictionForceToApply = -_rigidbody.linearVelocity.normalized * _physicsMaterial.dynamicFriction * _rigidbody.mass * Physics.gravity.magnitude;
           
        }
        windForceToApply = windDirection * force;
        _rigidbody.AddForce(frictionForceToApply);
        _rigidbody.AddForce(windForceToApply);
    }
    
    void ApplyAirFriction()
    {
        // Расчет трения с воздухом
        float airDragCoefficient = 0.47f; 
        Vector3 airResistance = -_rigidbody.linearVelocity .normalized * airDragCoefficient * _rigidbody.linearVelocity.sqrMagnitude;
        _rigidbody.AddForce(airResistance); 
    }
}
#version 440 core



struct AmbientLight
{
    vec3 color;
    bool isOn;
};

struct DiffuseLight
{
    vec3 color;
    vec3 direction;
    float factor;
    bool isOn;
};

struct FogParameters
{
    vec3 color; // Color to be used with fog, usually grayish
    float linearStart; // This is where linear fog starts (valid for linear equation only)
    float linearEnd; // This is where linear fog ends (valid for linear equation only)
    float density; // Density of the fog, used by exp and exp2 equation
    
    int equation; // Used fog equation, 3 options are valid - 0 = linear, 1 = exp, 2 = exp2
    bool isEnabled; // Flag telling if fog is enabled or not
};


vec3 getAmbientLightColor(AmbientLight ambientLight)
{
    return ambientLight.isOn ? ambientLight.color : vec3(0.0, 0.0, 0.0);
}

vec3 getDiffuseLightColor(DiffuseLight diffuseLight, vec3 normal)
{
    if(!diffuseLight.isOn) {
        return vec3(0.0, 0.0, 0.0);
    }
    
    float finalIntensity = max(0.0, dot(normal, -diffuseLight.direction));
    finalIntensity = clamp(finalIntensity*diffuseLight.factor, 0.0, 1.0);
    return vec3(diffuseLight.color*finalIntensity);
}


float getFogFactor(FogParameters params, float fogCoordinate)
{
    float result = 0.0;
    if(params.equation == 0)
    {
        float fogLength = params.linearEnd - params.linearStart;
        result = (params.linearEnd - fogCoordinate) / fogLength;
    }
    else if(params.equation == 1) {
        result = exp(-params.density * fogCoordinate);
    }
    else if(params.equation == 2) {
        result = exp(-pow(params.density * fogCoordinate, 2.0));
    }
    
    result = 1.0 - clamp(result, 0.0, 1.0);
    return result;
}


vec3 sumColors(vec3 colorA, vec3 colorB)
{
    vec3 result = colorA+colorB;
    return clamp(result, vec3(0.0, 0.0, 0.0), vec3(1.0, 1.0, 1.0));
}


layout(location = 0) out vec4 outputColor;

smooth in vec2 ioVertexTexCoord;
smooth in vec3 ioVertexNormal;
smooth in vec4 ioEyeSpacePosition;

uniform sampler2D sampler;
uniform vec4 color;

uniform AmbientLight ambientLight;
uniform DiffuseLight diffuseLight;
uniform FogParameters fogParams;

void main()
{
    vec3 normal = normalize(ioVertexNormal);
    vec4 textureColor = texture(sampler, ioVertexTexCoord);
    vec4 objectColor = textureColor*color;
    vec3 lightColor = sumColors(getAmbientLightColor(ambientLight), getDiffuseLightColor(diffuseLight, normal));

    outputColor = objectColor*vec4(lightColor, 1.0);
    
    // Apply fog calculation only if fog is enabled
    if(fogParams.isEnabled)
    {
          float fogCoordinate = abs(ioEyeSpacePosition.z / ioEyeSpacePosition.w);
          outputColor = mix(outputColor, vec4(fogParams.color, 1.0), getFogFactor(fogParams, fogCoordinate));
    }
}
#version 410
out vec4 fragColor;

struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;    
    float shininess;
}; 

struct Light {
    vec3 position;

    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

in vec3 fragPos;  
in vec3 fragNorm; 
in vec2 fragTex;

uniform vec3 viewPos;
uniform Material material;
uniform Light light;
uniform sampler2D texture1;

void main()
{
    vec3 ambient = light.ambient * material.ambient;

    vec3 norm = normalize(fragNorm);
    vec3 lightDir = normalize(light.position - fragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = light.diffuse * (0.5f * material.diffuse);

    vec3 viewDir = normalize(viewPos - fragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 specular = light.specular * (spec * material.specular);

    vec3 result = (ambient + diffuse + specular);
    fragColor = vec4(result, 1.0) * texture(texture1, fragTex);
}
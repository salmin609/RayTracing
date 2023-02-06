/*
 * Sangmin Kim
 * 2022/03/28
 * Raytracing Assignment
 */

#pragma once

#include <vector>

enum MeshType
{
    NONE = 0,
    SPHERE,
    BOX,
    CYLINDER,
    MESH,
    LIGHT,
    MOVING_SPHERE
};

class Obj
{
public:
    enum class Distributor
    {
        Phong,
        GGX,
        Beckman
    };

    bool Intersection(hitRecord& record, RayInfo ray);

    vec3 EvalScattering(vec3 N, vec3 wi, vec3 wo, float t, vec3 textureColor = vec3(-1.f, -1.f, -1.f));
    float D(vec3 m, vec3 N);
    float G(vec3 wi, vec3 wo, vec3 m, vec3 N);
    float G1(vec3 v, vec3 m, vec3 N);
    vec3 SampleBrdf(vec3 N, vec3 wo);
    float PdfBrdf(vec3 N, vec3 wi, vec3 wo);
    vec3 F(vec3 wi, vec3 m);
    vec3 SampleLobe(vec3 A, float c, float angle);

    float CharacteristicFactor(float val);
    void CalculateIndexofRefraction(vec3 wo, vec3 N);

    vec3 BeerLaw(float t, vec3 wo, vec3 N)
    {
        vec3 result{1.f, 1.f, 1.f};

        if(dot(wo, N) < 0.f)
        {
            float e = exp(1);
            result.x = pow(e, t * log(material->Kt.x));
            result.y = pow(e, t * log(material->Kt.y));
            result.z = pow(e, t * log(material->Kt.z));
        }
        return result;
    }

    float sign(float val)
    {
        if (val >= 0.f)
            return 1.f;
        return -1.f;
    }

    MeshData* meshdata;
    mat4 modelTR;
    Material* material;
    vec3 center;
    float area;

    vec3 pos;
    vec3 diagonal;

    vec3 boxMin = vec3(-1.f, -1.f, -1.f);
    vec3 boxMax = vec3(1.f, 1.f, 1.f);

    ivec3 iv;

    float radius = 0.f;
    vec3 cylaxis{0.f, 0.f, 0.f};

    bool isLight = false;

    float tantM;
    float tantV;
    float pd, pr, pt, Pd, Pr, Pt;

    float ab;
    float ag;

    float ni, no, n;
    bool isTextured = false;

    float time0, time1;
    vec3 center0, center1;

    Obj(MeshData* m, Material* b, vec3 temp, MeshType type);
    vec3 Center() { return center; }
    MeshType type;
    Distributor distributor = Distributor::Phong;
};

class LightObj : public Obj
{
public:
    LightObj(MeshData* m, Material* b, vec3 temp, MeshType meshType) : Obj(m, b, temp, meshType)
    {
        vec3 lightCenter = temp;
        vec3 rrr(2.f, 2.f, 2.f);

        isLight = true;
        boxMin = lightCenter - rrr;
        boxMax = lightCenter + rrr;
        radius = 2.f;
        area = 4.f * static_cast<float>(PI) * radius * radius;
        rgb = vec3(5.f, 5.f, 5.f);
        material->Kd = rgb;
        material->Ks = rgb;
    }
    vec3 rgb;
    
};

////////////////////////////////////////////////////////////////////////
// Realtime handles all realtime drawing/interaction
////////////////////////////////////////////////////////////////////////
class Simulation
{
public:

    bool nav;
    char motionkey;
    float speed;

    // Camera/viewing parameters
    vec3 ambient;
    vec3 eye;      // Position of eye for viewing scene
    quat orient;   // Represents rotation of -Z to view direction
    float ry;
    float front, back;
    float spin, tilt;
    float cDist;              // Distance from eye to center of scene
    //float lightSpin, lightTilt, lightDist;

    MeshData* sphMesh;
    MeshData* boxMesh;
    MeshData* cylMesh;
    std::vector<Obj*> triangles;

    int width, height;
    std::vector<Obj*> objs;
    std::vector<Obj*> lights;
    
    vec3 lightEmit[8];
    vec3 lightPosn[8];
		
    quat ViewQuaternion() {
        quat q = angleAxis((tilt-90.0f)*Radians, vec3(1,0,0))
            *conjugate(orient)
            *angleAxis(spin*Radians, vec3(0,0,1));
        return conjugate(q);
    }

    vec3 ViewDirection() {
        vec4 v = toMat4(ViewQuaternion()) * vec4(0.0f, 0.0f, -1.0f, 1.0f);
        return vec3(v[0], v[1], v[2]);
    }

    Simulation();
    
    void setScreen(const int _width, const int _height) { width=_width;  height=_height; }
    void setCamera(const vec3& _eye, const quat& _o, const float _ry)
    { eye=_eye; orient=_o; ry=_ry; }
    void setAmbient(const vec3& _a) { ambient = _a; }
    
    void sphere(const vec3 center, const float r, Material* mat, bool isTextured = false);
    void mSphere(const vec3 center0, const vec3 center1, const float r, Material* mat);
    void box(const vec3 base, const vec3 diag, Material* mat);
    void cylinder(const vec3 base, const vec3 axis, const float radius, Material* mat);

    void triangleMesh(MeshData* meshdata) {
        Obj* obj = new Obj(meshdata, meshdata->mat,vec3(0.f, 0.f, 0.f), MESH);
        //objs.push_back(obj);
        if (meshdata->mat->isLight())
            lights.push_back(obj);

        triangles.push_back(obj);
        
    }
    

};



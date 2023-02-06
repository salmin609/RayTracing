//////////////////////////////////////////////////////////////////////
// Uses the ASSIMP library to read mesh models in of 30+ file types
// into a structure suitable for the raytracer.
////////////////////////////////////////////////////////////////////////

#include <string>
#include <vector>

#include "geom.h"
#include "raytrace.h"

#ifndef NOASSIMP
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

void recurseModelNodes(Scene* scene,
                       const  aiScene* aiscene,
                       const  aiNode* node,
                       const aiMatrix4x4& parentTr,
                       const int level=0);

void Scene::ReadAssimpFile(const std::string& path, const mat4& M)
{
    aiMatrix4x4 modelTr(M[0][0], M[1][0], M[2][0], M[3][0],
                        M[0][1], M[1][1], M[2][1], M[3][1],
                        M[0][2], M[1][2], M[2][2], M[3][2],
                        M[0][3], M[1][3], M[2][3], M[3][3]);
    
    printf("Reading %s\n", path.c_str()); 
    Assimp::Importer importer;
    const aiScene* aiscene = importer.ReadFile(path.c_str(),
                                               aiProcess_Triangulate|aiProcess_GenSmoothNormals);
    if (!aiscene) {
        printf("... Failed to read.\n");
        exit(-1); }
    //aiProcessPreset_TargetRealtime_Quality);
    //printf("HasCameras: %d\n", aiscene->HasCameras());
    //printf("HasMaterials: %d\n", aiscene->HasMaterials());
    //printf("HasMeshes: %d\n", aiscene->HasMeshes());
    //printf("HasTextures: %d\n", aiscene->HasTextures());
    if (!aiscene->mRootNode) {
        printf("Scene has no rootnode.\n");
        exit(-1); }
    recurseModelNodes(this, aiscene, aiscene->mRootNode, modelTr);
}

// Recursively traverses the assimp node hierarchy, accumulating
// modeling transformations, and creating and transforming any meshes
// found.  Meshes comming from assimp can have associated surface
// properties, so each mesh *copies* the current BRDF as a starting
// point and modifies it from the assimp data structure.
void recurseModelNodes(Scene* scene,
                       const aiScene* aiscene,
                       const aiNode* node,
                       const aiMatrix4x4& parentTr,
                       const int level)
{
    // Print line with indentation to show structure of the model node hierarchy.
    //for (int i=0;  i<level;  i++) printf("| ");
    //printf("%s ", node->mName.data);

    // Accumulating transformations while traversing down the hierarchy.
    aiMatrix4x4 childTr = parentTr*node->mTransformation;
    aiMatrix3x3 normalTr = aiMatrix3x3(childTr); // Really should be inverse-transpose for full generality
     
    // Loop through this node's meshes
    for (unsigned int i=0;  i<node->mNumMeshes; ++i) {
        aiMesh* aimesh = aiscene->mMeshes[node->mMeshes[i]];
        //printf("%d:%d ", aimesh->mNumVertices, aimesh->mNumFaces);

        // Extract this node's surface material.
        aiString texPath;
        aiMaterial* mtl = aiscene->mMaterials[aimesh->mMaterialIndex];

        // Assimp can read material colors from the input files, but
        // it seems to invent colors of its own unpredictably so I
        // ignore them.  Textures and texture coordinates seem to work
        // well.
        
        //aiColor3D diff (0.f,0.f,0.f); 
        //aiColor3D spec (0.f,0.f,0.f); 
        //float s;
        // if (AI_SUCCESS == mtl->Get(AI_MATKEY_COLOR_DIFFUSE, diff))
        //     scene->setKd(vec3(diff.r, diff.g, diff.b));
        // if (AI_SUCCESS == mtl->Get(AI_MATKEY_COLOR_SPECULAR, spec))
        //     scene->setKs(vec3(spec.r, spec.g, spec.b));
        // if (AI_SUCCESS == mtl->Get(AI_MATKEY_SHININESS, &s, NULL))
        //     scene->setAlpha(s);
        
        //Material *material = new Material(*scene->currentMat);
        //if (AI_SUCCESS == mtl->GetTexture(aiTextureType_DIFFUSE, 0, &texPath))
        //    material->setTexture(texPath.C_Str());

        // Arrays to hold all vertex and triangle data.
        MeshData* meshdata = new MeshData;
        
        // Loop through all vertices and record the
        // vertex/normal/texture/tangent data with the node's model
        // transformation applied.
        for (unsigned int t=0;  t<aimesh->mNumVertices;  ++t) {
            aiVector3D aipnt = childTr*aimesh->mVertices[t];
            aiVector3D ainrm = aimesh->HasNormals() ? normalTr*aimesh->mNormals[t] : aiVector3D(0,0,1);
            aiVector3D aitex = aimesh->HasTextureCoords(0) ? aimesh->mTextureCoords[0][t] : aiVector3D(0,0,0);
            aiVector3D aitan = aimesh->HasTangentsAndBitangents() ? normalTr*aimesh->mTangents[t] :  aiVector3D(1,0,0);

          
            meshdata->vertices.push_back(VertexData(vec3(aipnt.x, aipnt.y, aipnt.z),
                                                    normalize(vec3(ainrm.x, ainrm.y, ainrm.z)),
                                                    vec2(aitex.x, aitex.y),
                                                    vec3(aitan.x, aitan.y, aitan.z))); }
        
        // Loop through all faces, recording indices
        for (unsigned int t=0;  t<aimesh->mNumFaces;  ++t) 
        {
            aiFace* aiface = &aimesh->mFaces[t];
            for (int i=2;  i<aiface->mNumIndices;  i++) 
            {
                meshdata->triangles.push_back(ivec3(aiface->mIndices[0],
                                                    aiface->mIndices[i-1],
                                                    aiface->mIndices[i]));
            }
        }
        meshdata->mat = scene->currentMat;
        scene->triangleMesh(meshdata);
    }

    //printf("\n");

    // Recurse onto this node's children
    for (unsigned int i=0;  i<node->mNumChildren;  ++i)
      recurseModelNodes(scene, aiscene, node->mChildren[i], childTr, level+1);
}
#endif

#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "SOIL2/SOIL2.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <math.h>
#include <float.h>
#include "Mesh.h"
#include "textModel.h"
using namespace std;

GLint TextureFromFile(const char *path, string directory);

glm::vec3 init_position(0.0f, 0.0f, 0.0f);

int m_index = 0;


float landmark[68][3] = {
	//턱(1~17)
	{ -4.0761f, 10.0902f, 3.3203f },
	{ -3.95030f, 8.32900f, 3.37250f },
	{ -4.3185f, 7.4192f, 2.6055f },
	{ -4.1467f, 6.3208f, 2.6576f },
	{ -3.43240f, 5.20830f, 3.52310f },
	{ -2.8979f, 3.6438f, 3.0276f },
	{ -1.92420f, 3.24940f, 4.07540f },
	{ -1.01360f, 2.86310f, 4.21780f },
	{ 0.0f, 2.77730f, 4.21780f },
	{ 1.01360f, 2.86310f, 4.21780f },
	{ 1.92420f, 3.24940f, 4.07540f },
	{ 2.8979f, 3.6438f, 3.0276f },
	{ 3.43240f, 5.20830f, 3.52310f },
	{ 4.1467f, 6.3208f, 2.6576f },
	{ 4.3185f, 7.4192f, 2.6055f },
	{ 3.95030f, 8.32900f, 3.37250f },
	{ 4.0761f, 10.0902f, 3.3203f },
	//왼쪽눈썹(18~22)
	{ -3.13310f, 15.33660f, 3.65980f },
	{ -2.77050f, 15.51320f, 3.96290f },
	{ -2.13710f, 15.84040f, 4.32180f },
	{ -1.2893f, 15.9927f, 4.5276f },
	{ -0.69910f, 15.76470f, 4.58580f },
	//오른쪽눈썹(23~27)
	{ 0.69910f, 15.76470f, 4.58580f },
	{ 1.2893f, 15.9927f, 4.5276f },
	{ 2.13710f, 15.84040f, 4.32180f },
	{ 2.77050f, 15.51320f, 3.96290f },
	{ 3.13310f, 15.33660f, 3.65980f },
	//콧등(28~31)
	{ 0.0f, 12.41830f, 6.72470f },
	{ 0.0f, 11.79300f, 7.05550f },
	{ 0.0f, 9.76680f, 7.42370f },
	{ 0.0f, 8.41510f, 7.30500f },
	//코밑(32~36)
	{ -1.15670f, 8.36550f, 6.07470f },
	{ -0.78050f, 8.20570f, 6.91820f },
	{ 0.0f, 7.96550f, 7.01180f },
	{ 0.78050f, 8.20570f, 6.91820f },
	{ 1.15670f, 8.36550f, 6.07470f },
	//왼쪽눈(37~42)
/*	{ -57.092274, 39.338158, -144.211258 },
	{ -55.705235, 40.485653, -142.288727 },
	{ -53.753994, 40.641651, -141.701950 },
	{ -51.630730, 39.008049, -142.401291 },
	{ -53.543953, 38.696751, -141.925705 },
	{ -55.854286, 38.766403, -142.621979 }, */

	{ -3.906929, 13.907951, 4.274189 },
	{ -2.958842, 13.688049, 5.396903 },
	{ -2.041803, 13.697922, 5.677214 },
	{ -1.097218, 13.906448, 5.186348 },
	{ -1.960854, 13.271591, 5.491220 },
	{ -2.944272, 13.423185, 5.361156 },

	//오른쪽눈(43~48)
/*
	{ -43.884716, 38.925159, -142.480240 },
	{ -41.644543, 40.653931, -141.673828 },
	{ -39.693306, 40.486671, -142.260269 },
	{ -38.267963, 39.338066, -144.163757 },
	{ -39.568428, 38.765312, -142.588974 },
	{ -41.799007, 38.731426, -141.945618 },
	*/
	{ 1.091165f, 13.896233f, 5.168822f },
	{ 2.075999f, 13.710732f, 5.699061f },
	{ 3.044594f, 13.741562f, 5.439048f },
	{ 3.595925f, 13.882439f, 4.578974f },
	{ 2.996771f, 13.462119f, 5.374627f },
	{ 2.190701f, 13.382497f, 5.616309f },
	//바깥입(49~60)
	/*
	{ -53.592880, 24.515507, -140.897430 },
	{ -51.808598, 27.307562, -139.235596 },
	{ -50.164169, 27.293869, -137.625580 },
	{ -47.699017, 27.279118, -137.265778 },
	{ -45.201157, 27.289036, -137.641098 },
	{ -43.665802, 27.362265, -139.085281 },
	{ -41.786617, 24.513721, -141.046494 },
	{ -44.444057, 20.720566, -139.062653 },
	{ -46.301605, 19.797264, -138.149857 },
	{ -47.699017, 19.624229, -138.123001 },
	{ -49.096424, 19.797630, -138.154099 },
	{ -51.090752, 20.741266, -138.981903 },*/
	
	{ -2.043301, 10.220375, 5.471424 },
	{ -1.299818, 10.432640, 6.106249 },
	{ -0.588149, 10.583138, 6.509957 },
	{ 0.004611, 10.492653, 6.462230 },
	{ 0.597356, 10.583138, 6.509957 },
	{ 1.293100, 10.449799, 6.112343 },
	{ 2.072128, 10.267822, 5.461580 },
	{ 1.533411, 9.862900, 5.563573 },
	{ 0.888973, 9.521477, 5.985585 },
	{ 0.004611, 9.460449, 6.100905 },
	{ -0.856694, 9.504166, 5.991948 },
	{ -1.425313, 9.799026, 5.634323 },

	//안쪽입(61~68)
/*	{ -52.361420, 24.672651, -140.548538 },
	{ -50.180840, 25.518250, -138.197388 },
	{ -47.699017, 25.657272, -137.277985 },
	{ -45.170506, 25.546333, -138.111374 },
	{ -43.241982, 24.639051, -140.732544 },
	{ -44.721066, 22.193333, -139.165054 },
	{ -47.699017, 21.041971, -138.009781 },
	{ -50.674248, 22.036921, -139.051010 }*/
	{ -1.726806, 10.156395, 5.458702 },
	{ -0.685014, 10.143318, 5.959883 },
	{ 0.004611, 10.045204, 6.028680 },
	{ 0.652566, 10.145653, 5.958487 },
	{ 1.930177, 10.184944, 5.358767 },
	{ 0.917475, 10.034180, 5.941374 },
	{ 0.004611, 9.959778, 5.998528 },
	{ -0.894291, 10.018173, 5.945654 }
};
class Model		//mesh 팔다리같은거 다 합쳐논거
{
public:
	int land[68] = { 0 };
	int Size = 0;
	textModel* entire_mesh;
	Model() {

	}
	Model(GLchar *path)
	{
		for (int i = 0; i < 68; i++)
			land[i] = 0;
		this->loadModel(path);
	}
	void Draw(Shader shader)
	{
		for (GLuint i = 0; i < this->meshes.size(); i++)
		{
			this->meshes[i].Draw(shader);
		}
	}
	
	void All_size(int size) {
		Size = size;
		entire_mesh = new textModel[Size];
	}
	void Set_vert() {
		for (int i = 0; i < meshes.size(); i++) {
			meshes[i].Set_vertex(meshes[i].vertices);
		}
	}
	void blending(vector<textModel> models, vector<float> weights, int x) {

			for (int j = 0; j < models.size(); j++) {
				entire_mesh[j] = models[j];
			}
			this->meshes[0].blending(entire_mesh, weights, x);
	}
	
	void Interpolation(float delta, float times) {
		for (int i = 0; i < this->meshes.size(); i++) {
			this->meshes[i].Interpolation(delta, times);
		}
	}
	
private:	
	vector<Mesh> meshes;		//팔다리같은애들 멤버로
	string directory;		//경로
	vector<Texture> textures_loaded;
										// jpg 이런거
	void loadModel(string path)	//모델 불러오자.
	{
		Assimp::Importer importer;
		const aiScene *scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);
		//path 경로에있는 오브젝트를 Assimp::Importer를 통해 불러옴

		if (!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
		{
			cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << endl;
			return;
		}
		this->directory = path.substr(0, path.find_last_of('/'));
		//현재경로만 남겨놓는듯

		this->processNode(scene->mRootNode, scene);
		//root부터 자식들까지 다 모델링함
	}

	void processNode(aiNode* node, const aiScene* scene)
	{
		for (GLuint i = 0; i < node->mNumMeshes; i++)
		{
			aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
			//mesh 불러오고(팔다리)
			this->meshes.push_back(this->processMesh(mesh, scene, m_index));
			m_index++;
			//Model(최종 불러온 결과물. 예를들어 온전한 인간)의 멤버 meshes(팔다리 등)에 추가하는데
			//processMesh 실행. <-본격적으로 모델링시작하는부분
		}
		for (GLuint i = 0; i < node->mNumChildren; i++)
		{
			this->processNode(node->mChildren[i], scene);
		}
	}

	Mesh processMesh(aiMesh *mesh, const aiScene *scene, int index)
	{
		// Data to fill
		vector<Vertex> vertices;
		vector<GLuint> indices;
		vector<Texture> textures;
		for (GLuint i = 0; i < mesh->mNumVertices; i++)
		{
			Vertex vertex;
			glm::vec3 vector; 

							  // Positions
							  //이제 읽은 mesh를 본격적으로 입히는과정

			vector.x = mesh->mVertices[i].x;
			vector.y = mesh->mVertices[i].y;
			vector.z = mesh->mVertices[i].z;

			vertex.Position = vector;

			vector.x = mesh->mNormals[i].x;
			vector.y = mesh->mNormals[i].y;
			vector.z = mesh->mNormals[i].z;
			vertex.Normal = vector;
			if (mesh->mTextureCoords[0]) 
			{
				glm::vec2 vec;
				vec.x = mesh->mTextureCoords[0][i].x;
				vec.y = mesh->mTextureCoords[0][i].y;
				vertex.TexCoords = vec;
			}
			else
			{
				vertex.TexCoords = glm::vec2(0.0f, 0.0f);
			}

			//vertex(점)에 관한 모든것들 입력받고 vertices에 push
			vertices.push_back(vertex);
		}
		init_position[0] += 0.1f;
		for (GLuint i = 0; i < mesh->mNumFaces; i++)
		{
			aiFace face = mesh->mFaces[i];
			for (GLuint j = 0; j < face.mNumIndices; j++)
			{
				indices.push_back(face.mIndices[j]);
			}
		}
		if (mesh->mMaterialIndex >= 0)
		{
			aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];

			vector<Texture> diffuseMaps = this->loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
			textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());

			vector<Texture> specularMaps = this->loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
			textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
		}

		//완성된 Mesh 리턴함
		return Mesh(vertices, indices, textures, index);
	}
	vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type, string typeName)
	{

		vector<Texture> textures;
		for (GLuint i = 0; i < mat->GetTextureCount(type); i++)
		{
			aiString str;
			mat->GetTexture(type, i, &str);
			GLboolean skip = false;

			for (GLuint j = 0; j < textures_loaded.size(); j++)
			{
				if (textures_loaded[j].path == str)
				{
					textures.push_back(textures_loaded[j]);
					skip = true;

					break;
				}
			}

			if (!skip)
			{   
				Texture texture;
				texture.id = TextureFromFile(str.C_Str(), this->directory);
				texture.type = typeName;
				texture.path = str;
				textures.push_back(texture);
				this->textures_loaded.push_back(texture);
			}
		}

		return textures;
	}
};

GLint TextureFromFile(const char *path, string directory)
{
	string filename = string(path);
	filename = directory + '/' + filename;
	GLuint textureID;
	glGenTextures(1, &textureID);

	int width, height;

	unsigned char *image = SOIL_load_image(filename.c_str(), &width, &height, 0, SOIL_LOAD_RGB);

	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
	glGenerateMipmap(GL_TEXTURE_2D);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glBindTexture(GL_TEXTURE_2D, 0);
	SOIL_free_image_data(image);

	return textureID;
}



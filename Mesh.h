#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <Windows.h>

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "Model.h"
#include "textModel.h"

using namespace std;


struct Vertex
{
	// Position
	glm::vec3 Position;		//점 위치
	glm::vec3 Position_goal;
	// Normal
	glm::vec3 Normal;		//점의 법선벡터
							// TexCoords
	glm::vec2 TexCoords;	//텍스쳐상의 좌표
};

struct Texture		//jpg, png 이런거(mesh에 씌움.)
{
	GLuint id;		//파일이름
	string type;	//dispute(확산되는거) 또는 specular(반사되는거)
	aiString path;	//파일경로.
};

class Mesh		//예를들어 몸에서 팔다리 이런거 각각의 구조
{
public:
	vector<Vertex> vertices;	//점
	vector<GLuint> indices;		//인덱스인가??
	vector<Texture> textures;	//jpg 같이 mesh에 씌울 그림같은거
	vector<glm::vec3> init_vertex;
	vector<Vertex> ori_vert;
	vector<Vertex> goal_vert;

	int mesh_index;
	float interpolation_position;
	float weight_sum;
	glm::vec3 temp_position;
	Mesh() {

	}
	Mesh(vector<Vertex> vertices, vector<GLuint> indices, vector<Texture> textures, int index)
	{
		this->vertices = vertices;
		this->indices = indices;
		this->textures = textures;
		for (int i = 0; i < vertices.size(); i++) {
			this->init_vertex.push_back(vertices[i].Position);
		}
		this->mesh_index = index;
		this->setupMesh();
	}
	void Set_vertex(vector<Vertex> vertices) {
		this->ori_vert = vertices;
		this->goal_vert = vertices;
	}

	void blending(textModel * meshes, vector<float> weights, int x){
		weight_sum = 0.0f;
		for (int i = 0; i < weights.size(); i++) {
			weight_sum += weights[i];
		}
		this->ori_vert = this->vertices;
		for (int i = 0; i < this->vertices.size(); i++) {
			if (x == 0) {	//왼쪽 눈
				if (this->vertices[i].Position[0] >= 0.0f || this->vertices[i].Position[1] <= 12.0f)
					continue;
			}
			else if (x == 1) {	//오른쪽 눈
				if (this->vertices[i].Position[0] <= 0.0f || this->vertices[i].Position[1] <= 12.0f)
					continue;
			}
			else {	//입
				if (this->vertices[i].Position[1] > 12.0f)
					continue;
			}
			for (int j = 0; j < 3; j++) {
				interpolation_position = 0.0f;
				for (int k = 0; k < weights.size(); k++) {
					interpolation_position += meshes[k].position[i][j] * (weights[k] / weight_sum);
				}
				this->goal_vert[i].Position[j] = interpolation_position;
			}
		}
	}
	void Interpolation(float delta, float time) {
		for (int j = 0; j < this->vertices.size(); j++) {
			for (int k = 0; k < 3; k++) {
				this->vertices[j].Position[k] = (delta / time)* this->goal_vert[j].Position[k] + ((time - delta) / time) * this->ori_vert[j].Position[k];
			}
		}		
		glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(Vertex), &this->vertices[0], GL_DYNAMIC_DRAW);
	}
	void Draw(Shader shader)
	{
		GLuint diffuseNr = 1;
		GLuint specularNr = 1;

		for (GLuint i = 0; i < this->textures.size(); i++)
		{
			glActiveTexture(GL_TEXTURE0 + i); 
											  
			stringstream ss;
			string number;
			string name = this->textures[i].type;

			if (name == "texture_diffuse")
			{
				ss << diffuseNr++; // Transfer GLuint to stream
			}
			else if (name == "texture_specular")
			{
				ss << specularNr++;
			}

			number = ss.str();
			glBindTexture(GL_TEXTURE_2D, this->textures[i].id);
		}
		
		glUniform1f(glGetUniformLocation(shader.Program, "material.shininess"), 16.0f);
		
		glBindVertexArray(this->VAO);
		glDrawElements(GL_TRIANGLES, this->indices.size(), GL_UNSIGNED_INT, 0);	//mesh 그리자.
		glBindVertexArray(0);

		for (GLuint i = 0; i < this->textures.size(); i++)
		{
			glActiveTexture(GL_TEXTURE0 + i);
			glBindTexture(GL_TEXTURE_2D, 0);
		}
	}

private:
	GLuint VAO, VBO, EBO, Cur;
	// VAO = Vertex Array Object였나? VBO = Vertex Buffer .. EBO = Element Buffer 이런것
	void setupMesh()
	{
		// Create buffers/arrays
		glGenVertexArrays(1, &this->VAO);	//여기서 VAO 생성

		glBindVertexArray(this->VAO);		//만들고나서 bind 해주고(기다리고)
											// Load data into vertex buffers
		glGenBuffers(1, &this->VBO);
		glGenBuffers(1, &this->EBO);
		glBindBuffer(GL_ARRAY_BUFFER, this->VBO);
		glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(Vertex), &this->vertices[0], GL_DYNAMIC_DRAW);
		//여기서 이제 기다린거 실행
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->indices.size() * sizeof(GLuint), &this->indices[0], GL_DYNAMIC_DRAW);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid *)offsetof(Vertex, Position));
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid *)offsetof(Vertex, Normal));

		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid *)offsetof(Vertex, TexCoords));

		glBindVertexArray(0);

	}
};



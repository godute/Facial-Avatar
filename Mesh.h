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
	glm::vec3 Position;		//�� ��ġ
	glm::vec3 Position_goal;
	// Normal
	glm::vec3 Normal;		//���� ��������
							// TexCoords
	glm::vec2 TexCoords;	//�ؽ��Ļ��� ��ǥ
};

struct Texture		//jpg, png �̷���(mesh�� ����.)
{
	GLuint id;		//�����̸�
	string type;	//dispute(Ȯ��Ǵ°�) �Ǵ� specular(�ݻ�Ǵ°�)
	aiString path;	//���ϰ��.
};

class Mesh		//������� ������ �ȴٸ� �̷��� ������ ����
{
public:
	vector<Vertex> vertices;	//��
	vector<GLuint> indices;		//�ε����ΰ�??
	vector<Texture> textures;	//jpg ���� mesh�� ���� �׸�������
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
			if (x == 0) {	//���� ��
				if (this->vertices[i].Position[0] >= 0.0f || this->vertices[i].Position[1] <= 12.0f)
					continue;
			}
			else if (x == 1) {	//������ ��
				if (this->vertices[i].Position[0] <= 0.0f || this->vertices[i].Position[1] <= 12.0f)
					continue;
			}
			else {	//��
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
		glDrawElements(GL_TRIANGLES, this->indices.size(), GL_UNSIGNED_INT, 0);	//mesh �׸���.
		glBindVertexArray(0);

		for (GLuint i = 0; i < this->textures.size(); i++)
		{
			glActiveTexture(GL_TEXTURE0 + i);
			glBindTexture(GL_TEXTURE_2D, 0);
		}
	}

private:
	GLuint VAO, VBO, EBO, Cur;
	// VAO = Vertex Array Object����? VBO = Vertex Buffer .. EBO = Element Buffer �̷���
	void setupMesh()
	{
		// Create buffers/arrays
		glGenVertexArrays(1, &this->VAO);	//���⼭ VAO ����

		glBindVertexArray(this->VAO);		//������� bind ���ְ�(��ٸ���)
											// Load data into vertex buffers
		glGenBuffers(1, &this->VBO);
		glGenBuffers(1, &this->EBO);
		glBindBuffer(GL_ARRAY_BUFFER, this->VBO);
		glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(Vertex), &this->vertices[0], GL_DYNAMIC_DRAW);
		//���⼭ ���� ��ٸ��� ����
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



/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Loader3DS.h
*
* Change Log:
*      Date                Who             What
*      2015/02/15         Xin Shao        Create
*******************************************************************************
*/
#pragma once

#include <fstream>
#include "Model3DS.h"

class Loader3DS
{
public:
Loader3DS();
void               OpenFile( const std::string& );
void               LoadFile();
void               CloseFile();
const Model&       GetModel();
private:
void               LoadModel( const Chunk& );

void               LoadMaterial( const Chunk& );
void               LoadColor( float* );
void               LoadPercent( float* );

void               LoadObject( const Chunk& );
void               LoadVertex( Object* const& );
void               LoadFaces( Object* );
void               LoadObjectMaterial( Object* );
void               LoadMesh( const Chunk& MyChunk );
private:
Vertex             Vectors( const Vertex&, const Vertex& );
Vertex             Cross( const Vertex&, const Vertex& );
void               Normalize( Vertex* Point );
void               ComputeNormals();
private:
void               ReadChunk( Chunk* MyChunk );
void               ReadGLfloat( GLfloat* );
void               ReadGLushort( GLushort* );
void               ReadGLuint( GLuint* );
void               ReadGLubyte( GLubyte* );
void               SkipNByte( const size_t& );
std::string        ReadString();
private:
size_t             NowPos;
size_t             FileLength;
size_t             Version;
Model              MyModel;
std::ifstream      FileReader;
};
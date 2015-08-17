/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Loader3DS.cpp
* @brief a class to read a 3ds file.
*
* Change Log:
*      Date                Who             What
*      2015/02/15         Xin Shao        Create
*******************************************************************************
*/

#include <stdexcept>
#include <cmath>
#include "Loader3DS.h"
#include "3dsId.h"
#include "LogInfo.h"

using namespace std;

Loader3DS::Loader3DS() : NowPos( 0 ),FileLength( 0 ), Version( 0 )
{}
void Loader3DS::OpenFile( const string& FileRoad )
{
locale::global( locale("") );
FileReader.open( FileRoad.c_str(), ifstream::binary );
locale::global( locale("C") );
if( !FileReader )
	logPrintf(logLevelError_e, "VISUAL", "Open file failed!");
}

void Loader3DS::CloseFile()
{
FileReader.close();
}

void Loader3DS::LoadFile()
{
Chunk MyChunk;
ReadChunk( &MyChunk );
if( PRIMARY != MyChunk.ID )
	logPrintf(logLevelError_e, "VISUAL", "The file is destroied!");
FileLength = MyChunk.Len;

ReadChunk( &MyChunk );
ReadGLuint( &Version );
while( NowPos < FileLength )
{
   ReadChunk( &MyChunk ); 
   if( MAINOBJECT == MyChunk.ID )
    LoadModel( MyChunk );
   else
    SkipNByte( MyChunk.Len - 6 );
}

ComputeNormals();
}

void Loader3DS::LoadModel( const Chunk& MyChunk )
{
size_t BeginPos( NowPos - 6 );
Chunk TempChunk;
while( NowPos - BeginPos != MyChunk.Len )
{
   ReadChunk( &TempChunk );
   switch( TempChunk.ID )
   {
   case OBJECT:
    LoadObject( TempChunk );
    break;

   case MATERIAL:
    LoadMaterial( TempChunk );
    break;

   default:
    SkipNByte( TempChunk.Len - 6 );
   }
}
}

void Loader3DS::LoadObject( const Chunk& MyChunk )
{
Object object;
object.Name = ReadString();
MyModel.MyObject.push_back( object );

Chunk ThisChunk;
size_t BeginPos( NowPos - 7 - object.Name.size() );
while( NowPos - BeginPos != MyChunk.Len )
{
   ReadChunk( &ThisChunk );
   if( OBJECT_MESH == ThisChunk.ID )
    LoadMesh( ThisChunk );
   else
    SkipNByte( ThisChunk.Len - 6 );
}
}

void Loader3DS::LoadMesh( const Chunk& MyChunk )
{
Object &object = MyModel.MyObject[ MyModel.MyObject.size() - 1 ];

size_t BeginPos( NowPos - 6 );
Chunk ThisChunk;
while( NowPos - BeginPos != MyChunk.Len )
{
   ReadChunk( &ThisChunk );
   switch( ThisChunk.ID )
   {
   case OBJECT_VERTICES: //vertex
    LoadVertex( &object );
    break;

   case OBJECT_FACES:     //face
    LoadFaces( &object );
    break;

   case OBJECT_MATERIAL: //material
    LoadObjectMaterial( &object );
    break;

   default:              
    SkipNByte( ThisChunk.Len - 6 );
   }
}
}

void Loader3DS::LoadObjectMaterial( Object* object )
{
string Name = ReadString();
int Pos( -1 );
for( size_t i = 0; i != MyModel.MyMaterial.size(); ++ i )
{
   if( MyModel.MyMaterial[ i ].name == Name )
    Pos = i;
}

if( Pos == -1 )
	logPrintf(logLevelError_e, "VISUAL", "Don't find the material!");

GLushort Sum( 0 ); GLushort FacePos( 0 );
ReadGLushort( &Sum );
for( size_t i = 0; i != Sum; ++ i )
{
   ReadGLushort( &FacePos );
   object->Faces[ FacePos ].MaterialPos = Pos;
}
}

void Loader3DS::LoadMaterial( const Chunk& MyChunk )
{

Chunk TempChunk;
Material material;
size_t BeginPos( NowPos - 6 );

while( NowPos - BeginPos < MyChunk.Len )
{
   ReadChunk( &TempChunk );
   switch( TempChunk.ID )
   {
   case MATNAME:                       //material name
    material.name = ReadString();
    break;
   case MAT_AMBIENT:                  //material ambient
    LoadColor( material.ambientColor );
    break;
   case MAT_DIFFUSE:                  //material diffuse
    LoadColor( material.diffuseColor );
    break;
   case MAT_SPECULAR:                 //material specular
    LoadColor( material.specularColor );
    break;
   case MAT_SHININESS:                //material shininess
    LoadPercent( &material.shininess );
    break;
   case MAT_TRANSPARENCY:             //material transparency
    LoadPercent( &material.transparency );
    break;
   default:
    SkipNByte( TempChunk.Len - 6 );
   }
}
MyModel.MyMaterial.push_back( material );
}

void Loader3DS::LoadColor( float* color )
{
Chunk TempChunk;
ReadChunk( &TempChunk );
switch( TempChunk.ID )
{
case COLOR_F:
   ReadGLfloat( &color[ 0 ] );
   ReadGLfloat( &color[ 1 ] );
   ReadGLfloat( &color[ 2 ] );
   break;
case COLOR_24:
   GLubyte Byte;
   for( size_t i = 0; i != 3; ++ i )
   {
    ReadGLubyte( &Byte );
    color[ i ] = Byte / 255.0;
   }
   break;
default:
   SkipNByte( TempChunk.Len - 6 );
}
}

void Loader3DS::LoadPercent( GLfloat* Temp )
{
Chunk TempChunk;
ReadChunk( &TempChunk );
switch( TempChunk.ID )
{
case INT_PERCENTAGE:    //Int percent
   GLushort Percent;
   ReadGLushort( &Percent );
   *Temp = Percent / 100.0;
   break;
case FLOAT_PERCENTAGE: //Float percent
   ReadGLfloat( Temp );
   break;
default:
   SkipNByte( TempChunk.Len - 6 );
}
}

Vertex Loader3DS::Vectors( const Vertex& lPoint, const Vertex& rPoint )
{
Vertex Point;
Point.x = lPoint.x - rPoint.x;
Point.y = lPoint.y - rPoint.y;
Point.z = lPoint.z - rPoint.z;
return Point;
}

Vertex Loader3DS::Cross( const Vertex& lPoint, const Vertex& rPoint )
{
Vertex Point;
Point.x = lPoint.y * rPoint.z - lPoint.z * rPoint.y;
Point.y = lPoint.z * rPoint.x - lPoint.x * rPoint.z;
Point.z = lPoint.x * rPoint.y - lPoint.y * rPoint.x;
return Point;
}

void Loader3DS::Normalize( Vertex* point )
{
float Magnitude = sqrt( point->x * point->x + point->y * point->y + point->z * point->z );
if( 0 == Magnitude )
   Magnitude = 1;
point->x /= Magnitude;    
point->y /= Magnitude;    
point->z /= Magnitude;           
}

void Loader3DS::ComputeNormals()
{
for( size_t i = 0; i != MyModel.MyObject.size(); ++ i )
{
   Object& object = MyModel.MyObject[ i ];
   for( size_t j = 0; j != MyModel.MyObject[ i ].Faces.size(); ++ j )
   {
    Face& face = object.Faces[ j ];
    const Vertex &Point1 = object.Vertexs[ face.Index[ 0 ] ];
    const Vertex &Point2 = object.Vertexs[ face.Index[ 1 ] ];
    const Vertex &Point3 = object.Vertexs[ face.Index[ 2 ] ];

    face.Normal = Cross( Vectors( Point1, Point3 ), Vectors( Point3, Point2 ) );
    Normalize( &face.Normal );
   }
}
}

const Model& Loader3DS::GetModel()
{
return MyModel;
}

void Loader3DS::LoadFaces( Object* ThisObject )
{
GLushort Sum( 0 );
ReadGLushort( &Sum );
Face face; GLushort Temp( 0 );
for( size_t i = 0; i != Sum; ++ i )
{
   for( size_t j = 0; j != 4; ++ j )
   {
    ReadGLushort( &Temp );
    if( j < 3 )
     face.Index[ j ] = Temp;
   }
   ThisObject->Faces.push_back( face );
}
}

void Loader3DS::LoadVertex( Object* const& ThisObject )
{
GLushort Sum( 0 );
ReadGLushort( &Sum );
Vertex Point;
float Num( 0 );float distence( 0 );
for( size_t i = 0; i != Sum; ++ i )
{

   ReadGLfloat( &Point.x );
   ReadGLfloat( &Point.z );
   ReadGLfloat( &Point.y );
   Point.z *= -1;
   ThisObject->Vertexs.push_back( Point );
}
}

void Loader3DS::ReadChunk( Chunk* MyChunk )
{
ReadGLushort( &MyChunk->ID );
ReadGLuint( &MyChunk->Len );
}

void Loader3DS::ReadGLubyte( GLubyte* Ubyte )
{
FileReader.read( reinterpret_cast< char* >( Ubyte ), sizeof( GLubyte ) );
NowPos += sizeof( GLubyte );
}

void Loader3DS::ReadGLushort( GLushort* Ushort )
{
FileReader.read( reinterpret_cast< char* >( Ushort ), sizeof( GLushort ) );
NowPos += sizeof( GLushort );
}

void Loader3DS::ReadGLuint( GLuint* Uint )
{
FileReader.read( reinterpret_cast< char* >( Uint ), sizeof( GLuint ) );
NowPos += sizeof( GLuint );
}

void Loader3DS::ReadGLfloat( GLfloat* Float )
{
FileReader.read( reinterpret_cast< char* >( Float ), sizeof( GLfloat ) );
NowPos += sizeof( GLfloat );
}

std::string Loader3DS::ReadString()
{
char alpha; string TempWord;
while( FileReader.get( alpha ), alpha != 0 )
   TempWord += alpha;
NowPos += TempWord.size() + 1;
return TempWord;
}

void Loader3DS::SkipNByte( const size_t& Num )
{
FileReader.seekg( Num, ifstream::cur );
NowPos += Num;
}
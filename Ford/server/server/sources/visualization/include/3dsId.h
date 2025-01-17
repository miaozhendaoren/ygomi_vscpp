/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  3dsId.h
*
* Change Log:
*      Date                Who             What
*      2015/02/15         Xin Shao        Create
*******************************************************************************
*/
#pragma once

const GLsizei   PRIMARY             = 0x4D4D;
const GLsizei   MAINOBJECT          = 0x3D3D;     //version 
const GLsizei   EDITKEYFRAME        = 0xB000;    
const GLsizei   MATERIAL         = 0xAFFF;     
const GLsizei   OBJECT          = 0x4000;     
const GLsizei   MATNAME             = 0xA000;     
const GLsizei   OBJECT_MESH         = 0x4100;     
const GLsizei   OBJECT_VERTICES     = 0x4110;    
const GLsizei   OBJECT_FACES     = 0x4120;    
const GLsizei   OBJECT_MATERIAL     = 0x4130;    
const GLsizei MAT_AMBIENT    = 0xa010;
const GLsizei MAT_DIFFUSE    = 0xa020;
const GLsizei MAT_SPECULAR   = 0xa030;
const GLsizei MAT_SHININESS   = 0xa040;
const GLsizei MAT_TRANSPARENCY = 0xa050;
const GLsizei INT_PERCENTAGE   = 0x0030;
const GLsizei FLOAT_PERCENTAGE = 0x0031;
const GLsizei COLOR_F     = 0x0010;
const GLsizei COLOR_24    = 0x0011;
